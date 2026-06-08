"""
wsg_tcp.py
----------
Pure-Python GCL (Gripper Control Language) TCP driver for the Weiss WSG 32.

No ROS dependency — this layer only knows about sockets and the GCL protocol.
The ROS2 node (wsg32_node.py) imports this class.

PREREQUISITE (one-time hardware setup):
    Open http://<gripper_ip> in a browser.
    Go to Settings → Command Interface → enable "Use text based interface".
    This setting persists across reboots.

GCL protocol basics:
    - All commands are ASCII strings terminated with \\n
    - Gripper responds with "ACK <CMD>\\n" (immediate) or "FIN <CMD>\\n" (motion done)
    - On error: "ERR <CODE>\\n"
    - Default IP: 192.168.1.20  |  Default port: 1000

Adapted from: https://github.com/real-stanford/minWSG (MIT License)
Extended with: non-blocking move, position query, connection guards.
"""

import socket
import threading
from time import time, sleep


class WSG32TCP:
    """
    Thread-safe GCL TCP driver for the Weiss WSG 32 gripper.

    Key design decisions:
    - TCP_NODELAY is set immediately — kills Nagle algorithm, no buffering latency.
    - move_nonblocking() sends MOVE and returns without waiting for FIN.
      This is what you want during teleoperation: the leader keeps streaming
      new positions and the gripper tracks continuously.
    - move() is the blocking version — use it for scripted sequences.
    - A background reader thread is NOT used here (keepit simple).
      For position-controlled teleoperation, fire-and-forget is enough.
    """

    # Gripper physical limits for WSG 32
    MIN_POS_MM = 0.0    # fully closed
    MAX_POS_MM = 66.0   # fully open (WSG 32 stroke = 66 mm, NOT 110)
                        # NOTE: WSG 50 stroke is 110 mm. Verify with your hardware.
                        # Check Settings → System Info in the web UI.

    def __init__(self,
                 ip: str = "192.168.1.201",
                 port: int = 1000,
                 timeout: float = 5.0):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()   # one command at a time

    # ──────────────────────────────────────────────────────────────────────
    # Connection
    # ──────────────────────────────────────────────────────────────────────

    def connect(self) -> None:
        """
        Open TCP connection and clear any latched fault.
        Raises ConnectionError if the gripper is unreachable.
        """
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # TCP_NODELAY: send each command immediately, do not wait to batch.
        # Without this you get random ~40 ms delays from Nagle buffering.
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock.settimeout(self.timeout)
        try:
            self._sock.connect((self.ip, self.port))
        except (socket.error, OSError) as e:
            self._sock = None
            raise ConnectionError(
                f"WSG32: Cannot reach {self.ip}:{self.port} — {e}\n"
                f"Check: (1) gripper powered, (2) alias IP 192.168.1.100 up, "
                f"(3) cable to switch."
            )
        # Clear any latched fast-stop from a previous crash
        self.ack_fault()

    def disconnect(self) -> None:
        """Gracefully close the TCP session."""
        if self._sock is not None:
            try:
                self._send_raw("BYE()\n")
            except Exception:
                pass
            self._sock.close()
            self._sock = None

    def is_connected(self) -> bool:
        return self._sock is not None

    # ──────────────────────────────────────────────────────────────────────
    # Low-level send / receive
    # ──────────────────────────────────────────────────────────────────────

    def _send_raw(self, cmd: str) -> None:
        """Send a GCL command string. Thread-safe."""
        with self._lock:
            self._sock.sendall(cmd.encode("ascii"))

    def _recv_line(self) -> str:
        """
        Read bytes until \\n.  Returns the line as a decoded string.
        Simple and correct for GCL — every response is one line.
        """
        buf = b""
        while True:
            chunk = self._sock.recv(256)
            if not chunk:
                raise ConnectionError("WSG32: socket closed by gripper")
            buf += chunk
            if b"\n" in buf:
                line = buf.split(b"\n")[0]
                return line.decode("ascii", errors="replace").strip()

    def _send_and_wait(self, cmd: str, expected_prefix: str) -> bool:
        """
        Send command, block until a line starting with expected_prefix arrives.
        Returns True on success, False on ERR response.
        Raises TimeoutError if nothing arrives within self.timeout seconds.
        """
        self._send_raw(cmd)
        deadline = time() + self.timeout
        while time() < deadline:
            line = self._recv_line()
            if line.startswith(expected_prefix):
                return True
            if line.startswith("ERR"):
                return False
        raise TimeoutError(
            f"WSG32: timed out waiting for '{expected_prefix}' "
            f"after sending: {cmd.strip()!r}"
        )

    # ──────────────────────────────────────────────────────────────────────
    # GCL Commands
    # ──────────────────────────────────────────────────────────────────────

    def ack_fault(self) -> bool:
        """
        Acknowledge / clear a latched fast-stop or fault.
        Always call this on connect, and after any E-stop event.
        GCL: FSACK()  →  response: ACK FSACK
        """
        return self._send_and_wait("FSACK()\n", "ACK FSACK")

    def home(self) -> bool:
        """
        Run homing sequence: gripper opens fully to find the reference position.
        MUST be called at least once after power-on before any MOVE command.
        Blocks until homing completes (~3–5 seconds).
        GCL: HOME()  →  response: FIN HOME
        """
        return self._send_and_wait("HOME()\n", "FIN HOME")

    def move(self, position_mm: float) -> bool:
        """
        Move fingers to position_mm. Blocks until motion completes.
        position_mm: 0.0 = fully closed, MAX_POS_MM = fully open.

        Use this for scripted sequences. For live teleoperation, use
        move_nonblocking() instead.

        GCL: MOVE(<pos>)  →  response: FIN MOVE
        """
        position_mm = self._clamp(position_mm)
        cmd = f"MOVE({position_mm:.2f})\n"
        return self._send_and_wait(cmd, "FIN MOVE")

    def move_nonblocking(self, position_mm: float) -> None:
        """
        Send a MOVE command and return immediately WITHOUT waiting for FIN MOVE.

        This is the correct call for teleoperation:
        - The leader robot continuously streams new target positions.
        - We fire each MOVE and immediately return so the ROS subscriber
          can accept the next command.
        - The gripper's internal controller tracks the target.
        - The FIN MOVE response will arrive in the socket buffer later;
          we intentionally do NOT read it here (it would accumulate but
          the next MOVE resets gripper motion anyway).

        WARNING: do not mix move_nonblocking() and move() in the same session
        without flushing, because unread FIN MOVE bytes will accumulate.
        Pick one mode and stick with it.
        """
        position_mm = self._clamp(position_mm)
        cmd = f"MOVE({position_mm:.2f})\n"
        self._send_raw(cmd)

    def release(self, open_mm: float = 10.0) -> bool:
        """
        Open fingers by open_mm to release a grasped object.
        GCL: RELEASE(<mm>)  →  response: FIN RELEASE
        """
        return self._send_and_wait(f"RELEASE({open_mm:.1f})\n", "FIN RELEASE")

    def stop(self) -> None:
        """
        Immediately stop all finger motion.
        GCL: STOP()  →  response: ACK STOP
        """
        self._send_raw("STOP()\n")

    # ──────────────────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────────────────

    def _clamp(self, position_mm: float) -> float:
        return max(self.MIN_POS_MM, min(self.MAX_POS_MM, position_mm))

    def normalize_to_pos(self, value_0_to_1: float) -> float:
        """
        Convert a normalized [0.0, 1.0] leader value to gripper mm.
        0.0 → fully closed (0 mm)
        1.0 → fully open (MAX_POS_MM)
        Useful if your GELLO outputs a normalized joint angle.
        """
        return value_0_to_1 * self.MAX_POS_MM
