
import pyrealsense2 as rs
import time

SERIALS = ['151322062583', '151422060684', '146222254752']
pipelines = []

for serial in SERIALS:
    p = rs.pipeline()
    c = rs.config()
    c.enable_device(serial)
    c.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    p.start(c)
    pipelines.append((serial, p))

print("Warming up 2s ...")
time.sleep(2)

print("Counting frames for 10s ...")
counts = {s: 0 for s in SERIALS}
start = time.time()

while time.time() - start < 10:
    for serial, p in pipelines:
        try:
            frames = p.wait_for_frames(timeout_ms=50)
            if frames.get_color_frame():
                counts[serial] += 1
        except RuntimeError:
            pass

for serial, count in counts.items():
    print(f"  {serial}: {count} frames = {count/10:.1f} fps")

for serial, p in pipelines:
    p.stop()
