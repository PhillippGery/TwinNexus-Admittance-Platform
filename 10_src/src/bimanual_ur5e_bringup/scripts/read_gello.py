#!/usr/bin/env python3
import sys
import os
import time

# Inject your GELLO software path
GELLO_PATH = os.path.expanduser("~/TwinNexus-Admittance-Platform/10_src/src/gello_software")
if GELLO_PATH not in sys.path:
    sys.path.insert(0, GELLO_PATH)

from gello.agents.gello_agent import GelloAgent, DynamixelRobotConfig

# The exact left arm U2D2 port you isolated
PORT = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT89FD08-if00-port0'
PORT =  'dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO4WDM-if00-port0'  # Update this to your actual port
def main():
    print(f"Pinging Dynamixel bus on {PORT.split('/')[-1]}...")
    
    # Blank slate: no offsets, positive signs, raw data only
    config = DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        joint_signs=(1, 1, 1, 1, 1, 1),
        gripper_config=(7, 0.0, 0.0),
    )

    try:
        agent = GelloAgent(port=PORT, dynamixel_config=config, start_joints=None)
        print("SUCCESS: Hardware alive. Streaming data (Ctrl+C to quit).\n")
        
        while True:
            state = agent.act({})
            
            # Format output: J1 to J6 + Gripper
            out = " | ".join([f"J{i+1}: {val:+.3f}" for i, val in enumerate(state[:6])])
            out += f" | Grip: {state[6]:+.3f}"
            
            # Print over the same line for a clean terminal dashboard
            print(f"\r{out}", end="", flush=True)
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\nDisconnected.")
    except Exception as e:
        print(f"\nFATAL ERROR: Bus communication failed. Check your wiring.\n{e}")

if __name__ == '__main__':
    main()