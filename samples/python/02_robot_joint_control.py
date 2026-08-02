#!/usr/bin/env python3
# =============================================================================
# Corium SimLab Sample #02 — Python Joint Control & Offscreen Visual Sensor
# =============================================================================

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "python")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "build")))

try:
    import corium_sim_py
except ImportError:
    print("[Error] Could not find compiled C++ extension module corium_sim_py!")
    sys.exit(1)

def main():
    print("=========================================================")
    print(" Corium SimLab Sample #02: Python Robotic Joint Control")
    print("=========================================================\n")

    app = corium_sim_py.SimLabApp()
    app.reset()

    print("[Python Sample] Controlling Robotic Arm Joints...")
    app.set_joint_position("joint_shoulder_yaw", 0.7854)   # +45 deg Yaw
    app.set_joint_position("joint_elbow_pitch", -0.5236)   # -30 deg Pitch
    app.set_joint_position("joint_wrist_roll",  1.5708)   # +90 deg Roll

    obs = app.get_observation()
    sensor_bytes = app.get_sensor_frame()

    print("  - Observation Dict Keys:", list(obs.keys()))
    print("  - Offscreen Sensor Frame Byte Size:", len(sensor_bytes), "bytes (128x128 RGBA)")

    print("\nCorium SimLab Python Robotic Joint Control sample completed successfully!")

if __name__ == "__main__":
    main()
