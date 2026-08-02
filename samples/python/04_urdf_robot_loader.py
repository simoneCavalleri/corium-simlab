#!/usr/bin/env python3
# =============================================================================
# Corium SimLab Sample #04 — URDF Robot Model Loading & Kinematic Control
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
    print("=========================================================================")
    print(" Corium SimLab Sample #04: URDF Robot Specification Parser & Loader")
    print("=========================================================================\n")

    app = corium_sim_py.SimLabApp()

    urdf_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "assets", "urdf", "sample_arm.urdf"))
    print(f"[Python Sample] Loading URDF Specification File: {urdf_path}")

    success = app.load_urdf(urdf_path)
    if success:
        print("[Python Sample] URDF Robot Model Loaded Successfully!")
    else:
        print("[Error] Failed to load URDF model file!")
        sys.exit(1)

    app.reset()

    # Control URDF articulated joints
    print("\n[Python Sample] Setting URDF Joint Angles...")
    app.set_joint_position("joint_shoulder_yaw", 0.5236)  # +30 deg
    app.set_joint_position("joint_elbow_pitch", -0.7854) # -45 deg

    app.sim_step(0.016667)

    # Save visual frame
    app.save_sensor_frame_ppm("urdf_robot_view.ppm")
    print("  - Saved URDF Robot 3D Offscreen View to file: urdf_robot_view.ppm")

    print("\nCorium SimLab Sample #04 completed successfully!")

if __name__ == "__main__":
    main()
