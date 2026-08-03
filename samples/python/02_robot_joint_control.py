#!/usr/bin/env python3
# =============================================================================
# Corium SimLab Sample #02 — Python Joint Control & Offscreen Visual Sensor
# Explicit User-Defined Workstation Scene Construction
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

    # User explicitly constructs 3D workstation scene via SceneBuilder
    builder = app.create_scene_builder()
    scene = (builder
             .add_ground_grid(60.0, 60.0, 60)
             .add_cube("workstation_table", corium_sim_py.Vec3(0.0, 0.4, 0.0), corium_sim_py.Vec3(3.0, 0.8, 2.0), is_static=True)
             .add_cube("agent_robot", corium_sim_py.Vec3(0.0, 0.8, 0.0), corium_sim_py.Vec3(0.8, 0.8, 0.8), is_static=True)
             .add_cube("shoulder_link", corium_sim_py.Vec3(0.0, 1.4, 0.0), corium_sim_py.Vec3(0.4, 1.0, 0.4))
             .add_cube("elbow_link", corium_sim_py.Vec3(0.0, 2.3, 0.0), corium_sim_py.Vec3(0.3, 0.8, 0.3))
             .add_cube("wrist_link", corium_sim_py.Vec3(0.0, 2.9, 0.0), corium_sim_py.Vec3(0.25, 0.4, 0.25))
             .add_joint("joint_shoulder_yaw", "agent_robot", "shoulder_link", corium_sim_py.JointType.Revolute, corium_sim_py.Vec3(0.0, 0.6, 0.0), corium_sim_py.Vec3(0.0, 1.0, 0.0), -3.14, 3.14)
             .add_joint("joint_elbow_pitch", "shoulder_link", "elbow_link", corium_sim_py.JointType.Revolute, corium_sim_py.Vec3(0.0, 0.9, 0.0), corium_sim_py.Vec3(1.0, 0.0, 0.0), -2.09, 2.09)
             .add_joint("joint_wrist_roll", "elbow_link", "wrist_link", corium_sim_py.JointType.Revolute, corium_sim_py.Vec3(0.0, 0.6, 0.0), corium_sim_py.Vec3(0.0, 1.0, 0.0), -3.14, 3.14)
             .add_cube("target_workpiece", corium_sim_py.Vec3(3.5, 1.25, -1.5), corium_sim_py.Vec3(0.6, 0.5, 0.6))
             .build())

    app.set_scene(scene)
    app.reset()

    print("[Python Sample] Controlling Robotic Arm Joints...")
    app.set_joint_position("joint_shoulder_yaw", 0.7854)   # +45 deg Yaw
    app.set_joint_position("joint_elbow_pitch", -0.5236)   # -30 deg Pitch
    app.set_joint_position("joint_wrist_roll",  1.5708)   # +90 deg Roll

    # Step simulation to advance kinematics and physics
    app.sim_step(0.016667)

    obs = app.get_observation()
    sensor_bytes = app.get_sensor_frame()

    print("  - Observation Dict Keys:", list(obs.keys()))
    if obs:
        print(f"  - Agent Position: {obs.get('agent_pos')}")
        print(f"  - Target Position: {obs.get('target_pos')}")
        print(f"  - Distance to Target Workpiece: {obs.get('distance'):.4f} m")
    print("  - Offscreen Sensor Frame Byte Size:", len(sensor_bytes), "bytes (128x128 RGBA)")
    app.save_sensor_frame_ppm("agent_view.ppm")
    print("  - Saved onboard agent 3D camera view to file: agent_view.ppm")

    print("\nCorium SimLab Python Robotic Joint Control sample completed successfully!")

if __name__ == "__main__":
    main()
