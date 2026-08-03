#!/usr/bin/env python3
# =============================================================================
# Corium SimLab Sample #03 — Autonomous Agent RL & Inverse Kinematics Target Tracking
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
    print("=========================================================================")
    print(" Corium SimLab Sample #03: Autonomous Agent & Inverse Kinematics Tracking")
    print("=========================================================================\n")

    app = corium_sim_py.SimLabApp()

    # User explicitly constructs 3D workstation scene via SceneBuilder
    builder = app.create_scene_builder()
    scene = (builder
             .add_ground_grid(60.0, 60.0, 60)
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

    print("[Python Sample] Robotic Arm Workstation Environment Loaded Successfully!")
    print("[Python Sample] Target Workpiece Coordinates: (3.5, 1.25, -1.5)")

    # 1. Solve Inverse Kinematics to reach target workpiece
    print("\n[Python Sample] Solving Inverse Kinematics for End-Effector 'wrist_link'...")
    ik_success = app.solve_ik("wrist_link", target_x=3.5, target_y=1.25, target_z=-1.5, max_iterations=100, tolerance=0.05)

    if ik_success:
        print("  - IK Solution Converged Successfully!")
    else:
        print("  - IK Solver finished (Closest joint configuration reached within limits)")

    # 2. Advance physics & kinematics step
    app.sim_step(0.016667)
    obs = app.get_observation()

    if obs:
        print(f"  - Updated End-Effector / Agent Position: {obs.get('agent_pos')}")
        print(f"  - Target Workpiece Position:           {obs.get('target_pos')}")
        print(f"  - Remaining Distance to Target:        {obs.get('distance'):.4f} m")

    # 3. Render and save offscreen sensor camera view
    app.save_sensor_frame_ppm("ik_robot_reach_view.ppm")
    print("  - Saved onboard agent visual sensor 3D view to file: ik_robot_reach_view.ppm")

    print("\nCorium SimLab Sample #03 completed successfully!")

if __name__ == "__main__":
    main()
