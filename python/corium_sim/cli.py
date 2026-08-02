#!/usr/bin/env python3
import sys
import os
import argparse

def main():
    parser = argparse.ArgumentParser(
        prog="corium-sim",
        description="Corium SimLab 3D Simulation & WebGPU Engine CLI Tool"
    )
    subparsers = parser.add_subparsers(dest="command", help="Available Commands")

    # Command: info
    info_parser = subparsers.add_parser("info", help="Display Corium SimLab system, GPU backend, and version info")

    # Command: view
    view_parser = subparsers.add_parser("view", help="Preview a 3D URDF or OBJ model file")
    view_parser.add_argument("file", type=str, help="Path to .urdf or .obj model file")

    # Command: run
    run_parser = subparsers.add_parser("run", help="Run a Corium SimLab sample script (1-6)")
    run_parser.add_argument("sample", type=int, choices=range(1, 7), help="Sample number (1 to 6)")

    args = parser.parse_args()

    if args.command == "info":
        print("=========================================================================")
        print(" Corium SimLab — High-Performance 3D Agent Simulation System")
        print("=========================================================================")
        print(" Version:            0.2.0")
        print(" Architecture:       C++20 & WebGPU WGSL Shading Engine")
        print(" Event Runtime:      Corium MPSC Zero-Heap Architecture")
        print(" Features Supported: Offscreen WebGPU RGBA Sensors, Inverse Kinematics,")
        print("                     URDF Parser, 3D LiDAR Raycasting, VectorEnv")
        print("=========================================================================")

    elif args.command == "view":
        file_path = os.path.abspath(args.file)
        if not os.path.exists(file_path):
            print(f"[Error] Target file not found: {file_path}")
            sys.exit(1)

        print(f"[Corium SimLab CLI] Previewing 3D Model: {file_path}")
        try:
            import corium_sim_py
            app = corium_sim_py.SimLabApp()
            if file_path.endswith(".urdf"):
                app.load_urdf(file_path)
            else:
                app.load_scene_mesh(file_path)
            app.reset()
            app.sim_step(0.016667)
            out_ppm = "model_preview.ppm"
            app.save_sensor_frame_ppm(out_ppm)
            print(f"[Success] Saved 3D offscreen view preview to: {out_ppm}")
        except Exception as e:
            print(f"[Error] Failed to render model preview: {e}")

    elif args.command == "run":
        sample_files = {
            1: "01_gym_environment.py",
            2: "02_robot_joint_control.py",
            3: "03_train_rl_agent.py",
            4: "04_urdf_robot_loader.py",
            5: "05_lidar_sensor_scan.py",
            6: "06_vectorized_environments.py"
        }
        sample_name = sample_files.get(args.sample)
        sample_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "samples", "python", sample_name))
        
        if os.path.exists(sample_path):
            print(f"[Corium SimLab CLI] Launching Sample #{args.sample}: {sample_name}\n")
            os.system(f"python3 {sample_path}")
        else:
            print(f"[Error] Sample file not found: {sample_path}")

    else:
        parser.print_help()

if __name__ == "__main__":
    main()
