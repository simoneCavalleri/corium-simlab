#!/usr/bin/env python3
import sys
import os
import argparse

def main():
    parser = argparse.ArgumentParser(
        prog="corium-sim",
        description="Corium SimLab — High-Performance Physical Agent Incubator & Simulation System CLI Tool"
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

    # Command: visualizer
    vis_parser = subparsers.add_parser("visualizer", help="Launch live Web Telemetry & 3D LiDAR Visualizer Dashboard in browser")

    args = parser.parse_args()

    if args.command == "info":
        print("=========================================================================")
        print(" Corium SimLab — High-Performance Physical Agent Incubator System")
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

    elif args.command == "visualizer":
        vis_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        if os.path.exists(vis_dir):
            import http.server
            import socketserver
            import webbrowser

            port = 8080
            class SimLabHttpHandler(http.server.SimpleHTTPRequestHandler):
                def do_GET(self):
                    if "agent_view.ppm" in self.path or "camera" in self.path:
                        candidates = [
                            "urdf_robot_view.ppm",
                            "ik_robot_reach_view.ppm",
                            "agent_view.ppm",
                            "samples/python/urdf_robot_view.ppm",
                            "samples/python/ik_robot_reach_view.ppm",
                            "samples/python/agent_view.ppm"
                        ]
                        for cand in candidates:
                            if os.path.exists(os.path.join(vis_dir, cand)):
                                self.path = "/" + cand
                                break
                    return super().do_GET()

            handler = SimLabHttpHandler

            print("=========================================================================")
            print(f" Corium SimLab Web Visualizer Dashboard Server running at:")
            print(f"   --> http://localhost:{port}/tools/visualizer/index.html")
            print(f" Press Ctrl+C to stop server.")
            print("=========================================================================\n")

            try:
                webbrowser.open(f"http://localhost:{port}/tools/visualizer/index.html")
            except Exception:
                pass

            try:
                with socketserver.TCPServer(("", port), handler) as httpd:
                    httpd.serve_forever()
            except KeyboardInterrupt:
                print("\n[Corium SimLab CLI] Visualizer server stopped.")
        else:
            print(f"[Error] Visualizer directory not found: {vis_dir}")

    else:
        parser.print_help()

if __name__ == "__main__":
    main()
