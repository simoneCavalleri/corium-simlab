#!/usr/bin/env python3
# =============================================================================
# Corium SimLab Sample #05 — 3D LiDAR & Proximity Sensor Point Cloud Scanning
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
    print(" Corium SimLab Sample #05: 3D LiDAR & Ultrasonic Sensor Raycasting")
    print("=========================================================================\n")

    app = corium_sim_py.SimLabApp()
    app.setup_default_scene()
    app.reset()

    # 1. Single Raycast Test
    print("[Python Sample] Casting single 3D ray towards obstacle at (-3.0, 1.0, 2.0)...")
    hit = app.cast_ray(orig_x=0.0, orig_y=1.0, orig_z=0.0, dir_x=-3.0, dir_y=0.0, dir_z=2.0, max_distance=10.0)

    if hit["hit"]:
        print(f"  - Ray Hit Entity: '{hit['entity']}'")
        print(f"  - Intersection Distance: {hit['distance']:.4f} m")
        print(f"  - Hit Coordinates 3D:   {hit['point']}")
        print(f"  - Surface Normal Vector: {hit['normal']}")
    else:
        print("  - Raycast missed (no obstacle encountered).")

    # 2. 360-degree LiDAR Scan
    print("\n[Python Sample] Executing 360-degree 3D LiDAR Scan (36 Rays)...")
    scan_hits = app.cast_lidar_scan(orig_x=0.0, orig_y=1.0, orig_z=0.0, num_rays=36, fov_deg=360.0, max_distance=15.0)

    print(f"  - Total LiDAR Rays Cast: {len(scan_hits)}")
    detected_entities = set(h["entity"] for h in scan_hits if h["hit"])
    print(f"  - Detected 3D Scene Entities: {list(detected_entities)}")

    print("\nSample Ray Hits Summary:")
    for idx, ray in enumerate(scan_hits[:6]):
        if ray["hit"]:
            print(f"  Ray #{idx+1:02d}: Hit '{ray['entity']}' at {ray['distance']:.2f} m -> Point: [{ray['point'][0]:.2f}, {ray['point'][1]:.2f}, {ray['point'][2]:.2f}]")
        else:
            print(f"  Ray #{idx+1:02d}: Open Space (Distance > {ray['distance']:.1f} m)")

    print("\nCorium SimLab Sample #05 completed successfully!")

if __name__ == "__main__":
    main()
