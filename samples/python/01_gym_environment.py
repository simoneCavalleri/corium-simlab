#!/usr/bin/env python3
# =============================================================================
# Corium SimLab Sample #01 — Python Gymnasium RL Environment Loop
# Fluent CoriumEnvBuilder API
# =============================================================================

import sys
import os
import random

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "python")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "build")))

import corium_sim_py
from corium_sim import CoriumEnvBuilder

def build_user_scene(builder):
    """User explicitly defines the 3D environment scene."""
    return (builder
            .add_ground_grid(50.0, 50.0, 50)
            .add_cube("agent_robot", corium_sim_py.Vec3(0.0, 0.5, 0.0), corium_sim_py.Vec3(1.0, 1.0, 1.0))
            .add_cube("target_goal", corium_sim_py.Vec3(4.0, 0.75, -2.0), corium_sim_py.Vec3(1.2, 1.2, 1.2))
            .add_sphere("obstacle_ball", corium_sim_py.Vec3(-3.0, 1.0, 2.0), 1.0))

def main():
    print("=========================================================")
    print(" Corium SimLab Sample #01: Python Gymnasium Environment")
    print(" Features: Fluent CoriumEnvBuilder API")
    print("=========================================================\n")

    # Instantiate Gymnasium environment using fluent CoriumEnvBuilder
    env = (CoriumEnvBuilder()
           .with_scene(build_user_scene)
           .with_max_episode_steps(500)
           .with_sensor_resolution(128, 128)
           .build())

    obs, info = env.reset()

    print("[Python Sample] User-Defined Environment Reset Successfully!")
    print("  - Vector Observation Payload:", obs["vector"])

    print("\n[Python Sample] Stepping Environment 5 Steps with User Random Policy...")
    for step in range(5):
        action = [random.uniform(-1.0, 1.0) for _ in range(3)]
        obs, reward, terminated, truncated, info = env.step(action)
        print(f"  - Step #{step+1}: Reward = {reward:.4f} | Dist = {info['distance_to_target']:.4f} | Terminated = {terminated}")

    print("\nCorium SimLab Python Gymnasium sample completed successfully!")

if __name__ == "__main__":
    main()
