#!/usr/bin/env python3
import sys
import os
import random

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "build")))

from corium_sim import CoriumEnv

def main():
    print("=========================================================")
    print(" Corium SimLab — Python Gymnasium Environment Test")
    print("=========================================================\n")

    env = CoriumEnv()
    obs, info = env.reset()

    print("[Python API] Environment Reset Successfully!")
    print("  - Vector Observation Payload:", obs["vector"])

    print("\n[Python API] Stepping Environment 5 Episodes with Random Policy...")
    for step in range(5):
        action = [random.uniform(-1.0, 1.0) for _ in range(3)]
        obs, reward, terminated, truncated, info = env.step(action)
        print(f"  - Step #{step+1}: Reward = {reward:.4f} | Dist = {info['distance_to_target']:.4f} | Terminated = {terminated}")

    print("\nCorium SimLab Python Gymnasium Environment test completed successfully!")

if __name__ == "__main__":
    main()
