#!/usr/bin/env python3
# =============================================================================
# Corium SimLab Sample #01 — Python Gymnasium RL Environment Loop
# =============================================================================

import sys
import os
import random

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "python")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "build")))

from corium_sim import CoriumEnv

def main():
    print("=========================================================")
    print(" Corium SimLab Sample #01: Python Gymnasium Environment")
    print("=========================================================\n")

    env = CoriumEnv()
    obs, info = env.reset()

    print("[Python Sample] Environment Reset Successfully!")
    print("  - Vector Observation Payload:", obs["vector"])

    print("\n[Python Sample] Stepping Environment 5 Steps with Random Policy...")
    for step in range(5):
        action = [random.uniform(-1.0, 1.0) for _ in range(3)]
        obs, reward, terminated, truncated, info = env.step(action)
        print(f"  - Step #{step+1}: Reward = {reward:.4f} | Dist = {info['distance_to_target']:.4f} | Terminated = {terminated}")

    print("\nCorium SimLab Python Gymnasium sample completed successfully!")

if __name__ == "__main__":
    main()
