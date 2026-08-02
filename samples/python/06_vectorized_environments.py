#!/usr/bin/env python3
# =============================================================================
# Corium SimLab Sample #06 — Vectorized Parallel Environments for RL Training
# =============================================================================

import sys
import os
import random

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "python")))

try:
    from corium_sim import VectorEnv
except ImportError:
    print("[Error] Could not import VectorEnv from corium_sim package!")
    sys.exit(1)

def main():
    print("=========================================================================")
    print(" Corium SimLab Sample #06: Vectorized Parallel Environments (VectorEnv)")
    print("=========================================================================\n")

    num_parallel_envs = 8
    print(f"[Python Sample] Initializing VectorEnv with {num_parallel_envs} parallel 3D simulation instances...")

    vec_env = VectorEnv(num_envs=num_parallel_envs)
    batched_obs, info_list = vec_env.reset()

    print(f"  - Number of Active Parallel Envs:  {len(batched_obs)}")
    print(f"  - Environment #1 Observation:     {batched_obs[0]}")

    # Execute 50 parallel batch steps
    print("\n[Python Sample] Running 50 Batch Simulation Steps across all 8 parallel environments...")
    for step in range(1, 51):
        # Generate random batch actions for 8 environments
        batch_actions = [
            [random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)]
            for _ in range(num_parallel_envs)
        ]
        obs, rewards, terminated, truncated, infos = vec_env.step(batch_actions)

        if step % 10 == 0:
            avg_reward = sum(rewards) / len(rewards)
            avg_dist = sum(info.get("distance_to_target", 0.0) for info in infos) / len(infos)
            print(f"  Step {step:02d}/50 | Avg Batch Reward: {avg_reward:.2f} | Avg Distance to Target: {avg_dist:.2f} m")

    vec_env.close()
    print("\nCorium SimLab Sample #06 completed successfully!")

if __name__ == "__main__":
    main()
