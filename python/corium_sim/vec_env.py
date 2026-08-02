from typing import List, Dict, Any, Tuple
from .env import CoriumEnv

class VectorEnv:
    """
    High-Performance Vectorized Parallel Environment Manager for Corium SimLab.
    Manages N parallel simulation instances for accelerated RL batch training (PPO / SAC).
    """

    def __init__(self, num_envs: int = 4, render_mode: str = None, sim_dt: float = 0.016667):
        self.num_envs = num_envs
        self.envs: List[CoriumEnv] = [
            CoriumEnv(render_mode=render_mode, sim_dt=sim_dt)
            for _ in range(num_envs)
        ]
        self.single_action_space = self.envs[0].action_space
        self.single_observation_space = self.envs[0].observation_space

    def reset(self, seeds: List[int] = None) -> Tuple[List[List[float]], List[Dict[str, Any]]]:
        """
        Reset all N parallel environments and return batched initial observations.
        """
        obs_list = []
        info_list = []

        for i, env in enumerate(self.envs):
            seed = seeds[i] if seeds and i < len(seeds) else None
            obs, info = env.reset(seed=seed)
            obs_list.append(obs)
            info_list.append(info)

        return obs_list, info_list

    def step(self, actions: List[List[float]]) -> Tuple[List[List[float]], List[float], List[bool], List[bool], List[Dict[str, Any]]]:
        """
        Execute actions batch [N, action_dim] across all N parallel environments.
        """
        obs_list = []
        reward_list = []
        term_list = []
        trunc_list = []
        info_list = []

        for i, env in enumerate(self.envs):
            action = actions[i] if i < len(actions) else [0.0, 0.0, 0.0]
            obs, reward, terminated, truncated, info = env.step(action)

            # Auto-reset terminated/truncated envs for continuous RL sampling
            if terminated or truncated:
                info["terminal_observation"] = obs
                obs, reset_info = env.reset()
                info.update(reset_info)

            obs_list.append(obs)
            reward_list.append(reward)
            term_list.append(terminated)
            trunc_list.append(truncated)
            info_list.append(info)

        return obs_list, reward_list, term_list, trunc_list, info_list

    def close(self):
        """Close all parallel environment instances."""
        for env in self.envs:
            env.close()
