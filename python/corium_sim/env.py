try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

try:
    import corium_sim_py
except ImportError:
    import sys
    import os
    # Add build directory to path to locate compiled C++ module
    build_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "build"))
    if build_dir not in sys.path:
        sys.path.append(build_dir)
    import corium_sim_py

class CoriumEnv:
    """
    Standard Gymnasium-compliant Reinforcement Learning Environment for Corium SimLab.
    """
    def __init__(self, render_mode: str = "human"):
        self.render_mode = render_mode
        self._app = corium_sim_py.SimLabApp()
        self._step_count = 0
        self._max_steps = 500

    def reset(self, seed: int = None, options: dict = None):
        self._step_count = 0
        self._app.reset()

        obs = self._get_obs()
        info = {"episode_step": 0}
        return obs, info

    def step(self, action):
        """
        Execute environment step with action vector:
        action = [move_forward, turn_yaw, move_up]
        """
        self._step_count += 1

        move_forward = float(action[0]) if len(action) > 0 else 0.0
        turn_yaw = float(action[1]) if len(action) > 1 else 0.0
        move_up = float(action[2]) if len(action) > 2 else 0.0

        # Extract current observation state
        raw_obs = self._app.get_observation()
        dist = raw_obs.get("distance", 5.0)
        reward = raw_obs.get("reward", -dist)
        terminated = raw_obs.get("terminated", False)
        truncated = (self._step_count >= self._max_steps)

        obs = self._get_obs()
        info = {"distance_to_target": dist}

        return obs, reward, terminated, truncated, info

    def _get_obs(self):
        raw_obs = self._app.get_observation()
        agent_pos = raw_obs.get("agent_pos", [0.0, 0.0, 0.0])
        agent_vel = raw_obs.get("agent_vel", [0.0, 0.0, 0.0])
        target_pos = raw_obs.get("target_pos", [4.0, 0.75, -2.0])

        vector_obs = agent_pos + agent_vel + target_pos
        rgb_bytes = self._app.get_sensor_frame()

        if HAS_NUMPY:
            vector_obs = np.array(vector_obs, dtype=np.float32)
            rgb_frame = np.frombuffer(rgb_bytes, dtype=np.uint8).reshape((128, 128, 4))
        else:
            rgb_frame = rgb_bytes

        return {
            "vector": vector_obs,
            "rgb": rgb_frame
        }
