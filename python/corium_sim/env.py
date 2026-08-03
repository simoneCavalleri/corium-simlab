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
    Standard Gymnasium-compliant Physical Agent Incubator Environment for Corium SimLab.

    The user explicitly defines the environment scene via a `scene_builder_fn` callback.
    """
    def __init__(self, scene_builder_fn=None, render_mode: str = "human", max_steps: int = 500, sim_dt: float = 0.016667,
                 action_shape: tuple = (3,), observation_shape: tuple = (9,)):
        self.render_mode = render_mode
        self._app = corium_sim_py.SimLabApp()

        if scene_builder_fn is not None:
            builder = self._app.create_scene_builder()
            user_scene = scene_builder_fn(builder)
            if hasattr(user_scene, "build"):
                user_scene = user_scene.build()
            self._app.set_scene(user_scene)
        else:
            # Explicit default user environment setup via SceneBuilder
            builder = self._app.create_scene_builder()
            scene = (builder
                     .add_ground_grid(50.0, 50.0, 50)
                     .add_cube("agent_robot", corium_sim_py.Vec3(0.0, 0.5, 0.0), corium_sim_py.Vec3(1.0, 1.0, 1.0))
                     .add_cube("target_goal", corium_sim_py.Vec3(4.0, 0.75, -2.0), corium_sim_py.Vec3(1.2, 1.2, 1.2))
                     .build())
            self._app.set_scene(scene)

        self._step_count = 0
        self._max_steps = max_steps
        self._sim_dt = sim_dt
        self.action_space_shape = action_shape
        self.observation_space_shape = observation_shape

    @property
    def action_space(self):
        return {"shape": self.action_space_shape, "dtype": "float32"}

    @property
    def observation_space(self):
        return {"shape": self.observation_space_shape, "dtype": "float32"}

    def reset(self, seed: int = None, options: dict = None):
        self._step_count = 0
        self._app.reset()

        obs = self._get_obs()
        info = {"episode_step": 0}
        return obs, info

    def step(self, action):
        """
        Execute environment step with user action vector.
        """
        self._step_count += 1

        move_forward = float(action[0]) if len(action) > 0 else 0.0
        turn_yaw = float(action[1]) if len(action) > 1 else 0.0
        move_up = float(action[2]) if len(action) > 2 else 0.0

        # Apply agent action forces and advance physics simulation step
        self._app.apply_action(move_forward, turn_yaw, move_up)
        self._app.sim_step(self._sim_dt)

        # Extract current observation state after physics step
        raw_obs = self._app.get_observation()
        dist = raw_obs.get("distance", 5.0)
        reward = raw_obs.get("reward", -dist)
        terminated = raw_obs.get("terminated", False)
        truncated = (self._step_count >= self._max_steps)

        obs = self._get_obs()
        info = {
            "distance_to_target": dist,
            "episode_step": self._step_count,
        }

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

    def get_pil_image(self):
        """Extract current onboard camera frame as a PIL Image object."""
        rgb_bytes = self._app.get_sensor_frame()
        try:
            from PIL import Image
            return Image.frombytes("RGBA", (128, 128), rgb_bytes)
        except ImportError:
            return rgb_bytes

    def save_sensor_frame_ppm(self, filename: str) -> bool:
        """Save current onboard camera frame directly to a PPM image file."""
        return self._app.save_sensor_frame_ppm(filename)

    def close(self):
        """Clean up environment resources."""
        pass
