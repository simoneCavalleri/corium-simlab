# =============================================================================
# Corium SimLab — Python User-Friendly Environment & Agent Builder
# =============================================================================

from typing import Callable, Optional, Any, List
from .env import CoriumEnv
import corium_sim_py

class AgentSpec:
    """
    Decoupled Physical Agent Specification blueprint containing 3D Model, Sensors,
    Perception Fusion Chain, Actuators, and Policy.
    """
    def __init__(self, name: str = "agent_robot"):
        self.name = name
        self.initial_position = [0.0, 0.5, 0.0]
        self.sensors: List[str] = []

    def with_model(self, name: str, position: List[float] = [0.0, 0.5, 0.0]) -> "AgentSpec":
        """Set agent physical body model name and default position."""
        self.name = name
        self.initial_position = position
        return self

class CoriumEnvBuilder:
    """
    Fluent Python Environment Builder for creating user-defined 3D physical agent environments,
    scene geometry, sensor suites, and RL Gym environments.
    """
    def __init__(self):
        self._scene_fn: Optional[Callable[[corium_sim_py.SceneBuilder], Any]] = None
        self._max_episode_steps: int = 500
        self._sensor_width: int = 128
        self._sensor_height: int = 128
        self._agents: List[AgentSpec] = []

    def with_environment(self, scene_builder_fn: Callable[[corium_sim_py.SceneBuilder], Any]) -> "CoriumEnvBuilder":
        """Attach user-defined 3D scene construction callback."""
        self._scene_fn = scene_builder_fn
        return self

    def with_scene(self, scene_builder_fn: Callable[[corium_sim_py.SceneBuilder], Any]) -> "CoriumEnvBuilder":
        """Alias for with_environment."""
        return self.with_environment(scene_builder_fn)


    def spawn_agent(self, agent_spec: AgentSpec) -> "CoriumEnvBuilder":
        """Spawn an agent instance defined by an AgentSpec blueprint into the environment."""
        self._agents.append(agent_spec)
        return self

    def with_max_episode_steps(self, max_steps: int) -> "CoriumEnvBuilder":
        """Set maximum episode steps before termination."""
        self._max_episode_steps = max_steps
        return self

    def with_sensor_resolution(self, width: int, height: int) -> "CoriumEnvBuilder":
        """Set offscreen visual sensor camera resolution."""
        self._sensor_width = width
        self._sensor_height = height
        return self

    def build(self) -> CoriumEnv:
        """Instantiate completed Gymnasium RL environment."""
        env = CoriumEnv(scene_builder_fn=self._scene_fn)
        env._app.config.max_episode_steps = self._max_episode_steps
        env._app.config.sensor_width = self._sensor_width
        env._app.config.sensor_height = self._sensor_height
        return env
