from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Union

from .client import DummyRobot
from .types import JOINT_COUNT, JOINT_NAMES, Pose6D, RobotAction


@dataclass(frozen=True)
class LeRobotFeature:
    type: str
    shape: List[int]
    names: List[str] = field(default_factory=list)

    def as_dict(self) -> Dict[str, Any]:
        payload: Dict[str, Any] = {"type": self.type, "shape": self.shape}
        if self.names:
            payload["names"] = self.names
        return payload


class DummyRobotAdapter:
    """
    Minimal adapter that exposes DummyRobot through a LeRobot-style interface.

    This adapter is intentionally low-dimensional:
    - observations: joint positions + optional 6D tool pose
    - actions: joint targets or 6D tool pose targets
    - no structured-light image or point-cloud support in this layer
    """

    robot_type = "dummy_robot"

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 1.0,
        include_tool_pose: bool = True,
        default_action_speed: Optional[float] = None,
    ) -> None:
        self.robot = DummyRobot(port=port, baudrate=baudrate, timeout=timeout)
        self.include_tool_pose = include_tool_pose
        self.default_action_speed = default_action_speed

    @property
    def name(self) -> str:
        return self.robot_type

    @property
    def is_connected(self) -> bool:
        return self.robot.is_connected

    @property
    def observation_features(self) -> Dict[str, Dict[str, Any]]:
        features = {
            "observation.joint_positions": LeRobotFeature(
                type="float32",
                shape=[JOINT_COUNT],
                names=list(JOINT_NAMES),
            ).as_dict(),
        }
        if self.include_tool_pose:
            features["observation.tool_pose"] = LeRobotFeature(
                type="float32",
                shape=[6],
                names=["x", "y", "z", "a", "b", "c"],
            ).as_dict()
        return features

    @property
    def action_features(self) -> Dict[str, Dict[str, Any]]:
        features = {
            "action.joint_positions": LeRobotFeature(
                type="float32",
                shape=[JOINT_COUNT],
                names=list(JOINT_NAMES),
            ).as_dict(),
        }
        if self.include_tool_pose:
            features["action.tool_pose"] = LeRobotFeature(
                type="float32",
                shape=[6],
                names=["x", "y", "z", "a", "b", "c"],
            ).as_dict()
        features["action.speed"] = LeRobotFeature(type="float32", shape=[1], names=["speed"]).as_dict()
        return features

    def connect(self) -> None:
        self.robot.connect()

    def disconnect(self) -> None:
        self.robot.disconnect()

    def reset(self) -> Dict[str, Any]:
        return self.capture_observation()

    def capture_observation(self) -> Dict[str, Any]:
        observation = self.robot.get_observation(include_pose=self.include_tool_pose, include_sensors=False)
        payload = {
            "observation.joint_positions": observation.joints.as_list() if observation.joints is not None else None,
        }
        if self.include_tool_pose and observation.tool_pose is not None:
            payload["observation.tool_pose"] = observation.tool_pose.as_list()
        return payload

    def get_observation(self) -> Dict[str, Any]:
        return self.capture_observation()

    def send_action(self, action: Union[RobotAction, Dict[str, Any], Sequence[float]]) -> str:
        normalized = self._normalize_action(action)
        return self.robot.execute_action(normalized)

    def apply_action(self, action: Union[RobotAction, Dict[str, Any], Sequence[float]]) -> str:
        return self.send_action(action)

    def teleop_step(self, action: Union[RobotAction, Dict[str, Any], Sequence[float]]) -> Dict[str, Any]:
        self.send_action(action)
        return self.capture_observation()

    def step(self, action: Union[RobotAction, Dict[str, Any], Sequence[float]]) -> Dict[str, Any]:
        return self.teleop_step(action)

    def _normalize_action(self, action: Union[RobotAction, Dict[str, Any], Sequence[float]]) -> RobotAction:
        if isinstance(action, RobotAction):
            if action.speed is None and self.default_action_speed is not None:
                return RobotAction(
                    joint_positions=action.joint_positions,
                    tool_pose=action.tool_pose,
                    speed=self.default_action_speed,
                )
            return action

        if isinstance(action, dict):
            joint_positions = action.get("joint_positions")
            tool_pose = action.get("tool_pose")
            speed = action.get("speed", self.default_action_speed)

            if tool_pose is not None and not isinstance(tool_pose, Pose6D):
                tool_pose = Pose6D(*tool_pose)

            if joint_positions is None:
                joint_positions = action.get("action.joint_positions")
            if tool_pose is None and action.get("action.tool_pose") is not None:
                tool_pose = Pose6D(*action["action.tool_pose"])
            if speed is None and action.get("action.speed") is not None:
                speed = action["action.speed"]

            return RobotAction(joint_positions=joint_positions, tool_pose=tool_pose, speed=speed)

        return RobotAction(joint_positions=list(action), speed=self.default_action_speed)
