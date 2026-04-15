from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Optional
import json
import time

from dummy_robot import DummyRobot


JOINT_COUNT = 6


@dataclass
class UniformScanConfig:
    port: str
    baudrate: int = 115200
    timeout: float = 1.0
    initial_joints: List[float] = field(default_factory=lambda: [0.0, -60.0, 150.0, 0.0, 0.0, 0.0])
    joint_index: int = 3
    step_angle_deg: float = -0.1
    total_steps: int = 200
    speed: float = 50.0
    pause_time_s: float = 0.5
    initial_settle_s: float = 3.0
    return_settle_s: float = 3.0
    sequential: bool = False
    enable_before_scan: bool = False
    disable_after_scan: bool = False
    return_to_initial: bool = True
    command_suffix: str = ";\n"
    log_path: Optional[str] = None

    def __post_init__(self) -> None:
        if len(self.initial_joints) != JOINT_COUNT:
            raise ValueError(f"initial_joints must contain {JOINT_COUNT} values")
        if not 1 <= self.joint_index <= JOINT_COUNT:
            raise ValueError(f"joint_index must be between 1 and {JOINT_COUNT}")
        if self.total_steps <= 0:
            raise ValueError("total_steps must be greater than 0")
        if self.pause_time_s < 0:
            raise ValueError("pause_time_s must be >= 0")
        if self.initial_settle_s < 0 or self.return_settle_s < 0:
            raise ValueError("settle times must be >= 0")

    @classmethod
    def from_json(cls, path: str | Path) -> "UniformScanConfig":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(**payload)


@dataclass
class UniformScanResult:
    started_at: float
    ended_at: Optional[float] = None
    completed_steps: int = 0
    interrupted: bool = False
    final_joint_value: Optional[float] = None
    log_path: Optional[str] = None

    def as_dict(self) -> Dict[str, object]:
        return asdict(self)


class SerialMotionController:
    def __init__(self, config: UniformScanConfig, dry_run: bool = False) -> None:
        self.config = config
        self.dry_run = dry_run
        self._robot: Optional[DummyRobot] = None
        self._log_records: List[Dict[str, object]] = []

    def __enter__(self) -> "SerialMotionController":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    def connect(self) -> None:
        if self.dry_run:
            return
        if self._robot is not None and self._robot.is_connected:
            return
        self._robot = DummyRobot(
            port=self.config.port,
            baudrate=self.config.baudrate,
            timeout=self.config.timeout,
        )
        self._robot.connect()

    def close(self) -> None:
        if self._robot is not None and self._robot.is_connected:
            self._robot.disconnect()
        self._robot = None

    def run_uniform_joint_scan(self) -> UniformScanResult:
        result = UniformScanResult(started_at=time.time(), log_path=self.config.log_path)
        current_joints = list(self.config.initial_joints)
        target_joint_idx = self.config.joint_index - 1

        try:
            self.connect()
            if self.config.enable_before_scan:
                self.send_lifecycle("enable")

            self.send_motion(current_joints)
            self._log_step(step_index=0, joints=current_joints, phase="initial")
            time.sleep(self.config.initial_settle_s)

            for step in range(1, self.config.total_steps + 1):
                current_joints[target_joint_idx] += self.config.step_angle_deg
                self.send_single_joint_target(current_joints[target_joint_idx])
                self._log_step(step_index=step, joints=current_joints, phase="scan")
                result.completed_steps = step
                result.final_joint_value = current_joints[target_joint_idx]
                if self.config.pause_time_s > 0:
                    time.sleep(self.config.pause_time_s)
        except KeyboardInterrupt:
            result.interrupted = True
        finally:
            try:
                if self.config.return_to_initial:
                    self.send_motion(self.config.initial_joints)
                    self._log_step(
                        step_index=result.completed_steps,
                        joints=self.config.initial_joints,
                        phase="return",
                    )
                    time.sleep(self.config.return_settle_s)
                if self.config.disable_after_scan:
                    self.send_lifecycle("disable")
            finally:
                self._flush_logs()
                self.close()
                result.ended_at = time.time()

        return result

    def send_motion(self, joints: List[float]) -> None:
        if self.dry_run:
            return
        if self._robot is None:
            raise RuntimeError("Robot is not connected")
        self._robot.move_joints(joints, speed=self.config.speed, sequential=self.config.sequential)

    def send_single_joint_target(self, value: float) -> None:
        if self.dry_run:
            return
        if self._robot is None:
            raise RuntimeError("Robot is not connected")
        self._robot.move_single_joint(
            joint_index=self.config.joint_index,
            target=value,
            speed=self.config.speed,
            sequential=self.config.sequential,
        )

    def send_lifecycle(self, action: str) -> None:
        if self.dry_run:
            return
        if self._robot is None:
            raise RuntimeError("Robot is not connected")
        if action == "enable":
            self._robot.enable()
        elif action == "disable":
            self._robot.disable()
        else:
            raise ValueError(f"Unsupported lifecycle action: {action}")

    def _log_step(self, step_index: int, joints: List[float], phase: str) -> None:
        self._log_records.append(
            {
                "timestamp": time.time(),
                "phase": phase,
                "step_index": step_index,
                "joints": list(joints),
                "speed": self.config.speed,
                "joint_index": self.config.joint_index,
                "joint_value": joints[self.config.joint_index - 1],
            }
        )

    def _flush_logs(self) -> None:
        if not self.config.log_path:
            return
        path = Path(self.config.log_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self._log_records, indent=2, ensure_ascii=False), encoding="utf-8")

    @staticmethod
    def _format_number(value: float) -> str:
        return format(float(value), ".6g")
