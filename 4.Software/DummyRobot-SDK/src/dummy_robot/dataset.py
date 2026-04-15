from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional
import json
import time

from .types import RobotAction, RobotObservation

try:
    import pyarrow as pa
    import pyarrow.parquet as pq
except ImportError:  # pragma: no cover
    pa = None
    pq = None


@dataclass
class LeRobotFrameRecord:
    episode_index: int
    frame_index: int
    timestamp: float
    task: str
    action: Dict[str, Any]
    observation: Dict[str, Any]
    done: bool = False
    extra: Dict[str, Any] = field(default_factory=dict)

    def as_row(self) -> Dict[str, Any]:
        joint_positions = self.observation.get("joint_positions") or [None] * 6
        tool_pose = self.observation.get("tool_pose") or [None] * 6
        action_joints = self.action.get("joint_positions") or [None] * 6
        return {
            "episode_index": self.episode_index,
            "frame_index": self.frame_index,
            "timestamp": self.timestamp,
            "task": self.task,
            "done": self.done,
            "action.type": "joint" if self.action.get("joint_positions") is not None else "pose",
            "action.joint_positions": json.dumps(action_joints),
            "action.speed": self.action.get("speed"),
            "observation.joint_positions": json.dumps(joint_positions),
            "observation.tool_pose": json.dumps(tool_pose),
            "extra": json.dumps(self.extra),
        }


class LeRobotDatasetWriter:
    """
    Optional low-dimensional LeRobot export.

    This writer intentionally excludes structured-light image and point-cloud data.
    It only records robot observations and actions.
    """

    def __init__(
        self,
        root_dir: str,
        repo_id: str = "local/dummy-robot",
        fps: float = 5.0,
        robot_type: str = "dummy_robot",
    ) -> None:
        self.root_dir = Path(root_dir)
        self.repo_id = repo_id
        self.fps = fps
        self.robot_type = robot_type
        self._episode_rows: List[Dict[str, Any]] = []
        self._episode_task: Optional[str] = None
        self._episode_index: Optional[int] = None
        self._frame_index = 0
        self._ensure_layout()
        self._write_info_file()

    def start_episode(self, episode_index: int, task: str, metadata: Optional[Dict[str, Any]] = None) -> None:
        self._episode_index = episode_index
        self._episode_task = task
        self._frame_index = 0
        self._episode_rows = []
        self._episode_metadata = metadata or {}

    def add_robot_frame(
        self,
        observation: RobotObservation,
        action: RobotAction,
        done: bool = False,
        extra: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        if self._episode_index is None or self._episode_task is None:
            raise RuntimeError("Call start_episode() before adding frames")

        record = LeRobotFrameRecord(
            episode_index=self._episode_index,
            frame_index=self._frame_index,
            timestamp=observation.timestamp,
            task=self._episode_task,
            action=action.as_dict(),
            observation=observation.as_dict(),
            done=done,
            extra=extra or {},
        )
        row = record.as_row()
        self._episode_rows.append(row)
        self._frame_index += 1
        return row

    def finalize_episode(self) -> Path:
        if self._episode_index is None or self._episode_task is None:
            raise RuntimeError("No active episode to finalize")
        if pa is None or pq is None:
            raise RuntimeError("pyarrow is required to export LeRobot parquet episodes")

        data_path = self._episode_parquet_path(self._episode_index)
        table = pa.Table.from_pylist(self._episode_rows)
        pq.write_table(table, data_path)
        self._append_episode_metadata(self._episode_index, self._episode_task, len(self._episode_rows))
        return data_path

    def _ensure_layout(self) -> None:
        for relative in ["data/chunk-000", "meta"]:
            (self.root_dir / relative).mkdir(parents=True, exist_ok=True)

    def _write_info_file(self) -> None:
        info = {
            "codebase_version": "dummy_robot_sdk",
            "robot_type": self.robot_type,
            "repo_id": self.repo_id,
            "fps": self.fps,
            "format": "lerobot_lowdim_v1",
            "created_at": time.time(),
        }
        (self.root_dir / "meta" / "info.json").write_text(json.dumps(info, indent=2), encoding="utf-8")

    def _append_episode_metadata(self, episode_index: int, task: str, length: int) -> None:
        payload = {
            "episode_index": episode_index,
            "task": task,
            "length": length,
            "data_path": str(self._episode_parquet_path(episode_index).relative_to(self.root_dir)).replace("\\", "/"),
            "metadata": self._episode_metadata,
        }
        with (self.root_dir / "meta" / "episodes.jsonl").open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload, ensure_ascii=False) + "\n")

    def _episode_parquet_path(self, episode_index: int) -> Path:
        return self.root_dir / "data" / "chunk-000" / f"episode_{episode_index:06d}.parquet"
