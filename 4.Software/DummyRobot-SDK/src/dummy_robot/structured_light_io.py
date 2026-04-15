from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional
import json
import time

import numpy as np

from .types import ReconstructedPointCloudFrame, RobotAction, SyncedCameraFrame


@dataclass
class StructuredLightFrameRecord:
    task_name: str
    episode_index: int
    frame_index: int
    timestamp: float
    action: Dict[str, Any]
    synced_frame: SyncedCameraFrame
    image_path: str
    pointcloud_path: str
    extra: Dict[str, Any]

    def as_dict(self) -> Dict[str, Any]:
        return {
            "task_name": self.task_name,
            "episode_index": self.episode_index,
            "frame_index": self.frame_index,
            "timestamp": self.timestamp,
            "action": self.action,
            "image_path": self.image_path,
            "pointcloud_path": self.pointcloud_path,
            "sync_latency_s": self.synced_frame.pose_after.timestamp - self.synced_frame.pose_before.timestamp,
            "camera_frame": self.synced_frame.frame.as_dict(),
            "pose_before": self.synced_frame.pose_before.as_dict(),
            "pose_after": self.synced_frame.pose_after.as_dict(),
            "pose_midpoint": self.synced_frame.pose_midpoint.as_dict(),
            "extra": self.extra,
        }


class StructuredLightDatasetWriter:
    """
    Store image-based structured-light captures for offline debugging and reconstruction.
    """

    def __init__(self, root_dir: str, project_name: str = "dummy_structured_light_scan") -> None:
        self.root_dir = Path(root_dir)
        self.project_name = project_name
        self._episode_dir: Optional[Path] = None
        self._frames_path: Optional[Path] = None
        self._frame_index = 0
        self._task_name: Optional[str] = None
        self._episode_index: Optional[int] = None
        self.root_dir.mkdir(parents=True, exist_ok=True)
        self._write_project_info()

    def start_episode(self, episode_index: int, task: str, metadata: Optional[Dict[str, Any]] = None) -> None:
        self._task_name = task
        self._episode_index = episode_index
        self._frame_index = 0
        episode_name = f"{task}_episode_{episode_index:06d}"
        self._episode_dir = self.root_dir / episode_name
        self._episode_dir.mkdir(parents=True, exist_ok=True)
        (self._episode_dir / "images").mkdir(exist_ok=True)
        (self._episode_dir / "pointclouds").mkdir(exist_ok=True)
        self._frames_path = self._episode_dir / "frames.jsonl"
        payload = {
            "project_name": self.project_name,
            "task_name": task,
            "episode_index": episode_index,
            "created_at": time.time(),
            "metadata": metadata or {},
        }
        (self._episode_dir / "task.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

    def add_frame(
        self,
        synced_frame: SyncedCameraFrame,
        reconstructed: ReconstructedPointCloudFrame,
        action: RobotAction,
        done: bool = False,
        extra: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        if self._episode_dir is None or self._frames_path is None or self._task_name is None or self._episode_index is None:
            raise RuntimeError("Call start_episode() before adding frames")

        image_path = self._write_image_artifact(self._frame_index, synced_frame)
        pointcloud_path = self._write_pointcloud_artifact(self._frame_index, reconstructed)
        record = StructuredLightFrameRecord(
            task_name=self._task_name,
            episode_index=self._episode_index,
            frame_index=self._frame_index,
            timestamp=synced_frame.frame.timestamp,
            action=action.as_dict(),
            synced_frame=synced_frame,
            image_path=image_path,
            pointcloud_path=pointcloud_path,
            extra={**(extra or {}), "done": done, "reconstruction": reconstructed.as_dict()},
        )
        with self._frames_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record.as_dict(), ensure_ascii=False) + "\n")
        self._frame_index += 1
        return record.as_dict()

    def finalize_episode(self) -> Path:
        if self._episode_dir is None:
            raise RuntimeError("No active episode to finalize")
        summary = {
            "task_name": self._task_name,
            "episode_index": self._episode_index,
            "frame_count": self._frame_index,
            "finalized_at": time.time(),
        }
        (self._episode_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
        return self._episode_dir

    def _write_project_info(self) -> None:
        payload = {
            "project_name": self.project_name,
            "format": "dummy_structured_light_scan_v1",
            "created_at": time.time(),
        }
        (self.root_dir / "project.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

    def _write_image_artifact(self, frame_index: int, synced_frame: SyncedCameraFrame) -> str:
        assert self._episode_dir is not None
        output_path = self._episode_dir / "images" / f"frame_{frame_index:06d}.npz"
        np.savez_compressed(
            output_path,
            image=np.asarray(synced_frame.frame.image),
            timestamp=np.asarray([synced_frame.frame.timestamp], dtype=np.float64),
        )
        return str(output_path.relative_to(self.root_dir)).replace("\\", "/")

    def _write_pointcloud_artifact(self, frame_index: int, reconstructed: ReconstructedPointCloudFrame) -> str:
        assert self._episode_dir is not None
        output_path = self._episode_dir / "pointclouds" / f"frame_{frame_index:06d}.npz"
        np.savez_compressed(
            output_path,
            points_xyz=np.asarray(reconstructed.points_xyz, dtype=np.float32),
            timestamp=np.asarray([reconstructed.timestamp], dtype=np.float64),
        )
        return str(output_path.relative_to(self.root_dir)).replace("\\", "/")
