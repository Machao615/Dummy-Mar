from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, Callable, Optional, Union
import time

import numpy as np

from .exceptions import DummyRobotCommandError, DummyRobotConnectionError
from .sensors import SensorInterface
from .types import CameraFrame, CameraIntrinsics, SensorFrame, SyncedCameraFrame

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


class CameraInterface(SensorInterface, ABC):
    """Interface for camera devices that provide image frames."""

    name: str

    @abstractmethod
    def read_frame(self) -> Optional[CameraFrame]:
        """Return the latest image frame, or None if unavailable."""

    def read(self) -> Optional[SensorFrame]:
        frame = self.read_frame()
        if frame is None:
            return None
        return SensorFrame(name=self.name, timestamp=frame.timestamp, data=frame.as_dict())


class CallbackCamera(CameraInterface):
    """Wrap a callback so an experimental camera source can be integrated quickly."""

    def __init__(self, name: str, callback: Callable[[], Optional[Any]]) -> None:
        self.name = name
        self._callback = callback

    def read_frame(self) -> Optional[CameraFrame]:
        payload = self._callback()
        if payload is None:
            return None
        if isinstance(payload, CameraFrame):
            return payload
        if isinstance(payload, dict):
            if "image" not in payload:
                raise TypeError("Camera callback payload must include an 'image' entry")
            return CameraFrame(
                timestamp=float(payload.get("timestamp", 0.0)) or time.time(),
                image=payload["image"],
                frame_id=str(payload.get("frame_id", self.name)),
                metadata=dict(payload.get("metadata", {})),
            )
        raise TypeError(f"Unsupported camera payload type: {type(payload)!r}")


class OpenCVCamera(CameraInterface):
    """
    Camera source backed by OpenCV VideoCapture.

    `source` may be a camera device index such as `0`, or a video file path.
    """

    def __init__(
        self,
        name: str,
        source: Union[int, str] = 0,
        intrinsics: Optional[CameraIntrinsics] = None,
        undistort: bool = True,
        convert_bgr_to_rgb: bool = True,
        width: Optional[int] = None,
        height: Optional[int] = None,
        fps: Optional[float] = None,
        warmup_frames: int = 3,
    ) -> None:
        self.name = name
        self.source = source
        self.intrinsics = intrinsics
        self.undistort = undistort
        self.convert_bgr_to_rgb = convert_bgr_to_rgb
        self.width = width
        self.height = height
        self.fps = fps
        self.warmup_frames = max(0, int(warmup_frames))
        self._capture = None

    @property
    def is_open(self) -> bool:
        return self._capture is not None and bool(self._capture.isOpened())

    def connect(self) -> "OpenCVCamera":
        if cv2 is None:
            raise RuntimeError("opencv-python is required to use OpenCVCamera")
        if self.is_open:
            return self
        self._capture = cv2.VideoCapture(self.source)
        if not self._capture.isOpened():
            self._capture.release()
            self._capture = None
            raise DummyRobotConnectionError(f"Failed to open camera source: {self.source!r}")
        if self.width is not None:
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        if self.height is not None:
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        if self.fps is not None:
            self._capture.set(cv2.CAP_PROP_FPS, float(self.fps))
        for _ in range(self.warmup_frames):
            ok, _ = self._capture.read()
            if not ok:
                break
        return self

    def disconnect(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    def __enter__(self) -> "OpenCVCamera":
        return self.connect()

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.disconnect()

    def read_frame(self) -> Optional[CameraFrame]:
        if cv2 is None:
            raise RuntimeError("opencv-python is required to use OpenCVCamera")
        if not self.is_open:
            self.connect()
        assert self._capture is not None
        ok, frame = self._capture.read()
        if not ok or frame is None:
            return None

        processed = frame
        if self.undistort and self.intrinsics is not None and self.intrinsics.dist_coeffs:
            processed = cv2.undistort(
                processed,
                self.intrinsics.camera_matrix(),
                np.asarray(self.intrinsics.dist_coeffs, dtype=np.float64),
            )
        if self.convert_bgr_to_rgb:
            processed = cv2.cvtColor(processed, cv2.COLOR_BGR2RGB)

        metadata = {
            "source": str(self.source),
            "width": int(processed.shape[1]),
            "height": int(processed.shape[0]),
            "undistort": self.undistort,
            "convert_bgr_to_rgb": self.convert_bgr_to_rgb,
        }
        return CameraFrame(timestamp=time.time(), image=processed, frame_id=self.name, metadata=metadata)

    @classmethod
    def from_json(
        cls,
        name: str,
        source: Union[int, str],
        intrinsics_path: Union[str, Path],
        undistort: bool = True,
        convert_bgr_to_rgb: bool = True,
        width: Optional[int] = None,
        height: Optional[int] = None,
        fps: Optional[float] = None,
        warmup_frames: int = 3,
    ) -> "OpenCVCamera":
        import json

        path = Path(intrinsics_path)
        payload = json.loads(path.read_text(encoding="utf-8"))
        intrinsics = CameraIntrinsics(
            fx=float(payload["fx"]),
            fy=float(payload["fy"]),
            cx=float(payload["cx"]),
            cy=float(payload["cy"]),
            dist_coeffs=[float(value) for value in payload.get("dist_coeffs", [])],
            image_width=payload.get("image_width"),
            image_height=payload.get("image_height"),
        )
        return cls(
            name=name,
            source=source,
            intrinsics=intrinsics,
            undistort=undistort,
            convert_bgr_to_rgb=convert_bgr_to_rgb,
            width=width,
            height=height,
            fps=fps,
            warmup_frames=warmup_frames,
        )


class CameraPoseSynchronizer:
    """Capture robot pose around camera acquisition to build synchronized image packets."""

    def sync_frame(self, robot: Any, camera: CameraInterface) -> SyncedCameraFrame:
        pose_before = robot.get_state_snapshot(include_pose=True)
        frame = camera.read_frame()
        if frame is None:
            raise DummyRobotCommandError(f"Camera '{camera.name}' returned no frame")
        pose_after = robot.get_state_snapshot(include_pose=True)
        pose_midpoint = self._midpoint_state(pose_before, pose_after, frame.timestamp)
        return SyncedCameraFrame(
            frame=frame,
            pose_before=pose_before,
            pose_after=pose_after,
            pose_midpoint=pose_midpoint,
        )

    @staticmethod
    def _midpoint_state(pose_before: Any, pose_after: Any, timestamp: float) -> Any:
        joint_values = [
            (before_value + after_value) / 2.0
            for before_value, after_value in zip(pose_before.joints.as_list(), pose_after.joints.as_list())
        ]
        midpoint_pose = None
        if pose_before.tool_pose is not None and pose_after.tool_pose is not None:
            midpoint_pose = type(pose_before.tool_pose)(
                *[
                    (before_value + after_value) / 2.0
                    for before_value, after_value in zip(pose_before.tool_pose.as_list(), pose_after.tool_pose.as_list())
                ]
            )
        return type(pose_before)(
            timestamp=timestamp,
            joints=type(pose_before.joints)(joint_values),
            tool_pose=midpoint_pose,
        )
