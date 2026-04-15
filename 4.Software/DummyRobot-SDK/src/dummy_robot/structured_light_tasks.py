from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional
import time

from .structured_light import extract_laser_line, reconstruct_laser_points
from .types import CameraIntrinsics, Plane3D, ReconstructedPointCloudFrame, RigidTransform, RobotAction, SyncedCameraFrame


TransformResolver = Callable[[SyncedCameraFrame], Optional[RigidTransform]]


@dataclass(frozen=True)
class StructuredLightWaypoint:
    action: RobotAction
    settle_time_s: float = 0.5
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class StructuredLightScanResult:
    task_name: str
    episode_index: int
    frames: List[ReconstructedPointCloudFrame] = field(default_factory=list)
    dataset_path: Optional[str] = None
    started_at: float = field(default_factory=time.time)
    ended_at: Optional[float] = None

    def close(self) -> None:
        self.ended_at = time.time()

    def as_dict(self) -> Dict[str, Any]:
        return {
            "task_name": self.task_name,
            "episode_index": self.episode_index,
            "dataset_path": self.dataset_path,
            "started_at": self.started_at,
            "ended_at": self.ended_at,
            "frame_count": len(self.frames),
        }


class StructuredLightScanTask:
    """Execute waypoint-based image capture and reconstruct laser-line point clouds."""

    def __init__(
        self,
        name: str,
        camera_name: str,
        waypoints: List[StructuredLightWaypoint],
        intrinsics: CameraIntrinsics,
        laser_plane_in_camera: Plane3D,
        episode_index: int = 0,
        auto_enable: bool = False,
        disable_on_finish: bool = False,
        extraction_kwargs: Optional[Dict[str, Any]] = None,
        transform_resolver: Optional[TransformResolver] = None,
    ) -> None:
        self.name = name
        self.camera_name = camera_name
        self.waypoints = waypoints
        self.intrinsics = intrinsics
        self.laser_plane_in_camera = laser_plane_in_camera
        self.episode_index = episode_index
        self.auto_enable = auto_enable
        self.disable_on_finish = disable_on_finish
        self.extraction_kwargs = extraction_kwargs or {}
        self.transform_resolver = transform_resolver

    def execute(self, robot: Any, writer: Optional[Any] = None) -> StructuredLightScanResult:
        result = StructuredLightScanResult(task_name=self.name, episode_index=self.episode_index)
        if writer is not None:
            writer.start_episode(
                episode_index=self.episode_index,
                task=self.name,
                metadata={
                    "camera_name": self.camera_name,
                    "waypoint_count": len(self.waypoints),
                    "intrinsics": self.intrinsics.as_dict(),
                    "laser_plane_in_camera": self.laser_plane_in_camera.as_dict(),
                },
            )

        if self.auto_enable:
            robot.enable()

        try:
            for waypoint_index, waypoint in enumerate(self.waypoints):
                robot.execute_action(waypoint.action)
                if waypoint.settle_time_s > 0:
                    time.sleep(waypoint.settle_time_s)

                synced_frame = robot.capture_camera_frame(self.camera_name)
                observation = extract_laser_line(synced_frame.frame.image, **self.extraction_kwargs)
                observation = type(observation)(
                    timestamp=synced_frame.frame.timestamp,
                    uv_points=observation.uv_points,
                    intensities=observation.intensities,
                    image_shape=observation.image_shape,
                    metadata={**observation.metadata, "camera_name": self.camera_name},
                )
                transform = self.transform_resolver(synced_frame) if self.transform_resolver is not None else None
                reconstructed = reconstruct_laser_points(
                    observation=observation,
                    intrinsics=self.intrinsics,
                    laser_plane_in_camera=self.laser_plane_in_camera,
                    transform=transform,
                    output_frame=transform.to_frame if transform is not None else "camera",
                )
                result.frames.append(reconstructed)
                if writer is not None:
                    done = waypoint_index == len(self.waypoints) - 1
                    writer.add_frame(
                        synced_frame=synced_frame,
                        reconstructed=reconstructed,
                        action=waypoint.action,
                        done=done,
                        extra={"waypoint_index": waypoint_index, "waypoint_metadata": waypoint.metadata},
                    )
        finally:
            if writer is not None:
                dataset_path = writer.finalize_episode()
                result.dataset_path = str(dataset_path)
            if self.disable_on_finish:
                robot.disable()
            result.close()

        return result

    @staticmethod
    def from_joint_trajectory(
        name: str,
        camera_name: str,
        trajectory: List[List[float]],
        intrinsics: CameraIntrinsics,
        laser_plane_in_camera: Plane3D,
        speed: Optional[float] = None,
        settle_time_s: float = 0.5,
        episode_index: int = 0,
        auto_enable: bool = False,
        disable_on_finish: bool = False,
        extraction_kwargs: Optional[Dict[str, Any]] = None,
        transform_resolver: Optional[TransformResolver] = None,
    ) -> "StructuredLightScanTask":
        waypoints = [
            StructuredLightWaypoint(
                action=RobotAction(joint_positions=joint_positions, speed=speed),
                settle_time_s=settle_time_s,
                metadata={"trajectory_type": "joint"},
            )
            for joint_positions in trajectory
        ]
        return StructuredLightScanTask(
            name=name,
            camera_name=camera_name,
            waypoints=waypoints,
            intrinsics=intrinsics,
            laser_plane_in_camera=laser_plane_in_camera,
            episode_index=episode_index,
            auto_enable=auto_enable,
            disable_on_finish=disable_on_finish,
            extraction_kwargs=extraction_kwargs,
            transform_resolver=transform_resolver,
        )
