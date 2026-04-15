import json
import os
import sys
import time

sys.path.insert(0, os.path.abspath("src"))

from dummy_robot import (
    CameraFrame,
    CameraIntrinsics,
    JointPositions,
    Plane3D,
    Pose6D,
    ReconstructedPointCloudFrame,
    RigidTransform,
    RobotAction,
    RobotStateSnapshot,
    StructuredLightDatasetWriter,
    SyncedCameraFrame,
    draw_laser_overlay,
    extract_laser_line,
    fuse_point_cloud_frames,
    generate_synthetic_laser_image,
    project_height_map,
    reconstruct_laser_points,
)


def make_snapshot(timestamp: float, x_offset: float) -> RobotStateSnapshot:
    return RobotStateSnapshot(
        timestamp=timestamp,
        joints=JointPositions([0.0, 5.0, 10.0, 0.0, 0.0, 0.0]),
        tool_pose=Pose6D(x_offset, 0.0, 250.0, -180.0, 0.0, 0.0),
    )


def main() -> None:
    root_dir = "test_outputs/structured_light_demo"
    intrinsics = CameraIntrinsics(fx=820.0, fy=820.0, cx=320.0, cy=240.0, image_width=640, image_height=480)
    laser_plane = Plane3D(normal_xyz=[0.15, 0.0, -0.99], offset=0.18)
    writer = StructuredLightDatasetWriter(root_dir=root_dir, project_name="dummy_structured_light_demo")
    writer.start_episode(episode_index=0, task="offline_line_laser_demo")

    reconstructed_frames = []

    for frame_index in range(3):
        timestamp = time.time() + frame_index * 0.05
        image = generate_synthetic_laser_image(intercept=180 + frame_index * 18, slope=0.08)
        observation = extract_laser_line(image=image, color="blue", channel_order="rgb", axis="columns")
        observation = type(observation)(
            timestamp=timestamp,
            uv_points=observation.uv_points,
            intensities=observation.intensities,
            image_shape=observation.image_shape,
            metadata=observation.metadata,
        )
        transform = RigidTransform(
            rotation=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation_xyz=[frame_index * 0.015, 0.0, 0.0],
            from_frame="camera",
            to_frame="robot_base",
        )
        reconstructed = reconstruct_laser_points(
            observation=observation,
            intrinsics=intrinsics,
            laser_plane_in_camera=laser_plane,
            transform=transform,
            output_frame="robot_base",
        )
        reconstructed_frames.append(reconstructed)

        synced_frame = SyncedCameraFrame(
            frame=CameraFrame(timestamp=timestamp, image=image, frame_id="synthetic_camera", metadata={"synthetic": True}),
            pose_before=make_snapshot(timestamp - 0.01, x_offset=frame_index * 10.0),
            pose_after=make_snapshot(timestamp + 0.01, x_offset=frame_index * 10.0),
            pose_midpoint=make_snapshot(timestamp, x_offset=frame_index * 10.0),
        )
        writer.add_frame(
            synced_frame=synced_frame,
            reconstructed=reconstructed,
            action=RobotAction(joint_positions=synced_frame.pose_midpoint.joints.as_list(), speed=5.0),
            done=frame_index == 2,
            extra={"overlay_shape": list(draw_laser_overlay(image, observation).shape)},
        )

    output_dir = writer.finalize_episode()
    fused = fuse_point_cloud_frames(reconstructed_frames)
    height_map, height_map_meta = project_height_map(fused, grid_resolution=0.003)

    summary = {
        "dataset_path": str(output_dir),
        "fused_point_count": int(fused.shape[0]),
        "height_map_shape": list(height_map.shape),
        "height_map_metadata": height_map_meta,
    }
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
