import logging
import os
import sys
import time

import serial.tools.list_ports

sys.path.insert(0, os.path.abspath("src"))

from dummy_robot import (
    CallbackCamera,
    CameraIntrinsics,
    DummyRobot,
    LeRobotDatasetWriter,
    Plane3D,
    RigidTransform,
    RobotAction,
    StructuredLightDatasetWriter,
    StructuredLightScanTask,
    draw_laser_overlay,
    extract_laser_line,
    generate_synthetic_laser_image,
)


logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)


def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial devices found.")
        return []
    print("Available serial devices:")
    for port in ports:
        print(f"  {port.device} - {port.description}")
    return [port.device for port in ports]


def fake_camera_frame():
    image = generate_synthetic_laser_image()
    observation = extract_laser_line(image)
    return {
        "frame_id": "synthetic_camera",
        "image": image,
        "metadata": {"synthetic": True, "overlay_preview_shape": list(draw_laser_overlay(image, observation).shape)},
    }


def main():
    print("=== DummyRobot SDK connection test ===\n")
    available_ports = list_serial_ports()
    if not available_ports:
        return

    target_port = available_ports[0]
    print(f"\nConnecting to {target_port}")

    with DummyRobot(target_port) as robot:
        robot.register_camera(CallbackCamera("scan_camera", fake_camera_frame))

        joints = robot.get_joint_positions()
        print(f"Current joints: {joints.as_dict()}")

        pose = robot.get_tool_pose()
        print(f"Current tool pose: {pose.as_list()}")

        observation = robot.get_lerobot_observation()
        print(f"Observation snapshot: {observation}")

        synced_frame = robot.capture_camera_frame("scan_camera")
        print(f"Synced camera frame: {synced_frame.as_dict()}")

        scan_task = StructuredLightScanTask.from_joint_trajectory(
            name="demo_leaf_scan",
            camera_name="scan_camera",
            trajectory=[joints.as_list()],
            intrinsics=CameraIntrinsics(fx=820.0, fy=820.0, cx=320.0, cy=240.0),
            laser_plane_in_camera=Plane3D(normal_xyz=[0.15, 0.0, -0.99], offset=0.18),
            speed=5,
            settle_time_s=0.1,
            transform_resolver=lambda frame: RigidTransform(
                rotation=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                translation_xyz=[0.0, 0.0, 0.0],
                from_frame="camera",
                to_frame="robot_base",
            ),
        )
        structured_writer = StructuredLightDatasetWriter(root_dir="test_outputs/structured_light_connection_demo")
        structured_result = robot.execute_structured_light_scan_task(scan_task, writer=structured_writer)
        print(f"Structured-light scan result: {structured_result.as_dict()}")

        lerobot_writer = LeRobotDatasetWriter(root_dir="test_outputs/lerobot_demo", repo_id="local/dummy-demo", fps=1)
        lerobot_writer.start_episode(episode_index=0, task="dummy_demo")
        demo_action = RobotAction(joint_positions=joints.as_list(), speed=5)
        lerobot_observation = robot.get_observation(include_pose=True, include_sensors=False)
        lerobot_writer.add_robot_frame(observation=lerobot_observation, action=demo_action, done=False)
        print(f"LeRobot export result: {lerobot_writer.finalize_episode()}")

        user_input = input("\nSend !START? (y/n): ").strip().lower()
        if user_input != "y":
            return

        print(robot.enable())
        time.sleep(1)

        action_input = input("Execute a small joint action (+5 deg on each joint)? (y/n): ").strip().lower()
        if action_input == "y":
            target_joints = [value + 5.0 for value in joints.as_list()]
            action = RobotAction(joint_positions=target_joints, speed=10)
            print(robot.execute_action(action))
            time.sleep(3)
            print(f"Updated joints: {robot.get_joint_positions().as_list()}")

        disable_input = input("Send !DISABLE before exit? (y/n): ").strip().lower()
        if disable_input != "n":
            print(robot.disable())


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        logger.exception("Connection test failed: %s", exc)
