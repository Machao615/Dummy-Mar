from __future__ import annotations

from argparse import ArgumentParser
from pathlib import Path
import json

from .controller import SerialMotionController, UniformScanConfig


def build_parser() -> ArgumentParser:
    parser = ArgumentParser(description="Uniform joint scan controller for DummyRobot")
    parser.add_argument("--config", type=str, help="Path to a JSON config file")
    parser.add_argument("--port", type=str, help="Serial port, for example COM3 or /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--joint-index", type=int, default=3, help="1-based joint index")
    parser.add_argument("--step-angle", type=float, default=-0.1, help="Angle increment per step in degrees")
    parser.add_argument("--steps", type=int, default=200, help="Total step count")
    parser.add_argument("--speed", type=float, default=50.0)
    parser.add_argument("--pause", type=float, default=0.5, help="Pause after each motion step in seconds")
    parser.add_argument("--initial-joints", type=str, default="0,-60,150,0,0,0")
    parser.add_argument("--initial-settle", type=float, default=3.0)
    parser.add_argument("--return-settle", type=float, default=3.0)
    parser.add_argument("--sequential", action="store_true")
    parser.add_argument("--enable-before-scan", action="store_true")
    parser.add_argument("--disable-after-scan", action="store_true")
    parser.add_argument("--no-return", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--log-path", type=str)
    return parser


def parse_joint_values(raw: str) -> list[float]:
    values = [float(item.strip()) for item in raw.split(",") if item.strip()]
    if len(values) != 6:
        raise ValueError("initial-joints must contain exactly 6 comma-separated values")
    return values


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    if args.config:
        config = UniformScanConfig.from_json(args.config)
    else:
        if not args.port:
            parser.error("--port is required when --config is not provided")
        config = UniformScanConfig(
            port=args.port,
            baudrate=args.baudrate,
            joint_index=args.joint_index,
            step_angle_deg=args.step_angle,
            total_steps=args.steps,
            speed=args.speed,
            pause_time_s=args.pause,
            initial_joints=parse_joint_values(args.initial_joints),
            initial_settle_s=args.initial_settle,
            return_settle_s=args.return_settle,
            sequential=args.sequential,
            enable_before_scan=args.enable_before_scan,
            disable_after_scan=args.disable_after_scan,
            return_to_initial=not args.no_return,
            log_path=args.log_path,
        )

    with SerialMotionController(config=config, dry_run=args.dry_run) as controller:
        result = controller.run_uniform_joint_scan()
    print(json.dumps(result.as_dict(), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
