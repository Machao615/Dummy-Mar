from __future__ import annotations

from pathlib import Path
from threading import RLock
from typing import List, Optional, Protocol
import time

from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from dummy_robot import DummyRobot, SO101MirrorBridge, VirtualDummyRobot


class RobotLike(Protocol):
    @property
    def is_connected(self) -> bool: ...

    def connect(self): ...

    def disconnect(self) -> None: ...

    def enable(self) -> str: ...

    def disable(self) -> str: ...

    def stop(self) -> str: ...

    def home(self) -> str: ...

    def reset(self) -> str: ...

    def send_raw(self, command: str, expect_response: bool = True): ...

    def get_joint_positions(self): ...

    def get_tool_pose(self): ...

    def move_joints(self, joints, speed=None, sequential: bool = False) -> str: ...

    def move_single_joint(
        self, joint_index: int, target=None, delta=None, speed=None, sequential: bool = False
    ) -> str: ...

    def move_to_pose(self, x: float, y: float, z: float, a: float, b: float, c: float, speed=None) -> str: ...


class ConnectRequest(BaseModel):
    port: str = ""
    baudrate: int = 115200
    timeout: float = 1.0
    mode: str = "real"


class RawCommandRequest(BaseModel):
    command: str
    expect_response: bool = True


class MoveJointsRequest(BaseModel):
    joints: List[float] = Field(..., min_length=6, max_length=6)
    speed: Optional[float] = None
    sequential: bool = False


class MovePoseRequest(BaseModel):
    pose: List[float] = Field(..., min_length=6, max_length=6)
    speed: Optional[float] = None


class MoveSingleJointRequest(BaseModel):
    joint_index: int = Field(..., ge=1, le=6)
    target: Optional[float] = None
    delta: Optional[float] = None
    speed: Optional[float] = None
    sequential: bool = False


class TeleopSourceRequest(BaseModel):
    source: str


class TeleopStartRequest(BaseModel):
    port: str
    poll_hz: float = Field(20.0, ge=1.0, le=120.0)
    leader_id: str = "dummy-webui-so101"


class RobotSession:
    def __init__(self) -> None:
        self._lock = RLock()
        self.robot: Optional[RobotLike] = None
        self.mode = "real"
        self.connected_port: Optional[str] = None
        self.connected_baudrate: Optional[int] = None
        self.is_enabled = False
        self.last_command: Optional[str] = None
        self.last_response: Optional[str] = None
        self.last_error: Optional[str] = None
        self.teleop_source = "manual"
        self.teleop_bridge: Optional[SO101MirrorBridge] = None
        self.last_state = {
            "connected": False,
            "enabled": False,
            "mode": "real",
            "port": None,
            "baudrate": None,
            "joints": None,
            "pose": None,
            "last_command": None,
            "last_response": None,
            "last_error": None,
            "updated_at": None,
            "teleop": self._default_teleop_state(),
        }

    def list_ports(self) -> List[str]:
        return DummyRobot.list_serial_ports()

    def connect(self, request: ConnectRequest) -> None:
        with self._lock:
            self.disconnect()
            mode = (request.mode or "real").strip().lower()
            if mode not in {"real", "virtual"}:
                raise HTTPException(status_code=400, detail=f"Unsupported mode: {request.mode}")

            if mode == "real":
                if not request.port:
                    raise HTTPException(status_code=400, detail="Serial port is required in real mode")
                robot: RobotLike = DummyRobot(request.port, baudrate=request.baudrate, timeout=request.timeout)
                connected_port = request.port
            else:
                robot = VirtualDummyRobot(port="VIRTUAL", baudrate=request.baudrate, timeout=request.timeout)
                connected_port = "VIRTUAL"

            robot.connect()
            self.robot = robot
            self.mode = mode
            self.connected_port = connected_port
            self.connected_baudrate = request.baudrate
            self.is_enabled = False
            self.last_error = None
            self._refresh_state()

    def disconnect(self) -> None:
        with self._lock:
            self._stop_teleop_locked(reset_source=True, clear_error=True)
            if self.robot is not None:
                try:
                    self.robot.disconnect()
                finally:
                    self.robot = None
            self.connected_port = None
            self.connected_baudrate = None
            self.is_enabled = False
            self.mode = "real"
            self.last_error = None
            self.last_state.update(
                {
                    "connected": False,
                    "enabled": False,
                    "mode": "real",
                    "port": None,
                    "baudrate": None,
                    "joints": None,
                    "pose": None,
                    "updated_at": time.time(),
                    "teleop": self._default_teleop_state(),
                }
            )

    def set_teleop_source(self, request: TeleopSourceRequest) -> dict:
        with self._lock:
            source = (request.source or "").strip().lower()
            if source not in {"manual", "so101"}:
                raise HTTPException(status_code=400, detail=f"Unsupported teleop source: {request.source}")
            if source == "manual":
                self._stop_teleop_locked(reset_source=False, clear_error=True)
            self.teleop_source = source
            self._refresh_state()
            return dict(self.last_state["teleop"])

    def start_teleop(self, request: TeleopStartRequest) -> dict:
        with self._lock:
            if self.teleop_source != "so101":
                raise HTTPException(status_code=400, detail="Select SO101 Mirror before starting teleoperation")
            if self.mode != "virtual":
                raise HTTPException(status_code=400, detail="SO101 mirror is only available in virtual mode")
            robot = self._require_robot()
            if not isinstance(robot, VirtualDummyRobot):
                raise HTTPException(status_code=400, detail="SO101 mirror requires a virtual robot session")
            self._stop_teleop_locked(reset_source=False, clear_error=True)
            bridge = SO101MirrorBridge(
                robot=robot,
                port=request.port,
                poll_hz=request.poll_hz,
                leader_id=request.leader_id,
            )
            try:
                bridge.start()
            except Exception as exc:
                self.last_error = str(exc)
                self._refresh_state()
                raise HTTPException(status_code=400, detail=str(exc)) from exc
            self.teleop_bridge = bridge
            self.last_response = f"ok SO101_MIRROR_STARTED {request.port.strip()}"
            self.last_error = None
            self._refresh_state()
            return dict(self.last_state["teleop"])

    def stop_teleop(self) -> dict:
        with self._lock:
            self._stop_teleop_locked(reset_source=False, clear_error=True)
            self.last_response = "ok SO101_MIRROR_STOPPED"
            self._refresh_state()
            return dict(self.last_state["teleop"])

    def get_teleop_state(self) -> dict:
        with self._lock:
            self._refresh_state()
            return dict(self.last_state["teleop"])

    def enable(self) -> str:
        response = self._call("!START", lambda robot: robot.enable())
        self.is_enabled = True
        return response

    def disable(self) -> str:
        response = self._call("!DISABLE", lambda robot: robot.disable())
        self.is_enabled = False
        return response

    def stop(self) -> str:
        response = self._call("!STOP", lambda robot: robot.stop())
        self.is_enabled = False
        return response

    def home(self) -> str:
        return self._call("!HOME", lambda robot: robot.home())

    def reset(self) -> str:
        return self._call("!RESET", lambda robot: robot.reset())

    def send_raw(self, request: RawCommandRequest):
        with self._lock:
            self._guard_manual_motion_locked(request.command)
        return self._call(request.command, lambda robot: robot.send_raw(request.command, request.expect_response))

    def move_joints(self, request: MoveJointsRequest) -> str:
        with self._lock:
            self._guard_manual_motion_locked()
        return self._call(
            f"move_joints:{request.joints}",
            lambda robot: robot.move_joints(request.joints, speed=request.speed, sequential=request.sequential),
        )

    def move_pose(self, request: MovePoseRequest) -> str:
        with self._lock:
            self._guard_manual_motion_locked()
        return self._call(
            f"move_pose:{request.pose}",
            lambda robot: robot.move_to_pose(*request.pose, speed=request.speed),
        )

    def move_single_joint(self, request: MoveSingleJointRequest) -> str:
        if (request.target is None and request.delta is None) or (
            request.target is not None and request.delta is not None
        ):
            raise HTTPException(status_code=400, detail="Specify exactly one of target or delta")
        with self._lock:
            self._guard_manual_motion_locked()
        return self._call(
            f"move_single_joint:{request.joint_index}",
            lambda robot: robot.move_single_joint(
                joint_index=request.joint_index,
                target=request.target,
                delta=request.delta,
                speed=request.speed,
                sequential=request.sequential,
            ),
        )

    def get_state(self) -> dict:
        with self._lock:
            self._refresh_state()
            return dict(self.last_state)

    def _call(self, command: str, fn):
        with self._lock:
            robot = self._require_robot()
            try:
                self.last_command = command
                self.last_error = None
                response = fn(robot)
                self.last_response = response
                self._refresh_state()
                return response
            except HTTPException:
                raise
            except Exception as exc:
                self.last_error = str(exc)
                self._refresh_state()
                raise HTTPException(status_code=400, detail=str(exc)) from exc

    def _require_robot(self) -> RobotLike:
        if self.robot is None or not self.robot.is_connected:
            raise HTTPException(status_code=400, detail="Robot is not connected")
        return self.robot

    def _default_teleop_state(self) -> dict:
        return {
            "source": self.teleop_source,
            "active": False,
            "connected": False,
            "fps": None,
            "last_sample_at": None,
            "last_error": None,
            "port": None,
        }

    def _stop_teleop_locked(self, reset_source: bool, clear_error: bool) -> None:
        bridge = self.teleop_bridge
        self.teleop_bridge = None
        if bridge is not None:
            bridge.stop()
        if reset_source:
            self.teleop_source = "manual"
        elif self.teleop_source not in {"manual", "so101"}:
            self.teleop_source = "manual"
        if clear_error and self.last_state.get("teleop"):
            self.last_state["teleop"]["last_error"] = None

    def _guard_manual_motion_locked(self, raw_command: Optional[str] = None) -> None:
        if self.teleop_bridge is None or not self.teleop_bridge.active:
            return
        if raw_command is not None and not self._is_motion_raw_command(raw_command):
            return
        raise HTTPException(status_code=409, detail="SO101 mirror is active; manual motion commands are disabled")

    @staticmethod
    def _is_motion_raw_command(command: str) -> bool:
        normalized = (command or "").strip()
        return normalized.startswith("&") or normalized.startswith(">") or normalized.startswith("@")

    def _snapshot_teleop_locked(self) -> dict:
        if self.teleop_bridge is None:
            return self._default_teleop_state()
        snapshot = self.teleop_bridge.snapshot()
        snapshot["source"] = self.teleop_source
        return snapshot

    def _refresh_state(self) -> None:
        connected = self.robot is not None and self.robot.is_connected
        joints = None
        pose = None
        if connected:
            try:
                joints = self.robot.get_joint_positions().as_list()
                pose = self.robot.get_tool_pose().as_list()
            except Exception as exc:
                self.last_error = str(exc)
        self.last_state = {
            "connected": connected,
            "enabled": self.is_enabled,
            "mode": self.mode,
            "port": self.connected_port,
            "baudrate": self.connected_baudrate,
            "joints": joints,
            "joint_names": [f"J{index}" for index in range(1, 7)],
            "pose": pose,
            "pose_labels": ["X", "Y", "Z", "A", "B", "C"],
            "last_command": self.last_command,
            "last_response": self.last_response,
            "last_error": self.last_error,
            "updated_at": time.time(),
            "teleop": self._snapshot_teleop_locked(),
        }


def create_app() -> FastAPI:
    app = FastAPI(title="DummyRobot Web Console", version="0.3.0")
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_methods=["*"],
        allow_headers=["*"],
    )

    @app.middleware("http")
    async def disable_cache(request: Request, call_next):
        response = await call_next(request)
        if request.method == "GET" and (
            request.url.path == "/"
            or request.url.path.startswith("/assets/")
            or request.url.path.startswith("/models/")
        ):
            response.headers["Cache-Control"] = "no-store, max-age=0"
            response.headers["Pragma"] = "no-cache"
            response.headers["Expires"] = "0"
        return response

    session = RobotSession()
    static_dir = Path(__file__).parent / "static"
    model_dir = Path(__file__).resolve().parents[3] / "2.Model" / "3D鎵撳嵃STL鏂囦欢"
    app.mount("/assets", StaticFiles(directory=static_dir), name="assets")
    if model_dir.exists():
        app.mount("/models", StaticFiles(directory=model_dir), name="models")

    @app.get("/")
    def index():
        return FileResponse(static_dir / "index.html")

    @app.get("/api/ports")
    def list_ports():
        return {"ports": session.list_ports()}

    @app.post("/api/connect")
    def connect(request: ConnectRequest):
        session.connect(request)
        return session.get_state()

    @app.post("/api/disconnect")
    def disconnect():
        session.disconnect()
        return session.get_state()

    @app.get("/api/state")
    def get_state():
        return session.get_state()

    @app.get("/api/teleop/state")
    def get_teleop_state():
        return session.get_teleop_state()

    @app.post("/api/teleop/source")
    def set_teleop_source(request: TeleopSourceRequest):
        return {"teleop": session.set_teleop_source(request), "state": session.get_state()}

    @app.post("/api/teleop/start")
    def start_teleop(request: TeleopStartRequest):
        return {"teleop": session.start_teleop(request), "state": session.get_state()}

    @app.post("/api/teleop/stop")
    def stop_teleop():
        return {"teleop": session.stop_teleop(), "state": session.get_state()}

    @app.post("/api/commands/enable")
    def enable():
        return {"response": session.enable(), "state": session.get_state()}

    @app.post("/api/commands/disable")
    def disable():
        return {"response": session.disable(), "state": session.get_state()}

    @app.post("/api/commands/stop")
    def stop():
        return {"response": session.stop(), "state": session.get_state()}

    @app.post("/api/commands/home")
    def home():
        return {"response": session.home(), "state": session.get_state()}

    @app.post("/api/commands/reset")
    def reset():
        return {"response": session.reset(), "state": session.get_state()}

    @app.post("/api/commands/raw")
    def send_raw(request: RawCommandRequest):
        return {"response": session.send_raw(request), "state": session.get_state()}

    @app.post("/api/move/joints")
    def move_joints(request: MoveJointsRequest):
        return {"response": session.move_joints(request), "state": session.get_state()}

    @app.post("/api/move/pose")
    def move_pose(request: MovePoseRequest):
        return {"response": session.move_pose(request), "state": session.get_state()}

    @app.post("/api/move/joint")
    def move_single_joint(request: MoveSingleJointRequest):
        return {"response": session.move_single_joint(request), "state": session.get_state()}

    return app


app = create_app()
