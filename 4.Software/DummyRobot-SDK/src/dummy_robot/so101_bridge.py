from __future__ import annotations

from dataclasses import dataclass
from threading import Event, Lock, Thread
from typing import Mapping, Optional, Sequence
import time

from .virtual_robot import VirtualDummyRobot


@dataclass(frozen=True)
class JointMirrorMapping:
    source_key: Optional[str]
    invert: float = 1.0
    offset: float = 0.0
    fallback: float = 0.0


DEFAULT_SO101_MIRROR_MAPPING: tuple[JointMirrorMapping, ...] = (
    JointMirrorMapping("shoulder_pan.pos"),
    JointMirrorMapping("shoulder_lift.pos"),
    JointMirrorMapping("elbow_flex.pos"),
    JointMirrorMapping("wrist_flex.pos"),
    JointMirrorMapping(None, fallback=0.0),
    JointMirrorMapping("wrist_roll.pos"),
)


class SO101MirrorBridge:
    """Mirror a LeRobot SO101 leader arm into a VirtualDummyRobot."""

    def __init__(
        self,
        robot: VirtualDummyRobot,
        port: str,
        poll_hz: float = 20.0,
        leader_id: str = "dummy-webui-so101",
        mapping: Sequence[JointMirrorMapping] = DEFAULT_SO101_MIRROR_MAPPING,
    ) -> None:
        self.robot = robot
        self.port = port.strip()
        self.poll_hz = max(1.0, float(poll_hz))
        self.leader_id = leader_id
        self.mapping = tuple(mapping)
        self._lock = Lock()
        self._stop_event = Event()
        self._thread: Optional[Thread] = None
        self._leader = None
        self._active = False
        self._connected = False
        self._fps: Optional[float] = None
        self._last_sample_at: Optional[float] = None
        self._last_error: Optional[str] = None

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "source": "so101",
                "active": self._active,
                "connected": self._connected,
                "fps": round(self._fps, 2) if self._fps is not None else None,
                "last_sample_at": self._last_sample_at,
                "last_error": self._last_error,
                "port": self.port or None,
            }

    def start(self) -> None:
        if not self.port:
            raise ValueError("SO101 serial port is required")

        with self._lock:
            if self._active:
                raise RuntimeError("SO101 mirror is already active")
            self._last_error = None

        leader = self._create_leader()
        try:
            leader.connect(calibrate=False)
            action = leader.get_action()
            joints = self._map_action(action)
            self.robot.move_joints(joints, sequential=False)
        except Exception:
            try:
                leader.disconnect()
            except Exception:
                pass
            raise

        with self._lock:
            self._leader = leader
            self._stop_event.clear()
            self._active = True
            self._connected = True
            self._last_sample_at = time.time()
            self._thread = Thread(target=self._run_loop, name="so101-mirror-bridge", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        with self._lock:
            self._stop_event.set()
            thread = self._thread
        if thread and thread.is_alive():
            thread.join(timeout=2.0)
        self._shutdown(clear_error=True)

    def _create_leader(self):
        try:
            from lerobot.teleoperators.so101_leader.config_so101_leader import SO101LeaderConfig
            from lerobot.teleoperators.so101_leader.so101_leader import SO101Leader
        except Exception as exc:  # pragma: no cover - depends on local LeRobot installation
            raise RuntimeError(f"Unable to import LeRobot SO101 leader API: {exc}") from exc

        config = SO101LeaderConfig(id=self.leader_id, port=self.port, use_degrees=True)
        return SO101Leader(config)

    def _run_loop(self) -> None:
        interval = 1.0 / self.poll_hz
        previous_sample_at = self._last_sample_at
        try:
            while not self._stop_event.is_set():
                loop_started = time.perf_counter()
                action = self._leader.get_action()
                joints = self._map_action(action)
                self.robot.move_joints(joints, sequential=False)
                now = time.time()
                with self._lock:
                    self._connected = True
                    self._last_error = None
                    self._last_sample_at = now
                    if previous_sample_at:
                        dt = now - previous_sample_at
                        self._fps = (1.0 / dt) if dt > 0 else None
                previous_sample_at = now
                remaining = interval - (time.perf_counter() - loop_started)
                if remaining > 0:
                    self._stop_event.wait(remaining)
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
                self._connected = False
                self._active = False
        finally:
            self._shutdown(clear_error=False)

    def _shutdown(self, clear_error: bool) -> None:
        leader = None
        with self._lock:
            leader = self._leader
            self._leader = None
            self._thread = None
            self._active = False
            self._connected = False
            self._fps = None
            if clear_error:
                self._last_error = None
        if leader is not None:
            try:
                leader.disconnect()
            except Exception:
                pass

    def _map_action(self, action: Mapping[str, float]) -> list[float]:
        current = self.robot.get_joint_positions().as_list()
        targets = list(current)
        for index, mapping in enumerate(self.mapping):
            if index >= len(targets):
                break
            if mapping.source_key is None:
                targets[index] = float(mapping.fallback)
                continue
            if mapping.source_key not in action:
                raise KeyError(f"SO101 action missing key: {mapping.source_key}")
            raw_value = float(action[mapping.source_key])
            targets[index] = raw_value * mapping.invert + mapping.offset
        return targets
