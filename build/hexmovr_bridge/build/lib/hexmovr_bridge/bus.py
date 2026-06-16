from __future__ import annotations

import time
from dataclasses import dataclass
from threading import RLock
from typing import Callable, Optional


@dataclass(frozen=True)
class CanFrame:
    arbitration_id: int
    data: bytes
    is_rx: bool = True

    @property
    def can_id(self) -> int:
        return self.arbitration_id

    @property
    def is_rtr(self) -> bool:
        return False

    @property
    def is_error(self) -> bool:
        return False


class CanBus:
    """Tiny python-can wrapper for Linux SocketCAN."""

    def __init__(self, channel: str) -> None:
        try:
            import can
        except ImportError as exc:
            raise RuntimeError("python-can is required for hexmovr_bridge") from exc

        self._can = can
        try:
            self._bus = can.Bus(interface="socketcan", channel=channel)
        except TypeError:
            self._bus = can.interface.Bus(bustype="socketcan", channel=channel)
        self._channel = channel
        self._lock = RLock()

    @property
    def channel(self) -> str:
        return self._channel

    def send(self, frame: CanFrame) -> None:
        payload = bytes(frame.data)
        if len(payload) > 8:
            raise ValueError("classic CAN frame payload must be 8 bytes or less")
        message = self._can.Message(
            arbitration_id=int(frame.arbitration_id),
            data=payload,
            is_extended_id=False,
        )
        with self._lock:
            self._bus.send(message)

    def recv(self, timeout: float) -> Optional[CanFrame]:
        with self._lock:
            message = self._bus.recv(timeout=max(float(timeout), 0.0))
        if message is None:
            return None
        return CanFrame(
            arbitration_id=int(message.arbitration_id),
            data=bytes(message.data),
            is_rx=True,
        )

    def send_frame(self, can_id: int, data: bytes) -> None:
        self.send(CanFrame(arbitration_id=int(can_id), data=bytes(data), is_rx=False))

    def read_frame(self, timeout_s: Optional[float] = None) -> Optional[CanFrame]:
        timeout = 0.0 if timeout_s is None else max(float(timeout_s), 0.0)
        return self.recv(timeout)

    def drain(self) -> None:
        deadline = time.monotonic() + 0.002
        while time.monotonic() < deadline:
            if self.read_frame(timeout_s=0.0) is None:
                break

    def request(
        self,
        can_id: int,
        data: bytes,
        predicate: Callable[[CanFrame], bool],
        timeout_s: Optional[float] = None,
        drain_before_send: bool = True,
    ) -> Optional[CanFrame]:
        if drain_before_send:
            self.drain()
        self.send_frame(can_id, data)
        timeout = 0.05 if timeout_s is None else max(float(timeout_s), 0.0)
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return None
            frame = self.read_frame(timeout_s=remaining)
            if frame is None:
                return None
            if predicate(frame):
                return frame

    def shutdown(self) -> None:
        with self._lock:
            shutdown = getattr(self._bus, "shutdown", None)
            if shutdown is not None:
                shutdown()
