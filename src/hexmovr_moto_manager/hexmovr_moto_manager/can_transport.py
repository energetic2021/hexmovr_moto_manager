import errno
import select
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional


CAN_EFF_MASK = 0x1FFFFFFF
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_FRAME_STRUCT = struct.Struct("=IB3x8s")


@dataclass(frozen=True)
class CanFrame:
    can_id: int
    data: bytes
    is_rtr: bool = False
    is_error: bool = False


class SocketCanError(RuntimeError):
    """Raised when SocketCAN transport operations fail."""


class SocketCanTransport:
    """Small synchronous SocketCAN helper for classic 8-byte CAN frames."""

    def __init__(self, interface: str, default_timeout_s: float = 0.05) -> None:
        self._interface = interface
        self._default_timeout_s = max(default_timeout_s, 0.001)
        self._socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            socket.if_nametoindex(interface)
        except OSError as exc:
            self._socket.close()
            raise SocketCanError(
                f"CAN interface '{interface}' does not exist. Configure SocketCAN first."
            ) from exc
        try:
            self._socket.bind((interface,))
        except OSError as exc:
            self._socket.close()
            if exc.errno == errno.ENODEV:
                raise SocketCanError(
                    f"CAN interface '{interface}' is not available. Configure SocketCAN first."
                ) from exc
            raise SocketCanError(
                f"Failed to bind CAN interface '{interface}': {exc.strerror or exc}"
            ) from exc
        self._lock = threading.Lock()

    @property
    def interface(self) -> str:
        return self._interface

    def close(self) -> None:
        with self._lock:
            try:
                self._socket.close()
            except OSError:
                pass

    def send_frame(self, can_id: int, data: bytes) -> None:
        payload = bytes(data)
        if len(payload) > 8:
            raise SocketCanError("Hexmovr classic CAN payload must be 8 bytes or less")
        raw = CAN_FRAME_STRUCT.pack(can_id, len(payload), payload.ljust(8, b"\x00"))
        with self._lock:
            self._socket.send(raw)

    def read_frame(self, timeout_s: Optional[float] = None) -> Optional[CanFrame]:
        timeout = self._default_timeout_s if timeout_s is None else max(timeout_s, 0.0)
        with self._lock:
            ready, _, _ = select.select([self._socket], [], [], timeout)
            if not ready:
                return None
            raw = self._socket.recv(CAN_FRAME_STRUCT.size)
        can_id, dlc, data = CAN_FRAME_STRUCT.unpack(raw)
        return CanFrame(
            can_id=can_id & CAN_EFF_MASK,
            data=bytes(data[:dlc]),
            is_rtr=bool(can_id & CAN_RTR_FLAG),
            is_error=bool(can_id & CAN_ERR_FLAG),
        )

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
        timeout = self._default_timeout_s if timeout_s is None else max(timeout_s, 0.0)
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
