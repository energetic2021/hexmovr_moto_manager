from dataclasses import dataclass
from typing import Callable, Optional

from hexmovr_bridge.bus import CanBus, CanFrame as BridgeCanFrame


@dataclass(frozen=True)
class CanFrame:
    can_id: int
    data: bytes
    is_rtr: bool = False
    is_error: bool = False


class SocketCanError(RuntimeError):
    """Raised when SocketCAN transport operations fail."""


class SocketCanTransport:
    """Compatibility wrapper around hexmovr_bridge.bus.CanBus."""

    def __init__(self, interface: str, default_timeout_s: float = 0.05) -> None:
        self._interface = interface
        self._default_timeout_s = max(default_timeout_s, 0.001)
        try:
            self._bus = CanBus(interface)
        except Exception as exc:
            raise SocketCanError(
                f"Failed to open CAN interface '{interface}' with hexmovr_bridge: {exc}"
            ) from exc

    @property
    def interface(self) -> str:
        return self._interface

    def close(self) -> None:
        self._bus.shutdown()

    def send_frame(self, can_id: int, data: bytes) -> None:
        try:
            self._bus.send_frame(can_id, data)
        except Exception as exc:
            raise SocketCanError(f"Failed to send CAN frame on '{self._interface}': {exc}") from exc

    def read_frame(self, timeout_s: Optional[float] = None) -> Optional[CanFrame]:
        timeout = self._default_timeout_s if timeout_s is None else max(timeout_s, 0.0)
        try:
            frame = self._bus.read_frame(timeout_s=timeout)
        except Exception as exc:
            raise SocketCanError(f"Failed to read CAN frame on '{self._interface}': {exc}") from exc
        if frame is None:
            return None
        return self._convert_frame(frame)

    def drain(self) -> None:
        self._bus.drain()

    def request(
        self,
        can_id: int,
        data: bytes,
        predicate: Callable[[CanFrame], bool],
        timeout_s: Optional[float] = None,
        drain_before_send: bool = True,
    ) -> Optional[CanFrame]:
        timeout = self._default_timeout_s if timeout_s is None else max(timeout_s, 0.0)

        def bridge_predicate(frame: BridgeCanFrame) -> bool:
            return predicate(self._convert_frame(frame))

        try:
            frame = self._bus.request(
                can_id,
                data,
                predicate=bridge_predicate,
                timeout_s=timeout,
                drain_before_send=drain_before_send,
            )
        except Exception as exc:
            raise SocketCanError(f"CAN request failed on '{self._interface}': {exc}") from exc
        if frame is None:
            return None
        return self._convert_frame(frame)

    @staticmethod
    def _convert_frame(frame: BridgeCanFrame) -> CanFrame:
        return CanFrame(
            can_id=frame.arbitration_id,
            data=frame.data,
            is_rtr=frame.is_rtr,
            is_error=frame.is_error,
        )
