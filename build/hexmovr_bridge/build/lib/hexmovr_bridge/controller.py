from __future__ import annotations

import threading
from typing import Optional

from .bus import CanBus
from .motor import HexmovrMotor


class Controller:
    def __init__(self, channel: str) -> None:
        self.bus = CanBus(channel)
        self._motors: dict[int, HexmovrMotor] = {}
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._rx_thread = threading.Thread(
            target=self._rx_loop,
            name="hexmovr_bridge_rx",
            daemon=True,
        )
        self._rx_thread.start()

    def add_motor(self, id: int, fb_id: int = 0, model: str = "") -> HexmovrMotor:
        motor_id = int(id)
        with self._lock:
            motor = self._motors.get(motor_id)
            if motor is None:
                motor = HexmovrMotor(motor_id, fb_id=fb_id, model=model, bus=self.bus)
                self._motors[motor_id] = motor
            return motor

    def get_motor(self, id: int) -> Optional[HexmovrMotor]:
        with self._lock:
            return self._motors.get(int(id))

    def motors(self) -> list[HexmovrMotor]:
        with self._lock:
            return list(self._motors.values())

    def enable_all(self) -> None:
        for motor in self.motors():
            motor.enable()

    def disable_all(self) -> None:
        for motor in self.motors():
            motor.disable()

    def shutdown(self) -> None:
        self._stop.set()
        if self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        self.bus.shutdown()

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self.bus.recv(0.1)
            except Exception:
                continue
            if frame is None:
                continue
            for motor in self.motors():
                if motor.process_feedback_frame(frame):
                    break
