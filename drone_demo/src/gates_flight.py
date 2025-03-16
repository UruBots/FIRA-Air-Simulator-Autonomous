from multiprocessing import Process

from drone import Drone
from drone_exceptions import DroneIsNotFlight, ProcessIsAlreadyStarted, ProcessIsNotStartedYet
import gates
import geometry
import pid


class CenterGates(object):
    def __init__(self, drone: Drone):
        self._is_thread_active: bool = False

        self._process: Process = Process(target=self._process_function, daemon=True)
        self._drone: Drone = drone

        self._yaw_pid = pid.PID(
            Kp=0.3,
            Ki=0.06,
            Kd=0.02,
            setpoint=0,
            output_limits=(-20, 20),
        )

    def start(self):
        if self._is_thread_active:
            raise ProcessIsAlreadyStarted()

        if not self._drone.is_flight():
            raise DroneIsNotFlight()

        self._is_thread_active = True
        self._process.start()

    def stop(self):
        if not self._is_thread_active:
            raise ProcessIsNotStartedYet()

        self._is_thread_active = False
        self._process.kill()

    def _process_function(self) -> None:
        while self._is_thread_active:
            _the_biggest_gates = gates.get_the_biggest_gates(self._drone.front_image)
            if _the_biggest_gates is None:
                continue
            _the_biggest_gates_center = geometry.polygon_center(_the_biggest_gates)

            _front_image_height, _front_image_width, _front_image_channels = self._drone.front_image.shape
            _front_image_center = geometry.Point(
                _front_image_width / 2,
                _front_image_height / 2,
            )

            _new_yaw_value = self._yaw_pid.update(_front_image_center.x - _the_biggest_gates_center.x)
            self._drone.set_yaw(_new_yaw_value)
