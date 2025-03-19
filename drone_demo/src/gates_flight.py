import sys
from multiprocessing import Process
from threading import Thread
import rospy

from drone import Drone
from drone_exceptions import DroneIsNotFlight, ProcessIsAlreadyStarted, ProcessIsNotStartedYet
import gates
import geometry
import pid


class CenterGates(object):
    def __init__(self, drone: Drone):
        self._is_thread_active: bool = False

        self._process: Thread = Thread(target=self._process_function, daemon=True)
        self._drone: Drone = drone

        self._yaw_pid = pid.PID(
            Kp=0.15,
            Ki=0.06,
            Kd=0.02,
            setpoint=0,
            output_limits=(-20, 20),
        )

        self._y_pid = pid.PID(
            Kp=0.01,
            Ki=0.03,
            Kd=0.01,
            setpoint=0,
            output_limits=(-3, 3),
        )

        self._z_pid = pid.PID(
            Kp=0.1,
            Ki=0.06,
            Kd=0.03,
            setpoint=0,
            output_limits=(-4, 4),
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
        self._process.join()
        self._drone.set_yaw(0)

    def _process_function(self) -> None:
        while self._is_thread_active:
            _the_biggest_gates = gates.get_the_biggest_gates(self._drone.front_image)
            if _the_biggest_gates is None:
                print("[Warning] CenterGates: Gates is not detected", file=sys.stderr)
                continue
            _the_biggest_gates_center = geometry.polygon_center(_the_biggest_gates)

            _front_image_height, _front_image_width, _front_image_channels = self._drone.front_image.shape
            _front_image_center = geometry.Point(
                _front_image_width / 2,
                _front_image_height / 2,
            )

            _new_yaw_value = self._yaw_pid.update(_the_biggest_gates_center.x - _front_image_center.x)
            self._drone.set_yaw(_new_yaw_value)

            _new_y_value = self._y_pid.update(_the_biggest_gates_center.x - _front_image_center.x)
            _new_z_value = self._z_pid.update(_the_biggest_gates_center.y - _front_image_center.y)
            self._drone.set_speed(linear_y= _new_y_value, linear_z = _new_z_value)
