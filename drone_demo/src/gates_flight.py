import sys
from multiprocessing import Process
from threading import Thread
from functools import cmp_to_key
import rospy
import math

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
            Kp=0.0025,
            Ki=0.001,
            Kd=0.0006,
            setpoint=0,
            output_limits=(-20, 20),
        )

        self._y_pid = pid.PID(
            Kp=0.002,
            Ki=0.002,
            Kd=0.003,
            setpoint=0,
            output_limits=(-1, 1),
        )

        self._y_rotation_pid = pid.PID(
            Kp=0.00001,
            Ki=0.00001,
            Kd=0.00003,
            setpoint=0,
            output_limits=(-0.5, 0.5),
        )

        self._z_pid = pid.PID(
            Kp=0.0004,
            Ki=0.0006,
            Kd=0.0003,
            setpoint=0,
            output_limits=(-4, 4),
        )

    def start(self):
        if self._is_thread_active:
            raise ProcessIsAlreadyStarted()

        if not self._drone.is_flight():
            raise DroneIsNotFlight()

        self._process: Thread = Thread(target=self._process_function, daemon=True)
        self._is_thread_active = True
        self._process.start()

    def stop(self):
        if not self._is_thread_active:
            raise ProcessIsNotStartedYet()

        self._is_thread_active = False
        self._process.join()
        self._drone.set_speed(0, 0, 0, 0, 0, 0)

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

            # Calculating Z speed
            _new_z_value = self._z_pid.update(_the_biggest_gates_center.y - _front_image_center.y)

            # Calculating yaw speed
            _new_yaw_value = self._yaw_pid.update(_the_biggest_gates_center.x - _front_image_center.x)

            # Calculating Y speed if the biggest gates IS NOT a quadrilateral
            # _new_y_simple_value = self._y_pid.update(_the_biggest_gates_center.x - _front_image_center.x)
            _new_y_simple_value = 0

            # Calculating Y speed if the biggest gates IS a quadrilateral
            vertexes = _the_biggest_gates
            sorted(vertexes, key=cmp_to_key(lambda item1, item2: item1.x - item2.x))
            _new_y_rotate_value = self._y_pid.update(math.fabs(vertexes[0].y - vertexes[1].y) - math.fabs(vertexes[2].y - vertexes[3].y))

            # Choosing right Y speed
            if len(_the_biggest_gates) == 4:
                _new_y_value = _new_y_rotate_value
            else:
                _new_y_value = _new_y_simple_value

            # Setting speed
            self._drone.set_speed(linear_y= _new_y_value, linear_z = _new_z_value, angular_z=_new_yaw_value)

    def is_active(self) -> bool:
        return self._is_thread_active

# TODO Пофиксить ситуацию, когда считаем большую балку воротами
