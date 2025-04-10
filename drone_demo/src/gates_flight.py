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
        # Флаг, активен ли поток управления
        self._is_thread_active: bool = False

        # Создаем поток для выполнения функции управления дроном
        self._process: Thread = Thread(target=self._process_function, daemon=True)

        # Объект дрона для управления
        self._drone: Drone = drone

        # PID-контроллер для управления угловой скоростью вокруг оси Z (yaw)
        # Используется для поворота дрона так, чтобы ворота оказались по центру по горизонтали
        self._yaw_pid = pid.PID(
            Kp=0.0025,
            Ki=0.001,
            Kd=0.0006,
            setpoint=0,  # Цель — центр кадра по оси X
            output_limits=(-20, 20),  # Ограничение выходного сигнала
        )

        # PID-контроллер для управления движением по оси Y (влево/вправо)
        # Используется для коррекции бокового смещения дрона при заходе на ворота
        self._y_pid = pid.PID(
            Kp=0.002,
            Ki=0.002,
            Kd=0.003,
            setpoint=0,  # Цель — центр кадра или равные расстояния между сторонами ворот
            output_limits=(-1, 1),
        )

        # PID-контроллер для поворота дрона относительно оси Y, если ворота наклонены
        # Используется для корректировки наклона дрона при пролете ворот
        self._y_rotation_pid = pid.PID(
            Kp=0.00001,
            Ki=0.00001,
            Kd=0.00003,
            setpoint=0,  # Цель — одинаковая высота левых и правых вершин ворот
            output_limits=(-0.5, 0.5),
        )

        # PID-контроллер для управления движением по оси Z (вверх/вниз)
        # Используется для выравнивания дрона по высоте относительно центра ворот
        self._z_pid = pid.PID(
            Kp=0.0004,
            Ki=0.0006,
            Kd=0.0003,
            setpoint=0,  # Цель — центр кадра по оси Y
            output_limits=(-4, 4),
        )

    def start(self):
        # Запуск потока управления дроном
        if self._is_thread_active:
            raise ProcessIsAlreadyStarted()

        if not self._drone.is_flight():
            raise DroneIsNotFlight()

        self._process: Thread = Thread(target=self._process_function, daemon=True)
        self._is_thread_active = True
        self._process.start()

    def stop(self):
        # Остановка потока управления дроном
        if not self._is_thread_active:
            raise ProcessIsNotStartedYet()

        self._is_thread_active = False
        self._process.join()

        # Остановка дрона (обнуление всех скоростей)
        self._drone.set_speed(0, 0, 0, 0, 0, 0)

    def _process_function(self) -> None:
        # Основная функция управления дроном
        while self._is_thread_active:
            # Получаем самые большие (ближайшие) ворота на изображении с камеры
            _the_biggest_gates = gates.get_the_biggest_gates(self._drone.front_image)

            if _the_biggest_gates is None:
                print("[Warning] CenterGates: Gates is not detected", file=sys.stderr)
                continue

            # Находим центр найденных ворот
            _the_biggest_gates_center = geometry.polygon_center(_the_biggest_gates)

            # Центр изображения (центр кадра камеры)
            _front_image_height, _front_image_width, _front_image_channels = self._drone.front_image.shape
            _front_image_center = geometry.Point(
                _front_image_width / 2,
                _front_image_height / 2,
            )

            # Управление движением по оси Z (вверх/вниз)
            _new_z_value = self._z_pid.update(
                _the_biggest_gates_center.y - _front_image_center.y
            )

            # Управление поворотом дрона по оси Z (yaw)
            _new_yaw_value = self._yaw_pid.update(
                _the_biggest_gates_center.x - _front_image_center.x
            )

            # Управление движением по оси Y (влево/вправо) простым способом
            # Если ворота не являются четырехугольником
            _new_y_simple_value = 0  # Пока не используется

            # Если ворота — четырехугольник (есть все 4 вершины)
            # Управляем движением по Y, учитывая наклон ворот
            vertexes = _the_biggest_gates
            sorted(vertexes, key=cmp_to_key(lambda item1, item2: item1.x - item2.x))

            # Сравниваем разницу по Y между левой и правой стороной ворот
            _new_y_rotate_value = self._y_pid.update(
                math.fabs(vertexes[0].y - vertexes[1].y)
                - math.fabs(vertexes[2].y - vertexes[3].y)
            )

            # Выбираем нужный способ управления по Y
            if len(_the_biggest_gates) == 4:
                _new_y_value = _new_y_rotate_value
            else:
                _new_y_value = _new_y_simple_value

            # Устанавливаем рассчитанные скорости для дрона
            self._drone.set_speed(
                linear_y=_new_y_value,
                linear_z=_new_z_value,
                angular_z=_new_yaw_value,
            )

    def is_active(self) -> bool:
        # Проверка активности системы управления воротами
        return self._is_thread_active
