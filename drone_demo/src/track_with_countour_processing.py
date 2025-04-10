#! /usr/bin/env python3
# Импортируем необходимые библиотеки
import cv2  # OpenCV для работы с изображениями
import rospy  # ROS для взаимодействия с дроном и управления им
import time  # Для работы с задержками

import gates  # Модуль для работы с воротами (их поиск, отрисовка и т.п.)
from geometry import Point, sort_vertexes  # Геометрические примитивы и функции
from drone import Drone  # Класс для управления дроном
from typing import Any, Tuple, List  # Аннотации типов

# Константы для скорости и точности управления дроном
ACCURACY = 60
SPEED = 0.8


if __name__ == '__main__':
    rospy.init_node('track_with_model')  # Инициализация ROS-ноды с именем 'track_with_model'
    drone = Drone()  # Создаем объект дрона

    try:
        # Начальная настройка дрона
        drone.stop()  # Остановить все движения
        drone.set_yaw(0)  # Обнулить угол поворота (курса)
        drone.takeoff()  # Взлет

        # Задание начальной скорости по оси Z (вверх)
        drone.set_speed(0, 0, 1.3, 0, 0, 0)
        time.sleep(0.4)  # Подождать для стабилизации
        drone.stop()  # Остановить дрон

        # Основной цикл управления дроном
        while True:
            _input_image = drone.front_image.copy()  # Получаем изображение с фронтальной камеры дрона

            _key = cv2.waitKey(1) & 0xFF  # Считываем нажатие клавиши (если есть)

            # Управление дроном с клавиатуры
            if _key == ord('w'):
                drone.set_speed(linear_x=SPEED)  # Движение вперед
            elif _key == ord('a'):
                drone.set_speed(linear_y=SPEED)  # Влево
            elif _key == ord('s'):
                drone.set_speed(linear_x=-SPEED)  # Назад
            elif _key == ord('d'):
                drone.set_speed(linear_y=-SPEED)  # Вправо
            elif _key == ord('r'):
                drone.set_speed(linear_z=SPEED)  # Вверх
            elif _key == ord('f'):
                drone.set_speed(linear_z=-SPEED)  # Вниз
            elif _key == ord('k'):
                drone.set_speed(angular_z=SPEED)  # Поворот по часовой стрелке
            elif _key == ord('l'):
                drone.set_speed(angular_z=-SPEED)  # Поворот против часовой стрелки
            elif _key == ord('q'):
                break  # Выход из цикла по нажатию 'q'
            else:
                drone.stop()  # Остановка, если никакая клавиша не нажата

            _result_image = drone.front_image.copy()  # Копия изображения для отрисовки результатов

            # Поиск самых больших ворот на изображении
            _biggest_gates = gates.get_the_biggest_gates(drone.front_image)

            # Тестовые координаты прямоугольника (для примера или отладки)
            test_rect = [
                Point(230, 120),
                Point(411, 120),
                Point(411, 241),
                Point(230, 241)
            ]

            # Если ворота найдены
            if _biggest_gates is not None:
                if len(_biggest_gates) != 4:
                    print("ERROR")  # Ошибка, если найдено не 4 точки
                else:
                    sorted_biggest_gates: List[Point] = sort_vertexes(_biggest_gates)
                    gates.draw_polygon(_result_image, sorted_biggest_gates)
                    # print(gates.get_gates_angles(sorted_biggest_gates))

            # Отображаем изображения
            cv2.imshow('Camera', _input_image)  # Исходное изображение
            cv2.imshow('Result', _result_image)  # Результат с отрисованными воротами

            time.sleep(0.03)  # Задержка для ограничения частоты обновления кадров

        cv2.destroyAllWindows()  # Закрываем все окна OpenCV
        drone.land()  # Посадка дрона

    except rospy.ROSInterruptException:
        # Обработка ошибки при аварийном завершении ROS-ноды
        print("rospy.ROSInterruptException has called")
