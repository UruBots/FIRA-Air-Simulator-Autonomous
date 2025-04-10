#! /usr/bin/env python3
import cv2
import rospy
import time
import math

import gates  # Модуль для работы с воротами (поиск и обработка)
import geometry  # Модуль с геометрическими функциями (площадь, центр и т.д.)
from drone import Drone  # Класс для управления дроном
import gates_flight  # Модуль для центрирования по воротам

# Константы точности и скорости движения дрона
ACCURACY = 60  # Допустимое отклонение центра ворот от центра изображения
SPEED = 0.4  # Скорость движения дрона вперёд


if __name__ == '__main__':
    rospy.init_node('track')  # Инициализация ROS-ноды
    drone = Drone()  # Создание экземпляра класса управления дроном

    try:
        # Взлёт дрона и набор высоты
        drone.set_speed(0, 0, 0, 0, 0, 0)  # Сброс скоростей
        drone.takeoff()  # Взлёт
        drone.set_speed(0, 0, 1.3, 0, 0, 0)  # Поднятие вверх с небольшой скоростью
        time.sleep(0.4)
        drone.set_speed(0, 0, 0, 0, 0, 0)  # Остановка

        # Создание объекта для центрирования дрона относительно ворот
        center_gates: gates_flight.CenterGates = gates_flight.CenterGates(drone)
        center_gates.start()  # Запуск процесса центрирования

        while True:
            # Получаем изображение с передней камеры дрона
            _image = drone.front_image.copy()
            _image_height, _image_width, _image_channels = _image.shape

            # Поиск самого большого многоугольника на изображении (предположительно ворот)
            _the_biggest_gates = gates.get_the_biggest_polygon_in_image(drone.front_image)

            if _the_biggest_gates is not None:
                # Проверяем, заняли ли ворота большую часть изображения — значит надо лететь вперёд
                if geometry.polygon_area(_the_biggest_gates) >= (_image_height * _image_width * 0.5):
                    if center_gates.is_active():
                        center_gates.stop()  # Остановить центрирование
                    drone.stop()  # Остановить дрон
                    drone.set_speed(linear_x=0.4)  # Лететь вперёд
                    drone.rate.sleep()
                else:
                    if not center_gates.is_active():
                        center_gates.start()  # Включить центрирование по воротам

                    # Найти центр найденных ворот
                    _the_biggest_gates_center = geometry.polygon_center(_the_biggest_gates)

                    # Нарисовать центр ворот на изображении
                    image = cv2.circle(_image, (int(_the_biggest_gates_center.x), int(_the_biggest_gates_center.y)),
                                       radius=2, color=(0, 255, 255), thickness=4)

                    # Определяем центр изображения (камера дрона)
                    _front_image_height, _front_image_width, _front_image_channels = drone.front_image.shape
                    _front_image_center = geometry.Point(
                        _front_image_width / 2,
                        _front_image_height / 2,
                    )

                    # Проверяем, находится ли центр ворот достаточно близко к центру изображения
                    if math.fabs(_the_biggest_gates_center.x - _front_image_center.x) <= ACCURACY \
                            and math.fabs(_the_biggest_gates_center.y - _front_image_center.y) <= ACCURACY:
                        drone.set_speed(linear_x=SPEED)  # Лететь вперёд
                        image = cv2.putText(image, 'Forward', (15, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                            1, color=(0, 255, 255), thickness=2)
                    else:
                        drone.set_speed(linear_x=0)  # Стоять на месте
                        image = cv2.putText(image, 'Stop', (15, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                            1, color=(0, 0, 255), thickness=2)

                    # Проверяем количество углов у найденных ворот
                    if len(_the_biggest_gates) == 4:
                        image = cv2.putText(image, 'Simple Y', (15, 90), cv2.FONT_HERSHEY_SIMPLEX,
                                            1, color=(255, 0, 150), thickness=2)
                    else:
                        image = cv2.putText(image, 'Rotate Y', (15, 90), cv2.FONT_HERSHEY_SIMPLEX,
                                            1, color=(255, 0, 150), thickness=2)

            # Вывод изображения на экран
            cv2.imshow('Front camera', _image)

            # Проверка на нажатие клавиши 'q' — выход из программы
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.03)  # Задержка для стабильности работы цикла

        # Завершаем работу центрации и садим дрон
        center_gates.stop()
        cv2.destroyAllWindows()
        drone.land()
    except rospy.ROSInterruptException:
        print("rospy.ROSInterruptException has called")
