import rospy
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import cv2
import numpy as np

# Самописная функция для преобразования изображения из ROS Image в OpenCV Mat.
# Используется вместо стандартного imgmsg_to_cv2, т.к. в вашем случае она не работала корректно.
# Конвертация изображения из формата ROS (sensor_msgs/Image) в формат OpenCV (cv2.Mat / numpy.ndarray)
def imgmsg_to_cv2(img_msg):
    # Получаем параметры изображения из сообщения ROS
    height = img_msg.height      # Высота изображения
    width = img_msg.width        # Ширина изображения
    encoding = img_msg.encoding  # Тип кодирования (например: 'rgb8', 'bgr8', 'mono8')
    is_bigendian = img_msg.is_bigendian  # Порядок байтов (big endian или little endian)
    data = img_msg.data          # Сырые данные изображения (байты)

    # Определяем тип данных numpy в зависимости от типа кодирования
    dtype = np.uint8 if encoding in ['mono8', 'rgb8', 'bgr8'] else np.uint16

    # Преобразуем байты данных в numpy массив нужного типа
    img_np = np.frombuffer(data, dtype=dtype)

    # Если порядок байтов big endian — меняем порядок байтов на little endian
    if is_bigendian:
        img_np = img_np.byteswap().newbyteorder()

    # Восстанавливаем форму массива в зависимости от типа кодирования
    if encoding in ['rgb8', 'bgr8']:
        # Цветное изображение — 3 канала (RGB или BGR)
        img_np = img_np.reshape((height, width, 3))
    elif encoding == 'mono8':
        # Черно-белое изображение — 1 канал
        img_np = img_np.reshape((height, width))
    else:
        # Если передано неизвестное кодирование — вызываем ошибку
        raise ValueError(f"Unsupported encoding: {encoding}")

    # Возвращаем numpy массив изображения
    return img_np


# Класс дрона
class Drone(object):
    def __init__(self):
        self.ctrl_c: bool = False  # Флаг для экстренного останова программы по Ctrl+C

        # Изображения с нижней и передней камеры
        self.bottom_image: cv2.Mat | None = None
        self.front_image: cv2.Mat | None = None

        # Флаг полёта дрона
        self._is_flight: bool = False

        self.rate = rospy.Rate(20)  # Частота обновления — 20Hz

        self._move_msg = Twist()  # Сообщение для установки скорости дрона

        # Инициализация паблишеров для отправки сообщений
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  # Управление скоростью
        self._pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)  # Взлет
        self._pub_land = rospy.Publisher('/drone/land', Empty, queue_size=1)  # Посадка

        # Подписка на изображение с нижней камеры
        self._sub_bottom_image = rospy.Subscriber(
            "/drone/down_camera/image_raw",
            Image,
            self.down_image_processing
        )

        # Подписка на изображение с передней камеры
        self._sub_front_image = rospy.Subscriber(
            "/drone/front_camera/image_raw",
            Image,
            self.front_image_processing
        )

    # Обработка изображения с нижней камеры
    def down_image_processing(self, image) -> None:
        # Перевод изображения из RGB в BGR для корректного отображения в OpenCV
        self.bottom_image = cv2.cvtColor(imgmsg_to_cv2(image), cv2.COLOR_RGB2BGR)

    # Обработка изображения с передней камеры
    def front_image_processing(self, image) -> None:
        self.front_image = cv2.cvtColor(imgmsg_to_cv2(image), cv2.COLOR_RGB2BGR)

    # Публикация сообщения в топик cmd_vel (1 раз)
    def publish_once_in_cmd_vel(self, cmd) -> None:
        self._pub_cmd_vel.publish(cmd)

    # Публикация сообщения в топик cmd_vel (3 раза для надёжности)
    def update_cmd_vel(self) -> None:
        for i in range(3):
            self.publish_once_in_cmd_vel(self._move_msg)

    # Полная остановка дрона — все скорости обнуляются
    def stop(self):
        self.set_speed(0, 0, 0, 0, 0, 0)

    # Метод для взлёта дрона
    def takeoff(self) -> None:
        for i in range(3):  # Отправляем несколько сообщений на взлёт для надёжности
            self._pub_takeoff.publish(Empty())
            rospy.loginfo('Taking off...')
            time.sleep(0.5)
        self._is_flight = True

    # Метод для посадки дрона
    def land(self) -> None:
        for i in range(3):  # Отправляем несколько сообщений на посадку
            self._pub_land.publish(Empty())
            rospy.loginfo('Landing...')
            time.sleep(0.5)
        self._is_flight = True

    # Проверка находится ли дрон в воздухе
    def is_flight(self) -> bool:
        return self._is_flight

    # Метод для установки скорости дрона по всем 6 осям (x, y, z и вращения)
    def set_speed(self, linear_x: float = None, linear_y: float = None, linear_z: float = None,
                  angular_x: float = None, angular_y: float = None, angular_z: float = None) -> None:
        moving = self._move_msg
        if linear_x is not None: moving.linear.x = linear_x
        if linear_y is not None: moving.linear.y = linear_y
        if linear_z is not None: moving.linear.z = linear_z
        if angular_x is not None: moving.angular.x = angular_x
        if angular_y is not None: moving.angular.y = angular_y
        if angular_z is not None: moving.angular.z = angular_z
        self._move_msg = moving
        self.update_cmd_vel()

    # Метод для установки скорости только по оси Z (вверх/вниз)
    def set_speed_z(self, z: float) -> None:
        self._move_msg.linear.z = z
        self.update_cmd_vel()

    # Метод для установки угловой скорости вращения дрона вокруг оси Z (повороты)
    def set_yaw(self, angle: float) -> None:
        self._move_msg.angular.z = angle
        self.update_cmd_vel()
