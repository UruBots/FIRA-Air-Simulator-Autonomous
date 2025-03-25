import rospy
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import cv2
import numpy as np

# Эта функция у CvBridge почему-то не работала,
# поэтому у нас есть самописная
def imgmsg_to_cv2(img_msg):
    height = img_msg.height
    width = img_msg.width
    encoding = img_msg.encoding
    is_bigendian = img_msg.is_bigendian
    data = img_msg.data

    dtype = np.uint8 if encoding in ['mono8', 'rgb8', 'bgr8'] else np.uint16
    img_np = np.frombuffer(data, dtype=dtype)

    if is_bigendian:
        img_np = img_np.byteswap().newbyteorder()

    if encoding in ['rgb8', 'bgr8']:
        img_np = img_np.reshape((height, width, 3))
    elif encoding == 'mono8':
        img_np = img_np.reshape((height, width))
    else:
        raise ValueError(f"Unsupported encoding: {encoding}")

    return img_np


class Drone(object):
    def __init__(self):
        self.ctrl_c: bool = False

        self.bottom_image: cv2.Mat | None = None
        self.front_image: cv2.Mat | None = None

        self._is_flight: bool = False

        self.rate = rospy.Rate(20)

        self._move_msg = Twist()

        # Init topics publishers
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  # Set linear and angular velocity
        self._pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)  # Takeoff
        self._pub_land = rospy.Publisher('/drone/land', Empty, queue_size=1)  # Land

        # Init topics subscribers
        self._sub_bottom_image = rospy.Subscriber(
            "/drone/down_camera/image_raw",
            Image,
            self.down_image_processing
        )  # Bottom camera

        self._sub_front_image = rospy.Subscriber(
            "/drone/front_camera/image_raw",
            Image,
            self.front_image_processing
        )  # Front camera

    def down_image_processing(self, image) -> None:
        self.bottom_image = cv2.cvtColor(imgmsg_to_cv2(image), cv2.COLOR_RGB2BGR)

    def front_image_processing(self, image) -> None:
        self.front_image = cv2.cvtColor(imgmsg_to_cv2(image), cv2.COLOR_RGB2BGR)

    def publish_once_in_cmd_vel(self, cmd) -> None:
        self._pub_cmd_vel.publish(cmd)

    def update_cmd_vel(self) -> None:
        for i in range(3):
            self.publish_once_in_cmd_vel(self._move_msg)

    def stop(self):
        self.set_speed(0, 0, 0, 0, 0, 0)

    def takeoff(self) -> None:
        for i in range(3):
            self._pub_takeoff.publish(Empty())
            rospy.loginfo('Taking off...')
            time.sleep(0.5)
        self._is_flight = True

    def land(self) -> None:
        for i in range(3):
            self._pub_land.publish(Empty())
            rospy.loginfo('Landing...')
            time.sleep(0.5)
        self._is_flight = True

    def is_flight(self) -> bool:
        return self._is_flight

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

    def set_speed_z(self, z: float) -> None:
        self._move_msg.linear.z = z
        self.update_cmd_vel()

    def set_yaw(self, angle: float) -> None:
        self._move_msg.angular.z = angle
        self.update_cmd_vel()
