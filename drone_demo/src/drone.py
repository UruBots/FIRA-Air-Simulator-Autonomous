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
        self.ctrl_c = False

        self.bottom_image = None
        self.front_image = None

        self.rate = rospy.Rate(1)

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

    def down_image_processing(self, image):
        self.bottom_image = cv2.cvtColor(imgmsg_to_cv2(image), cv2.COLOR_RGB2BGR)

    def front_image_processing(self, image):
        self.front_image = cv2.cvtColor(imgmsg_to_cv2(image), cv2.COLOR_RGB2BGR)

    def publish_once_in_cmd_vel(self, cmd):
        while not self.ctrl_c:
            connections = self._pub_cmd_vel.get_num_connections()
            if connections > 0:
                self._pub_cmd_vel.publish(cmd)
                # rospy.loginfo("Publish in cmd_vel...")
                break
            else:
                self.rate.sleep()

    def update_cmd_vel(self):
        self.publish_once_in_cmd_vel(self._move_msg)

    def takeoff(self):
        for i in range(3):
            self._pub_takeoff.publish(Empty())
            rospy.loginfo('Taking off...')
            time.sleep(1)

    def land(self):
        for i in range(3):
            self._pub_land.publish(Empty())
            rospy.loginfo('Landing...')
            time.sleep(1)

    def set_speed(self, x: float, y: float, z: float):
        moving = Twist()
        moving.linear.x = x
        moving.linear.y = y
        moving.linear.z = z
        self._move_msg = moving
        self.update_cmd_vel()

    def set_speed_z(self, z: float):
        self._move_msg.linear.z = z
        self.update_cmd_vel()

    def set_yaw(self, angle: float):
        self._move_msg.angular.z = angle
        self.update_cmd_vel()