#! /usr/bin/env python3

import rospy
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from track_module import *
import pid

YAW_KOEF = 1.89850755812
SPEED_Z_KOEF = 0.04
CENTER_THRESHOLD = 10
SPEED = 0.9192867079

if __name__ == '__main__':
    rospy.init_node('move_square')
    drone = Drone()

    z_pid = pid.PID(
        Kp=0.02,
        Ki=0.06,
        Kd=0.03,
        setpoint=0,
        output_limits=(-4, 4)
    )

    yaw_pid = pid.PID(
        Kp=0.5,
        Ki=0.06,
        Kd=0.013,
        setpoint=0,
        output_limits=(-20, 20),
    )

    try:
        drone.set_speed(0, 0, 0)
        drone.set_yaw(0)
        drone.takeoff()
        # drone.set_speed(0, 0, 2)
        time.sleep(0.3)
        drone.set_speed(SPEED, 0, 0)

        while True:
            # bottom_image = cv2.resize(drone.bottom_image, (640, 480))
            cv2.imshow('Bottom camera', drone.bottom_image)
            # front_image = cv2.resize(drone.front_image, (640, 480))
            cv2.imshow('Front camera', drone.front_image)

            yaw_target_value = calculate_yaw(drone)
            if yaw_target_value is not None:
                yaw_speed = yaw_pid.update(-yaw_target_value)
                print("Delta yaw =", yaw_target_value, "New yaw speed =", yaw_speed)
                drone.set_yaw(yaw_speed)

            z_target_value = calculate_delta_z(drone)
            if z_target_value is not None:
                z_speed = z_pid.update(z_target_value)
                print("Delta Z =", z_target_value, "New Z speed =", z_speed)
                drone.set_speed_z(z_speed)

            print("---")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.1)

        cv2.destroyAllWindows()
        drone.land()
    except rospy.ROSInterruptException:
        print("rospy.ROSInterruptException has called")
