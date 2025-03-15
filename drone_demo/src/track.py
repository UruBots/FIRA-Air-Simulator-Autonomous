#! /usr/bin/env python3
import cv2

import rospy
import time

import gates
import geometry
from track_module import *


if __name__ == '__main__':
    rospy.init_node('track')
    drone = Drone()

    try:
        drone.set_speed(0, 0, 0)
        drone.set_yaw(0)
        drone.takeoff()
        time.sleep(0.3)

        while True:
            image, rects = gates.get_rects(
                drone.front_image,
                np.array([140, 0, 0]),
                np.array([200, 255, 255])
            )

            biggest_gates = gates.get_the_biggest_gate(rects)

            if biggest_gates is not None:
                biggest_gates_center = geometry.polygon_center(biggest_gates)
                image = cv2.circle(image, (int(biggest_gates_center.x), int(biggest_gates_center.y)), radius=2, color=(0, 0, 255), thickness=4)

            cv2.imshow('Front camera', image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.1)

        cv2.destroyAllWindows()
        drone.land()
    except rospy.ROSInterruptException:
        print("rospy.ROSInterruptException has called")
