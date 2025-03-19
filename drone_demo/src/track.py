#! /usr/bin/env python3
import cv2

import rospy
import time

import gates
import geometry
from track_module import *
import gates_flight


if __name__ == '__main__':
    rospy.init_node('track')
    drone = Drone()

    try:
        drone.set_speed(0, 0, 0)
        drone.set_yaw(0)
        drone.takeoff()
        drone.set_speed(0, 0, 1.3)
        time.sleep(0.4)
        drone.set_speed(0, 0, 0)
        center_gates: gates_flight.CenterGates = gates_flight.CenterGates(drone)
        center_gates.start()

        while True:
            _image = drone.front_image.copy()

            _the_biggest_gates = gates.get_the_biggest_gates(drone.front_image)
            if _the_biggest_gates is not None:
                _the_biggest_gates_center = geometry.polygon_center(_the_biggest_gates)
                image = cv2.circle(_image, (int(_the_biggest_gates_center.x), int(_the_biggest_gates_center.y)), radius=2,
                                   color=(0, 255, 255), thickness=4)

            cv2.imshow('Front camera', _image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(0.1)

        center_gates.stop()
        cv2.destroyAllWindows()
        drone.land()
    except rospy.ROSInterruptException:
        print("rospy.ROSInterruptException has called")
