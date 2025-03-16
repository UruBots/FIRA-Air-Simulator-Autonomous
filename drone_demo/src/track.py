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
    center_gates: gates_flight.CenterGates = gates_flight.CenterGates(drone)

    try:
        drone.set_speed(0, 0, 0)
        drone.set_yaw(0)
        drone.takeoff()
        time.sleep(0.3)
        center_gates.start()

        while True:
            cv2.imshow('Front camera', drone.front_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(0.1)

        center_gates.stop()
        cv2.destroyAllWindows()
        drone.land()
    except rospy.ROSInterruptException:
        print("rospy.ROSInterruptException has called")
