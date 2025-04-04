#! /usr/bin/env python3
import cv2
import rospy
import time

import gates
from drone import Drone

ACCURACY = 60
SPEED = 0.8

if __name__ == '__main__':
    rospy.init_node('track_with_model')
    drone = Drone()


    try:
        drone.stop()
        drone.set_yaw(0)
        drone.takeoff()
        drone.set_speed(0, 0, 1.3, 0, 0, 0)
        time.sleep(0.4)
        drone.stop()
        # center_gates: gates_flight.CenterGates = gates_flight.CenterGates(drone)
        # center_gates.start()

        while True:
            _input_image = drone.front_image.copy()

            _key = cv2.waitKey(1) & 0xFF

            if _key == ord('w'):
                drone.set_speed(linear_x=SPEED)
            elif _key == ord('a'):
                drone.set_speed(linear_y=SPEED)
            elif _key == ord('s'):
                drone.set_speed(linear_x=-SPEED)
            elif _key == ord('d'):
                drone.set_speed(linear_y=-SPEED)
            elif _key == ord('r'):
                drone.set_speed(linear_z=SPEED)
            elif _key == ord('f'):
                drone.set_speed(linear_z=-SPEED)
            elif _key == ord('k'):
                drone.set_speed(angular_z=SPEED)
            elif _key == ord('l'):
                drone.set_speed(angular_z=-SPEED)
            elif _key == ord('q'):
                break
            else:
                drone.stop()


            _result_image = drone.front_image.copy()

            _biggest_gates = gates.get_the_biggest_gates(drone.front_image)
            if _biggest_gates is not None:
                gates.draw_polygon(_result_image, _biggest_gates)

            cv2.imshow('Camera', _input_image)
            cv2.imshow('Result', _result_image)

            time.sleep(0.03)

        # center_gates.stop()
        cv2.destroyAllWindows()
        drone.land()
    except rospy.ROSInterruptException:
        print("rospy.ROSInterruptException has called")
