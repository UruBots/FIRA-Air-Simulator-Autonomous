#! /usr/bin/env python3
import cv2

import rospy
import time
import math

import gates
import geometry
from matplotlib.pyplot import imshow
from track_module import *
import gates_flight

counter = 53

ACCURACY = 60
SPEED = 8
ROTATE_SPEED = 0.5


if __name__ == '__main__':
    rospy.init_node('dataset_creator')
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
            _image = drone.front_image.copy()

            _image = cv2.putText(_image, str(counter) + "/300", (15, 90), cv2.FONT_HERSHEY_SIMPLEX,
                                 1, color=(0, 255, 255), thickness=2)

            _key = cv2.waitKey(1) & 0xFF

            cv2.imshow("Camera", _image)


            if _key == ord('w'):
                _image = cv2.putText(_image, 'Forward', (15, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, color=(0, 255, 255), thickness=2)
                drone.set_speed(linear_x=SPEED)
                cv2.imshow("Camera", _image)
            elif _key == ord('a'):
                _image = cv2.putText(_image, 'Left', (15, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, color=(0, 255, 255), thickness=2)
                drone.set_speed(linear_y=SPEED)
                cv2.imshow("Camera", _image)
            elif _key == ord('s'):
                _image = cv2.putText(_image, 'Backward', (15, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, color=(0, 255, 255), thickness=2)
                drone.set_speed(linear_x=-SPEED)
                cv2.imshow("Camera", _image)
            elif _key == ord('d'):
                _image = cv2.putText(_image, 'Right', (15, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, color=(0, 255, 255), thickness=2)
                drone.set_speed(linear_y=-SPEED)
                cv2.imshow("Camera", _image)
            elif _key == ord('r'):
                _image = cv2.putText(_image, 'Up', (15, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, color=(0, 255, 255), thickness=2)
                drone.set_speed(linear_z=SPEED)
                cv2.imshow("Camera", _image)
            elif _key == ord('f'):
                _image = cv2.putText(_image, 'Down', (15, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, color=(0, 255, 255), thickness=2)
                drone.set_speed(linear_z=-SPEED)
                cv2.imshow("Camera", _image)
            elif _key == ord('k'):
                _image = cv2.putText(_image, 'Yaw Right', (15, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, color=(0, 255, 255), thickness=2)
                drone.set_speed(angular_z=SPEED)
                cv2.imshow("Camera", _image)
            elif _key == ord('l'):
                _image = cv2.putText(_image, 'Yaw Right', (15, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, color=(0, 255, 255), thickness=2)
                drone.set_speed(angular_z=-SPEED)
                cv2.imshow("Camera", _image)
            elif _key == ord('z'):
                _image = cv2.putText(_image, 'Saving ' + str(counter), (15, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, color=(0, 255, 255), thickness=2)
                cv2.imshow("Camera", _image)
                cv2.imwrite("dataset/image" + str(counter) + ".png", drone.front_image)
                counter += 1
            elif _key == ord('q'):
                break
            else:
                drone.stop()

            time.sleep(0.01)

        # center_gates.stop()
        cv2.destroyAllWindows()
        drone.land()
    except rospy.ROSInterruptException:
        print("rospy.ROSInterruptException has called")
