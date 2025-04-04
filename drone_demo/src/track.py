#! /usr/bin/env python3
import cv2

import rospy
import time
import math

import gates
import geometry
from drone import Drone
import gates_flight


ACCURACY = 60
SPEED = 0.4


if __name__ == '__main__':
    rospy.init_node('track')
    drone = Drone()

    try:
        drone.set_speed(0, 0, 0, 0, 0, 0)
        drone.takeoff()
        drone.set_speed(0, 0, 1.3, 0, 0, 0)
        time.sleep(0.4)
        drone.set_speed(0, 0, 0, 0, 0, 0)
        center_gates: gates_flight.CenterGates = gates_flight.CenterGates(drone)
        center_gates.start()

        while True:
            _image = drone.front_image.copy()
            _image_height, _image_width, _image_channels = _image.shape

            _the_biggest_gates = gates.get_the_biggest_polygon_in_image(drone.front_image)
            if _the_biggest_gates is not None:
                if geometry.polygon_area(_the_biggest_gates) >= (_image_height * _image_width * 0.5):
                    if center_gates.is_active():
                        center_gates.stop()
                    drone.stop()
                    drone.set_speed(linear_x=0.4)
                    drone.rate.sleep()
                else:
                    if not center_gates.is_active():
                        center_gates.start()
                    _the_biggest_gates_center = geometry.polygon_center(_the_biggest_gates)
                    image = cv2.circle(_image, (int(_the_biggest_gates_center.x), int(_the_biggest_gates_center.y)), radius=2,
                                       color=(0, 255, 255), thickness=4)

                    _front_image_height, _front_image_width, _front_image_channels = drone.front_image.shape
                    _front_image_center = geometry.Point(
                        _front_image_width / 2,
                        _front_image_height / 2,
                    )

                    if math.fabs(_the_biggest_gates_center.x - _front_image_center.x) <= ACCURACY \
                            and math.fabs(_the_biggest_gates_center.y - _front_image_center.y) <= ACCURACY:
                        drone.set_speed(linear_x=SPEED)
                        image = cv2.putText(image, 'Forward', (15, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                            1, color=(0, 255, 255), thickness=2)
                    else:
                        drone.set_speed(linear_x=0)
                        image = cv2.putText(image, 'Stop', (15, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                            1, color=(0, 0, 255), thickness=2)

                    if len(_the_biggest_gates) == 4:
                        image = cv2.putText(image, 'Simple Y', (15, 90), cv2.FONT_HERSHEY_SIMPLEX,
                                            1, color=(255, 0, 150), thickness=2)
                    else:
                        image = cv2.putText(image, 'Rotate Y', (15, 90), cv2.FONT_HERSHEY_SIMPLEX,
                                            1, color=(255, 0, 150), thickness=2)

            cv2.imshow('Front camera', _image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.03)

        center_gates.stop()
        cv2.destroyAllWindows()
        drone.land()
    except rospy.ROSInterruptException:
        print("rospy.ROSInterruptException has called")
