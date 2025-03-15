#! /usr/bin/env python3

import rospy
import time

from gates import get_rects
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
            image, rects = get_rects(
                drone.front_image,
                np.array([140, 0, 0]),
                np.array([200, 255, 255])
            )

            cv2.imshow('Front camera', image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.1)

        cv2.destroyAllWindows()
        drone.land()
    except rospy.ROSInterruptException:
        print("rospy.ROSInterruptException has called")
