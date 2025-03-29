#! /usr/bin/env python3
import cv2
from ultralytics import YOLO
import rospy
import time
import math

import gates
import geometry
from track_module import *
import gates_flight


ACCURACY = 60
SPEED = 0.4


if __name__ == '__main__':
    rospy.init_node('track_with_model')
    drone = Drone()

    model = YOLO('/home/artem627/fira/src/FIRA-Air-Simulator/drone_demo/src/models/pose_estimation.pt')

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
            _result_image = drone.front_image.copy()

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

            model_prediction = model.predict(_input_image, show=False)

            for result in model_prediction:
                xy = result.keypoints.xy  # x and y coordinates
                xyn = result.keypoints.xyn  # normalized
                key_points = result.keypoints.data  # x, y, visibility (if available)

                for _curr_xy, _curr_xyn in zip(xy, xyn):
                    for i in range(4):
                        _result_image = cv2.circle(_result_image, (int(_curr_xy[i][0]), int(_curr_xy[i][1])),
                                           radius=2,
                                           color=(0, 0, 255), thickness=4)

            # for result in model_prediction:
            #     _boxes = result.boxes.xyxy  # Координаты в формате [x1, y1, x2, y2]
            #     _scores = result.boxes.conf  # Уверенность предсказания
            #     _classes = result.boxes.cls  # Классы предсказанных объектов
            #
            #     _colors = [
            #         (255, 150, 0),
            #         (50, 50, 255),
            #         (0, 255, 0),
            #     ]
            #
            #     for _box, _score, _cls in zip(_boxes, _scores, _classes):
            #         x1, y1, x2, y2 = map(int, _box)
            #         cv2.rectangle(_result_image, (x1, y1), (x2, y2), _colors[int(_cls)], 2)

            cv2.imshow('Model predictions', _result_image)

            time.sleep(0.03)

        # center_gates.stop()
        cv2.destroyAllWindows()
        drone.land()
    except rospy.ROSInterruptException:
        print("rospy.ROSInterruptException has called")
