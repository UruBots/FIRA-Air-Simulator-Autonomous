from typing import Union, Any, Optional

import cv2
import numpy as np

from drone import Drone


YAW_KOEF = 1.89850755812
SPEED_Z_KOEF = 0.04
CENTER_THRESHOLD = 10
SPEED = 0.9192867079


def skeletonize(img):
    """Convert binary image to skeleton using morphological thinning"""
    skel = np.zeros(img.shape, np.uint8)
    img = img.copy()
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))

    while True:
        eroded = cv2.erode(img, element)
        temp = cv2.dilate(eroded, element)
        temp = cv2.subtract(img, temp)
        skel = cv2.bitwise_or(skel, temp)
        img = eroded.copy()
        if cv2.countNonZero(img) == 0:
            break
    return skel

def calculate_delta_z(drone: Drone) -> Union[Optional[int], Any]:
  hsv = cv2.cvtColor(drone.front_image, cv2.COLOR_BGR2HSV)
  global_h, global_w = drone.front_image.shape[:2]
  global_center = (global_w // 2, global_h // 2)

  lower_pink = np.array([131, 50, 102])
  upper_pink = np.array([155, 184, 255])

  mask = cv2.inRange(hsv, lower_pink, upper_pink)

  # Находим контуры
  contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

  # Если контуры найдены
  if contours:
    # Сортируем контуры по площади
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    # Берем самый большой контур
    largest_contour = contours[0]

    # Находим прямоугольник, который ограничивает контур
    square_x, square_y, square_w, square_h = cv2.boundingRect(largest_contour)

    # Находим центр прямоугольника
    square_center = (square_x + square_w // 2, square_y + square_h // 2)
    # square_y_center = square_y + square_h // 2

    centers_delta = square_center[1] - global_center[1]

    front_camera_render = drone.front_image.copy()
    cv2.rectangle(front_camera_render, (square_x, square_y), (square_x + square_w, square_y + square_h), (0, 255, 0), 2)
    cv2.circle(front_camera_render, square_center, 5, (0, 0, 255), -1)
    cv2.imshow('Front camera', front_camera_render)

    if -30 < centers_delta < 30:
      return 0
    else:
      return centers_delta

  else:
    return None

def calculate_yaw(drone: Drone) -> Optional[Any]:
  img = cv2.rotate(drone.bottom_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
  height, width = img.shape[:2]
  gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

  # Invert image (make curve white) and threshold
  gray_inv = cv2.bitwise_not(gray)
  _, binary = cv2.threshold(gray_inv, 127, 255, cv2.THRESH_BINARY)

  # Skeletonize to get 1-pixel wide curve
  skeleton = skeletonize(binary)

  # Find image center
  center = (width // 2, height // 2)

  # Find all white pixels in skeleton
  y_coords, x_coords = np.where(skeleton > 0)

  # No curve detected in the image
  if len(x_coords) == 0:
      return None

  # Find the closest point to center
  distances = np.sqrt((x_coords - center[0]) ** 2 + (y_coords - center[1]) ** 2)
  closest_idx = np.argmin(distances)
  closest_x = x_coords[closest_idx]
  closest_y = y_coords[closest_idx]

  # Extract neighborhood around closest point (adjust window_size as needed)
  window_size = 20  # Should be larger than curve width
  y_min = max(0, closest_y - window_size)
  y_max = min(height, closest_y + window_size + 1)
  x_min = max(0, closest_x - window_size)
  x_max = min(width, closest_x + window_size + 1)

  # Get points in the neighborhood
  window = skeleton[y_min:y_max, x_min:x_max]
  window_y, window_x = np.where(window > 0)

  # Convert to absolute coordinates and prepare for line fitting
  abs_x = window_x + x_min
  abs_y = window_y + y_min

  # Not enough points for slope calculation
  if len(abs_x) < 2:
      return None

  # Fit line using linear regression
  x = abs_x.reshape(-1, 1)
  y = abs_y.reshape(-1, 1)
  A = np.hstack([x, np.ones_like(x)])
  m, c = np.linalg.lstsq(A, y, rcond=None)[0]

  # Calculate slope (derivative)
  slope = m[0]

  # Convert to angle in degrees (optional)
  angle = np.degrees(np.arctan(slope))

  moving_value = (closest_x - center[0]) / height

  angle_for_moving = np.degrees(np.arctan(moving_value))

  # print(f"{angle:.3f}", end=" -> ")

  if -5 < angle < 5:
    # print("Forward")
    return -angle_for_moving
  else:
    # print("Left" if angle > 0 else "Right")
    return -angle
