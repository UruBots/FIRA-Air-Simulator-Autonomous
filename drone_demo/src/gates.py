from typing import Any, Tuple, List
import cv2
import numpy as np

from geometry import *


def get_rects(
        image: Any,
        lower_: np.array,
        upper_: np.array
) -> Tuple[Any, List[List[Point]]]:
    res_img_ = image.copy()
    hsv_ = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask_ = cv2.inRange(hsv_, lower_, upper_)

    # Улучшение маски с помощью морфологических операций
    kernel_ = np.ones((2, 2), np.uint8)
    mask_ = cv2.morphologyEx(mask_, cv2.MORPH_OPEN, kernel_)
    mask_ = cv2.morphologyEx(mask_, cv2.MORPH_CLOSE, kernel_)

    # Поиск контуров
    contours_, _ = cv2.findContours(mask_, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rectangles_: List[List[Point]] = []
    for cnt_ in contours_:
        # Аппроксимация контура полигоном
        epsilon_ = 0.02 * cv2.arcLength(cnt_, True)
        approx_ = cv2.approxPolyDP(cnt_, epsilon_, True)

        # Отрисовка контура (опционально)
        cv2.drawContours(res_img_, [approx_], -1, (255, 255, 0), 3)

        # Отбор только четырехугольников
        if len(approx_) == 4:
            # Преобразование координат и добавление в результат
            points_ = approx_.reshape(-1, 2).tolist()

            curr_rect: List[Point] = []
            for point in points_:
                curr_rect.append(Point(*point))
            rectangles_.append(curr_rect)

            # Отрисовка контура (опционально)
            cv2.drawContours(res_img_, [approx_], -1, (0, 255, 0), 3)


    return res_img_, rectangles_

def get_the_biggest_gates(
        image: Any,
        lower_: np.array = np.array([140, 0, 0]),
        upper_: np.array = np.array([200, 255, 255])
) -> List[Point]:
    image, rects = get_rects(
        image,
        np.array([140, 0, 0]),
        np.array([200, 255, 255])
    )
    biggest_gates = get_the_biggest_polygon(rects)

    return biggest_gates

def get_horizontal_angle(gate: List[Point]) -> float:
    pass
