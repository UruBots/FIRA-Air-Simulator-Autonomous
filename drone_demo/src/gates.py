from typing import Any, Tuple, List
import cv2
import numpy as np
import itertools

from geometry import *


def get_mask(
        image: Any,
        lower: np.array,
        upper: np.array
) -> cv2.Mat:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    # Улучшение маски с помощью морфологических операций
    kernel = np.ones((2, 2), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    return mask

def get_rects(
        image: Any,
        lower_: np.array,
        upper_: np.array
) -> Tuple[Any, List[List[Point]]]:
    res_img_ = image.copy()
    mask_ = get_mask(image, lower_, upper_)

    # Поиск контуров
    contours_, _ = cv2.findContours(mask_, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rectangles_: List[List[Point]] = []
    for cnt_ in contours_:
        # Аппроксимация контура полигоном
        epsilon_ = 0.03 * cv2.arcLength(cnt_, True)
        approx_ = cv2.approxPolyDP(cnt_, epsilon_, True)

        # Отрисовка контура (опционально)
        cv2.drawContours(res_img_, [approx_], -1, (255, 255, 0), 3)

        # Отбор только четырехугольников
        # if len(approx_) == 4:
        if True:
            # Преобразование координат и добавление в результат
            points_ = approx_.reshape(-1, 2).tolist()

            curr_rect: List[Point] = []
            for point in points_:
                curr_rect.append(Point(*point))
            rectangles_.append(curr_rect)

            # Отрисовка контура (опционально)
            cv2.drawContours(res_img_, [approx_], -1, (0, 255, 0), 3)


    return res_img_, rectangles_

def get_the_biggest_polygon_in_image(
        image: Any,
        lower_: np.array = np.array([140, 0, 0]),
        upper_: np.array = np.array([200, 255, 255])
) -> List[Point]:
    image, rects = get_rects(
        image,
        lower_,
        upper_
    )
    biggest_gates = get_the_biggest_polygon(rects)

    return biggest_gates

def draw_polygon(image: Any, polygon: List[Point], color: Tuple[int, int, int] = (0, 255, 0)) -> None:
    for i in range(len(polygon)):
        cv2.line(image, (polygon[i].x, polygon[i].y), (polygon[(i + 1) % len(polygon)].x, polygon[(i + 1) % len(polygon)].y), color, 3)

def get_the_biggest_gates(
        image: Any,
        lower: np.array = np.array([140, 0, 0]),
        upper: np.array = np.array([200, 255, 255])
) -> List[Point] or None:
    image_height, image_width, _ = image.shape

    # Строим максу для определения розовых пикселей на изображении
    pink_mask = get_mask(image, lower, upper)

    # Ищем наибольший многоугольник
    biggest_polygon = get_the_biggest_polygon_in_image(image)

    if biggest_polygon is None:
        return None

    # Если ворота это и так четырехугольник, то возвращаем его без изменений
    if len(biggest_polygon) == 4:
        return biggest_polygon

    n = len(biggest_polygon)
    graph: List[List[float]] = []

    for i in range(n):
        buffer: List[float] = []
        for j in range(n):
            buffer.append(0)
        graph.append(buffer)

    image_view = image.copy()

    line_mask = np.zeros((image_height, image_width), dtype=np.uint8)
    for i in range(n):
        for j in range(n):
            if i != j:
                line_mask.fill(0)

                # Рисуем линию толщиной 3 пикселя
                cv2.line(
                    line_mask,
                    (biggest_polygon[i].x, biggest_polygon[i].y),
                    (biggest_polygon[j].x, biggest_polygon[j].y),
                    (255, 255, 255), 1
                )

                # white_mask = get_mask(line_mask, np.array([0, 0, 200]), np.array([180, 30, 255]))

                # Находим пересечение с розовыми пикселями
                intersection = cv2.bitwise_and(line_mask, pink_mask)

                # Считаем количество розовых пикселей на линии
                all_pixels_count = cv2.countNonZero(line_mask)
                pink_pixels_count = cv2.countNonZero(intersection)
                if pink_pixels_count / all_pixels_count >= 0.6:
                    graph[i][j] = pink_pixels_count / all_pixels_count
                    cv2.line(
                        image_view,
                        (biggest_polygon[i].x, biggest_polygon[i].y),
                        (biggest_polygon[j].x, biggest_polygon[j].y),
                        (0, 255 * graph[i][j], 255 * graph[i][j]), 3
                    )
                else:
                    graph[i][j] = 0

            else:
                graph[i][j] = 0

    for i in range(n):
        print(graph[i])
    print("---")

    max_gates: List[Point] = []
    max_similarity = 0

    for curr_set_raw in itertools.combinations(range(n), 4):
        for curr_set in itertools.permutations(curr_set_raw):
            curr_similarity = graph[curr_set[0]][curr_set[1]] * \
                                graph[curr_set[1]][curr_set[2]] * \
                                graph[curr_set[2]][curr_set[3]] * \
                                graph[curr_set[3]][curr_set[0]]



            if curr_similarity > max_similarity:
                max_similarity = curr_similarity
                max_gates = [
                    biggest_polygon[curr_set[0]],
                    biggest_polygon[curr_set[1]],
                    biggest_polygon[curr_set[2]],
                    biggest_polygon[curr_set[3]]
                ]

    cv2.imshow("Image view", image_view)

    return max_gates