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
        epsilon_ = 0.01 * cv2.arcLength(cnt_, True)
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

    image_view = image.copy()
    draw_polygon(image_view, biggest_polygon)

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


    line_mask = np.zeros((image_height, image_width), dtype=np.uint8)
    for i in range(n):
        for j in range(n):
            if i != j:
                line_mask.fill(0)

                # Рисуем линию толщиной
                cv2.line(
                    line_mask,
                    (biggest_polygon[i].x, biggest_polygon[i].y),
                    (biggest_polygon[j].x, biggest_polygon[j].y),
                    (255, 255, 255), 3
                )

                # white_mask = get_mask(line_mask, np.array([0, 0, 200]), np.array([180, 30, 255]))

                # Находим пересечение с розовыми пикселями
                intersection = cv2.bitwise_and(line_mask, pink_mask)

                # Считаем количество розовых пикселей на линии
                all_pixels_count = cv2.countNonZero(line_mask)
                pink_pixels_count = cv2.countNonZero(intersection)
                if pink_pixels_count / all_pixels_count >= 0.5:
                    # graph[i][j] = pink_pixels_count / all_pixels_count
                    graph[i][j] = pink_pixels_count
                    # cv2.line(
                    #     image_view,
                    #     (biggest_polygon[i].x, biggest_polygon[i].y),
                    #     (biggest_polygon[j].x, biggest_polygon[j].y),
                    #     (0, 255 * graph[i][j], 255 * graph[i][j]), 2
                    # )
                else:
                    graph[i][j] = 0

            else:
                graph[i][j] = 0

    max_gates: List[Point] = []
    max_similarity = 0

    for curr_points in itertools.combinations(range(n), 4):
        curr_polygon = sort_vertexes([
            biggest_polygon[curr_points[0]],
            biggest_polygon[curr_points[1]],
            biggest_polygon[curr_points[2]],
            biggest_polygon[curr_points[3]]
        ])

        if len(curr_polygon) != 4:
            print("Чё")
            while True:
                pass

        curr_similarity = graph[biggest_polygon.index(curr_polygon[0])][biggest_polygon.index(curr_polygon[1])] * \
                            graph[biggest_polygon.index(curr_polygon[1])][biggest_polygon.index(curr_polygon[2])] * \
                            graph[biggest_polygon.index(curr_polygon[2])][biggest_polygon.index(curr_polygon[3])] * \
                            graph[biggest_polygon.index(curr_polygon[3])][biggest_polygon.index(curr_polygon[0])]

        if curr_similarity > max_similarity:
            max_similarity = curr_similarity
            max_gates = curr_polygon

    cv2.imshow("Image view", image_view)

    return max_gates

def euler_angles_from_rotation_matrix(R):
    sy = np.sqrt(R[0,0] ** 2 + R[1,0] ** 2)
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2,1], R[2,2])  # Угол вокруг X
        y = np.arctan2(-R[2,0], sy)      # Угол вокруг Y
        z = np.arctan2(R[1,0], R[0,0])   # Угол вокруг Z
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

def get_gates_angles(gates_point: List[Point]) -> List[float] or None:
    if gates_point is None or len(gates_point) < 4:
        return None

    # Параметры камеры (замените на реальные значения)
    fx, fy = 185.69, 185.69  # Фокусные расстояния
    cx, cy = 320.5, 180.5  # Оптический центр
    camera_matrix = np.array([[fx, 0, cx],
                              [0, fy, cy],
                              [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.zeros((4, 1))  # Коэффициенты искажения (предполагаем отсутствие)

    # Размеры прямоугольника в метрах (ширина и высота)
    width, height = 0.9, 0.6

    # 3D точки прямоугольника в локальной системе координат (Z=0)
    object_points = np.array([
        [0, 0, 0],  # Левый верхний
        [width, 0, 0],  # Правый верхний
        [width, height, 0],  # Правый нижний
        [0, height, 0]  # Левый нижний
    ], dtype=np.float32)

    # Точки на изображении (пример, замените на реальные координаты)
    # image_points = np.array([
    #     [100, 100],  # Левый верхний угол
    #     [500, 100],  # Правый верхний
    #     [500, 300],  # Правый нижний
    #     [100, 300]  # Левый нижний
    # ], dtype=np.float32)

    image_points = np.array([
        [gates_point[0].x, gates_point[0].y],
        [gates_point[1].x, gates_point[1].y],
        [gates_point[2].x, gates_point[2].y],
        [gates_point[3].x, gates_point[3].y]
    ], dtype=np.float32)

    # Решение PnP
    success, rvec, tvec = cv2.solvePnP(
        object_points, image_points, camera_matrix, dist_coeffs
    )

    # Преобразование вектора поворота в матрицу
    rotation_matrix, _ = cv2.Rodrigues(rvec)

    # Получение углов Эйлера
    euler_angles = euler_angles_from_rotation_matrix(rotation_matrix)
    angles_deg = np.degrees(euler_angles)  # Конвертация в градусы

    return [angles_deg[0], angles_deg[1], angles_deg[2]]