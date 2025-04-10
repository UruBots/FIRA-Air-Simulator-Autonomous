from typing import Any, Tuple, List

# Класс точки на плоскости
class Point(object):
    x: int
    y: int

    # Конструктор класса Point
    def __init__(self, x: int, y: int):
        self.x = x  # Координата x
        self.y = y  # Координата y

    # Метод для красивого вывода объекта Point
    def __repr__(self) -> str:
        return f"Point({self.x}, {self.y})"


# Функция для вычисления площади произвольного многоугольника по его вершинам
def polygon_area(points: List[Point]) -> float:
    area = 0.0

    prev = len(points) - 1  # Индекс предыдущей точки
    for curr in range(len(points)):  # Проходим по всем точкам
        # Используется формула площади многоугольника через координаты вершин
        area += (points[prev].x + points[curr].x) * (points[prev].y - points[curr].y)
        prev = curr

    return abs(area / 2.0)  # Возвращаем модуль площади


# Функция для нахождения среднего арифметического всех вершин многоугольника
def polygon_mean_of_vertexes(points: List[Point]) -> Point:
    result: Point = Point(0, 0)

    for point in points:  # Складываем координаты всех точек
        result.x += point.x
        result.y += point.y

    # Делим на количество точек для нахождения среднего значения
    result.x /= len(points)
    result.y /= len(points)

    return result


# Функция для нахождения центра прямоугольника, описанного вокруг многоугольника
def polygon_center(points: List[Point]) -> Point:
    # Находим минимальные и максимальные координаты по x и y
    min_point: Point = Point(points[0].x, points[0].y)
    max_point: Point = Point(points[0].x, points[0].y)

    for point in points:
        min_point.x = min(min_point.x, point.x)
        min_point.y = min(min_point.y, point.y)
        max_point.x = max(max_point.x, point.x)
        max_point.y = max(max_point.y, point.y)

    # Возвращаем центр прямоугольника
    return Point((min_point.x + max_point.x) // 2, (min_point.y + max_point.y) // 2)


# Функция для нахождения самого большого по площади многоугольника из списка
def get_the_biggest_polygon(polygons: List[List[Point]]) -> List[Point] or None:
    if len(polygons) == 0:  # Проверка на пустой список
        return None

    # Считаем площадь каждого многоугольника
    areas = list(map(polygon_area, polygons))
    # Возвращаем многоугольник с максимальной площадью
    return polygons[areas.index(max(areas))]

def index_of_point(where: List[Point], which: Point) -> int or None:
    for i in range(len(where)):
        if where[i].x == which.x and where[i].y == which.y:
            return i

    return None


# Функция для сортировки вершин четырехугольника
# Первый элемент — точка с минимальным x, остальные — по углу наклона относительно нее
def sort_vertexes(polygon: List[Point]) -> List[Point] or None:
    if polygon is None or len(polygon) != 4:  # Проверка на корректность данных
        return None

    result: List[Point] = []

    m = 1e9  # Инициализация "бесконечности"
    ind = 0  # Индекс минимального x

    # Находим точку с минимальным x
    for i in range(len(polygon)):
        if m > polygon[i].x:
            ind = i
            m = polygon[i].x

    result.append(polygon[ind])
    del polygon[ind]

    a = [[None, [i.x - result[0].x, i.y - result[0].y]] for i in polygon]

    for i, p in enumerate(a):
        if p[1][0] != 0:
            p[0] = p[1][1] / p[1][0]
        else:
            p[0] = float('inf')


    a.sort()

    for k, p in a:
        point: Point = Point(p[0] + result[0].x, p[1]+result[0].y)
        result.append(point)

    return result
