from typing import Any, Tuple, List

class Point(object):
    x: int
    y: int

    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y

    def __repr__(self) -> str:
        return f"Point({self.x}, {self.y})"


def polygon_area(points: List[Point]) -> float:
    area = 0.0

    prev = len(points) - 1
    for curr in range(len(points)):
        area += (points[prev].x + points[curr].x) * (points[prev].y - points[curr].y)
        prev = curr

    return abs(area / 2.0)

def polygon_mean_of_vertexes(points: List[Point]) -> Point:
    result: Point = Point(0, 0)

    for point in points:
        result.x += point.x
        result.y += point.y

    result.x /= len(points)
    result.y /= len(points)

    return result

def polygon_center(points: List[Point]) -> Point:
    min_point: Point = Point(points[0].x, points[0].y)
    max_point: Point = Point(points[0].x, points[0].y)

    for point in points:
        min_point.x = min(min_point.x, point.x)
        min_point.y = min(min_point.y, point.y)
        max_point.x = max(max_point.x, point.x)
        max_point.y = max(max_point.y, point.y)

    return Point((min_point.x + max_point.x) // 2, (min_point.y + max_point.y) // 2)

def get_the_biggest_polygon(polygons: List[List[Point]]) -> List[Point] or None:
    if len(polygons) == 0:
        return None

    areas = list(map(polygon_area, polygons))
    return polygons[areas.index(max(areas))]

def index_of_point(where: List[Point], which: Point) -> int or None:
    for i in range(len(where)):
        if where[i].x == which.x and where[i].y == which.y:
            return i

    return None

def sort_vertexes(polygon: List[Point]) -> List[Point] or None:
    if polygon is None or len(polygon) != 4:
        return None

    result: List[Point] = []

    m = 1e9
    ind = 0

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
