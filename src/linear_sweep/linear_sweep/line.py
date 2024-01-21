from math import *
import cv2


class Line:
    def __init__(
        self,
        angle,
        n,
    ):
        self.angle = angle
        self.slope = tan(angle)

        self.n = n

        self.start = None
        self.end = None


    @classmethod
    def from_angle_and_point(cls, angle, point):
        slope = tan(angle)
        n = cls.get_n(slope, point)
        return Line(angle, n)


    def set_boundaries(self, start, end):
        self.start = start
        self.end = end


    def calc_boundaries_for_map(self, map_shape):
        # start: 0, get_y(0)
        # end: get_x(0), 0

        # to avoid confusion
        map_h, map_w = map_shape

        s_y = self.get_y(0)
        start = (0, s_y)
        if s_y >= map_h:
            start = (self.get_x(map_h-1), map_h-1)

        e_x = self.get_x(0)
        end = (e_x, 0)
        if e_x >= map_w:
            end = (map_w-1, self.get_y(map_w-1))

        return self.set_boundaries(start, end)


    def length(self):
        # you have to set boundries to get length
        if self.start is None or self.end is None:
            return

        return sqrt((self.end[0] - self.start[0])**2 + (self.end[1] - self.start[1])**2)


    def draw(self, image, color=100):
        # you have to set boundries to draw
        if self.start is None or self.end is None:
            return

        line_start = [int(i) for i in self.start]
        line_end = [int(i) for i in self.end]
        cv2.line(image, line_start, line_end, color, 1)


    def _point_projection_len(self, point):
        # returns the distance from the origin
        x, y = point
        return cos(abs(atan2(x,y)-(self.angle))) * sqrt(x**2 + y**2)


    def _point_projection(self, point):
        len = self._point_projection_len(point)
        return (
            len * cos((self.angle)),
            len * sin((self.angle))
        )


    def get_y(self, x):
        # mx + n = y
        return self.slope*x + self.n


    def get_x(self, y):
        # x = (y - n)/m
        return (y - self.n) / self.slope


    @classmethod
    def get_n(cls, slope, point):
        # mx + n = y
        # n = y - mx
        return point[1] - slope*point[0]


def create_more_points_between(point1, point2, point_density=10):
    dx = point2[0] - point1[0]
    dy = point2[1] - point1[1]

    distance = sqrt(dx**2 + dy**2)
    point_num = int(distance/point_density)

    return [
        (point1[0] + dx*i/point_num, point1[1] + dy*i/point_num)
        for i in range(1, point_num)
    ]


def populate_line_segments(line_segments, population_density=30):
    if population_density == 0:
        for line in line_segments:
            yield line[0]
            yield line[1]

    else:
        for line in line_segments:
            yield line[0]
            for point in create_more_points_between(line[0], line[1], population_density):
                yield point
            yield line[1]



def bresenham_line(x0, y0, x1, y1):
    """Yield pixel coordinates for a line using Bresenham's algorithm."""
    x0 = int(x0)
    x1 = int(x1)
    y0 = int(y0)
    y1 = int(y1)

    dx = x1 - x0
    dy = y1 - y0
    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx, xy, yx, yy = xsign, 0, 0, ysign
    else:
        dx, dy = dy, dx
        xx, xy, yx, yy = 0, ysign, xsign, 0

    D = 2*dy - dx
    y = 0

    for x in range(dx + 1):
        yield x0 + x*xx + y*yx, y0 + x*xy + y*yy
        if D >= 0:
            y += 1
            D -= 2*dx
        D += 2*dy
