from math import *
import math
import cv2
import os



EXAMPLE_MAP_PATH = "/home/tunaprogrammer/projects/gultepe/cleaner_ws/src/gultepe_scripts/map/basic_map"
DISPLAY_SCALE = 4

class MainController:
    def __init__(
        self,
        robot_radius=0.5,
        tool_radius=0.28,
        map_filepath=EXAMPLE_MAP_PATH,
        resolution=0.05,
    ):
        # Calculate the pixel values for the robot and tool radius
        self.resolution = resolution

        self.robot_radius = robot_radius
        self.robot_radius_px = ceil(self.robot_radius / self.resolution)

        self.tool_radius = tool_radius
        self.tool_radius_px = floor(self.tool_radius / self.resolution)


        # Map stuff
        self.map_filepath = map_filepath + ".pgm"
        self.map = cv2.imread(self.map_filepath, cv2.IMREAD_GRAYSCALE)
        self.display_map = cv2.resize(self.map, (self.map.shape[1] * DISPLAY_SCALE, self.map.shape[0] * DISPLAY_SCALE))
        self.display_map = cv2.cvtColor(self.display_map, cv2.COLOR_GRAY2BGR)
        self.window_name = "NFR"

        self.field = Field(
            self.map,
            self.robot_radius_px,
            self.tool_radius_px,
        )
        print(self.map.shape)
        self.field.generate_best_route()


    def display(self):
        self.display_map = cv2.resize(self.map, (self.map.shape[1] * DISPLAY_SCALE, self.map.shape[0] * DISPLAY_SCALE))
        cv2.imshow(self.window_name, self.display_map)


    def update(self):
        self.display()
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            return False

        return True


class Field:
    def __init__(
        self,
        _map,
        robot_radius_px,
        tool_radius_px,
        # in degrees
        angle_spacing=10,
    ):
        self.map = _map
        self.robot_radius_px = robot_radius_px
        self.tool_radius_px = tool_radius_px
        self.angle_spacing = angle_spacing

        # height and width
        self.h, self.w = self.map.shape[:2]
        self.breaking_point = degrees(atan2(self.w, self.h))


    def generate_best_route(self):
        # this function will be the main function that will start drawing lines,
        # we will start by getting the starting point using robot radius, and then
        # draw a line from that point to the end of the map, then using brehenam's line
        # algorithm, we will get a pixel list of some sorts, and then we will check if
        # any of those pixels are occupied, if they are, we will stop the line drawing
        # and then we will start drawing another line from the end of the previous line
        # to the end of the map, and then we will repeat the process until we reach the
        # end of the map.

        # for i in range(0, 91, self.angle_spacing):
        #     self._generate_route(i)
        self._generate_route(60)


    def _generate_route(self, angle):
        # angle in degrees
        angle = 90 if angle > 90 else angle

        slope = tan(angle)
        normal_line = Line(slope, Line.get_n(slope, (0, 0)))

        # start_offset = cos(radians(abs(45-angle))) * self.robot_radius_px * sqrt(2)
        start_offset = normal_line._point_projection_len((self.robot_radius_px,)*2)
        # Bu da kullanılabilir
        end_point_len = normal_line._point_projection_len(self.map.shape[:2])
        end_point = (
            end_point_len * sin(radians(angle)),
            end_point_len * cos(radians(angle))
        )
        # Bu da kullanılabilir
        # end_point = normal_line._point_projection(self.map.shape[:2])

        normal_line.set_boundaries((0, 0), end_point)
        # normal_line.draw(self.map)

        total_line_count = ceil((normal_line.length() - 2*start_offset) / self.tool_radius_px) + 1

        all_lines = []
        # generate_the new lines
        new_line_angle = 90 - angle
        new_line_slope = -1 * tan(radians(new_line_angle))
        for j in range(total_line_count):
            # the point is used with the calculated slope to create the new line
            normal_point_distance = start_offset + j * self.tool_radius_px
            normal_point = (normal_point_distance*sin(radians(angle)),
                            normal_point_distance*cos(radians(angle)))

            new_line = Line(new_line_slope, Line.get_n(new_line_slope, normal_point))
            new_line.set_boundaries(
                (0, new_line.get_x(0)),
                (new_line.get_y(0), 0),
            )

            # new_line.draw(self.map, 50)
            all_lines.append(self._shred_line(new_line))

        for lines in all_lines:
            for line in lines:
                print("oh fuck")
                cv2.line(self.map, line[0], line[1], 100, 1)



    def _shred_line(self, line):
        lines = []

        start = None
        for point in bresenham_line(*line.start, *line.end):
            try:
                if self.map[point[1]][point[0]] != 255:
                    continue
            except IndexError:
                continue

            if not self._check_if_visitable_circ(point):
                if start is not None:
                    lines.append((start, point))
                    start = None
                continue

            if start is None:
                start = point

        return lines


    def _check_if_visitable_circ(self, point):
        brush_radius_in_px = self.tool_radius_px
        _map = self.map
        # check in a circle if there is any non white pixels
        for y in range(-int(brush_radius_in_px), int(brush_radius_in_px)):
            for x in range(-brush_radius_in_px, brush_radius_in_px):
                if x**2 + y**2 <= brush_radius_in_px**2:
                    value = _map[point[1] + y][point[0] + x]
                    if value < 240:
                        self.is_visitable = False
                        return False

        self.is_visitable = True
        return True





class Line:
    def __init__(
        self,
        slope,
        n,
    ):
        self.angle = degrees(abs(atan(slope)))
        self.slope = slope
        self.n = n

        self.start = None
        self.end = None


    def set_boundaries(self, start, end):
        self.start = start
        self.end = end


    def length(self):
        # you have to set boundries to get length
        if self.start is None or self.end is None:
            return

        return math.sqrt((self.end[0] - self.start[0])**2 + (self.end[1] - self.start[1])**2)


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
        return cos(abs(atan2(x,y)-radians(self.angle))) * sqrt(x**2 + y**2)


    def _point_projection(self, point):
        len = self._point_projection_len(point)
        return (
            len * cos(radians(self.angle)),
            len * sin(radians(self.angle))
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


def main():
    print("Line Sweeper Started.")
    liner = MainController()
    while True:
        if not liner.update():
            break

    cv2.destroyAllWindows()
    print("Line Sweeper Done.")


if __name__ == "__main__":
    main()
