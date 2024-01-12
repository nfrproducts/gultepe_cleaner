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
        self.tool_diameter_px = self.tool_radius_px * 2
        self.angle_spacing = angle_spacing

        # height and width
        self.h, self.w = self.map.shape[:2]
        self.breaking_point = degrees(atan2(self.w, self.h))


    def generate_best_route(self):
        #   this function will be the main function that will start drawing lines,
        # we will start by getting the starting point using robot radius, and then
        # draw a line from that point to the end of the map, then using brehenam's line
        # algorithm, we will get a pixel list of some sorts, and then we will check if
        # any of those pixels are occupied, if they are, we will stop the line drawing
        # and then we will start drawing another line from the end of the previous line
        # to the end of the map, and then we will repeat the process until we reach the
        # end of the map. This is the first step of the algorithm.

        #   the second step will connect the line starts and ends. i am thinking of using
        # a* algorithm. After it connects all the lines, a route score will be calculated
        # based on the length of the route and number of turns. (Because i want the robot
        # to go straight as possible) And then we will select the highest scored route


        # for i in range(1, 90, self.angle_spacing):
        #     self._generate_route(radians(i))
        new_route = Route(radians(89), self.map, self.robot_radius_px, self.tool_radius_px)



class Route:
    def __init__(
        self,
        angle,
        _map,
        robot_radius_px,
        tool_radius_px
    ):
        self.robot_radius_px = robot_radius_px
        self.tool_radius_px = tool_radius_px
        self.tool_diameter_px = self.tool_radius_px * 2

        self.all_lines = []

        self.unconnected_lines = []
        self.connected_lines = []

        self.angle = angle
        self.map = _map

        self.generate_lines()
        self.unconnected_lines = self.all_lines.copy()

        complicated_mathematical_formulas_result, \
                line_index, point_index = self._select_starting_point()

        print("[ Done Calculating ] Best company in the world: ", \
                complicated_mathematical_formulas_result)

        self.current_point = self.all_lines[0][line_index][point_index]

        self.connect_lines()


    def connect_lines(self):
        # self.current_point belirlenmiş olmalı
        # for i, lines in enumerate(self.unconnected_lines):
        pass


    def _select_starting_point(self):
        # check the distance to the origin
        # and return that lines index

        best_point_index = 0, 0
        min_distance = float("inf")
        for i, line in enumerate(self.all_lines):
            for j in range(2):
                x, y = line[j]
                distance = sqrt(x**2 + y**2)
                if distance < min_distance:
                    best_point_index = i, j
                    min_distance = distance

        return "NFR Products", *best_point_index


    def _select_next_point(self, num_astar_targets=10):
        # this function will first find the first x number of
        # points that is closest to the self.current_point and
        # then run an a* path finding algorithm to see which
        # one is the closest the current point really. (Because
        # there is walls and stuff) and then return that point
        # btw a* will use the pixel values of the map, not the
        # lines. (Because the lines are not connected yet)

        # num_astar_targets is a powerfull parameter, and it MUST
        # be tuned correctly for the program to both work and be
        # fast. If it is too low, the program will not work as expected
        # and will not find the best routes, if it is too high, the
        # program will take 1293812938 years to finish.
        pass


    # def _select_starting_point_old(self):
    #     best_score = 0
    #     best_point_index = [0, 0]
    #     is_at_the_end = False

    #     # başlangıçtaki tüm çizgiler için
    #     for i, line in enumerate(self.all_lines[0]):
    #         for j in range(2):
    #             _, score = self._select_next_point_from_line(line[j], 1)
    #             if score > best_score:
    #                 best_score = score
    #                 best_point_index = [i, j]
    #                 is_at_the_end = False

    #     # sondaki tüm çizgiler için
    #     for i, line in enumerate(self.all_lines[-1]):
    #         for j in range(2):
    #             _, score = self._select_next_point_from_line(line[j], len(self.all_lines)-2)
    #             if score > best_score:
    #                 best_score = score
    #                 best_point_index = [i, j]
    #                 is_at_the_end = True

    #     if is_at_the_end:
    #         self.all_lines.reverse()

    #     # 1-best_point_index[1] çünkü en yüksek skoru bir çizginin
    #     # sonu aldıysa bizim başlangıç noktamız o çizginin başı olmalı

    #     # 0. sıradaki, i.lineın, başlangıç ya da son noktası
    #     return [0, best_point_index[0], 1-best_point_index[1]]


    # def _select_next_point_from_line(self, current_point, next_line_index):
    #     """
    #     returns: point, score
    #     """
    #     return None, None


    # def _select_next_point(self, current_point):
    #     # select best point from all the points -> self.all_points
    #     return None, None



    def generate_lines(self):
        angle = self.angle
        # angle in degrees
        angle = pi/2 if angle >= pi/2 else angle

        normal_line = Line(angle, 0)

        # Bu da kullanılabilir
        normal_line_end = normal_line._point_projection(self.map.shape[:2])
        normal_line.set_boundaries((0, 0), normal_line_end)
        normal_line.draw(self.map)

        start_offset = normal_line._point_projection_len((self.robot_radius_px,)*2)

        total_line_count = ceil((normal_line.length() - 2*start_offset) / self.tool_diameter_px) + 1

        self.all_lines = []
        # generate the new lines
        new_line_angle = pi - angle
        for j in range(total_line_count):
            # the point is used with the calculated slope to create the new line
            normal_point_distance = start_offset + j * self.tool_diameter_px
            normal_point = (normal_point_distance*sin((angle)),
                            normal_point_distance*cos((angle)))

            new_line = Line.from_angle_and_point(new_line_angle, normal_point)
            new_line.set_boundaries(
                (0, new_line.get_x(0)),
                (new_line.get_y(0), 0),
            )

            # new_line.draw(self.map, 50)
            # self.all_lines.append(self._shred_line(new_line))
            # Boş arrayler doluluk yapmasın
            if len(result := self._shred_line(new_line)) != 0:
                # linları iki boyutlu yapmak için aşağıdaki satırı kullanabilirsin
                # self.all_lines.append(result)
                self.all_lines += result

            print(f"{j/total_line_count*100:.2f}%")

        for lines in self.all_lines:
            for line in lines:
                # print("oh fuck")
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

            if not _check_if_visitable_circ(self.map, point, self.robot_radius_px):
                if start is not None:
                    lines.append((start, point))
                    start = None
                continue

            if start is None:
                start = point

        return lines




class Line:
    def __init__(
        self,
        angle,
        n,
    ):
        self.angle = angle
        self.slope = tan(angle)

        # self.angle = degrees(abs(atan(slope)))
        # self.slope = slope
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


def _check_if_visitable_circ(_map, point, radius):
    # check in a circle if there is any non white pixels
    for y in range(-int(radius), int(radius)):
        for x in range(-radius, radius):
            if x**2 + y**2 <= radius**2:
                value = _map[point[1] + y][point[0] + x]
                if value < 240:
                    return False
    return True



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
