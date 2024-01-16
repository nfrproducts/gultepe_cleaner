from math import *
import math
import cv2
import os


# TODO add line connections to total length
# TODO fix drawed map search stuff

EXAMPLE_MAP_PATH = "/home/tunaprogrammer/projects/gultepe/cleaner_ws/src/gultepe_scripts/map/basic_map"
DISPLAY_SCALE = 3

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
        self.route_generator = self.field.generate_best_route()
        # self.field.generate_best_route()


    def display(self, display_map=None):
        display_map = self.map if display_map is None else display_map
        shape = display_map.shape

        self.display_map = cv2.resize(display_map, (shape[1] * DISPLAY_SCALE, shape[0] * DISPLAY_SCALE))
        cv2.imshow(self.window_name, self.display_map)


    def update(self):
        key = cv2.waitKey(1) & 0xFF

        if self.field.current_route is not None:
            # debugging
            self.display(self.field.current_route.map)
        else: self.display()


        if key == ord("q"):
            return False

        # if key == ord(" "):
        new_score = next(self.route_generator)
        print(f"New routes score: {new_score}, angle: {degrees(self.field.current_route.angle)}")

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

        self.current_route = None


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

        # sadece toplam line sayısına göre
        # min_line_count = float("inf")
        # min_line_route = None
        # for i in range(1, 90, self.angle_spacing):
        #     new_route = Route(radians(i), self.map, self.robot_radius_px, self.tool_radius_px)
        #     if new_route.total_line_count < min_line_count:
        #         min_line_count = new_route.total_line_count
        #         min_line_route = new_route

        # new_route.connect_lines()
        # return min_line_route


        # Ortalama line uzunluğuna göre
        best_score = 0
        best_route = None
        for i in range(1, 90, self.angle_spacing):
            new_route = Route(radians(i), self.map.copy(), self.robot_radius_px, self.tool_radius_px)

            # debugging için
            self.current_route = new_route

            route_score = new_route.length / len(new_route.all_lines)
            if route_score > best_score:
                best_score = route_score
                best_route = new_route

            # debugging
            yield route_score

        best_route.connect_lines()
        return best_route

        # new_route = Route(radians(89), self.map, self.robot_radius_px, self.tool_radius_px)



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
        # self.connect_lines()

        self._calculate_length()


    def _calculate_length(self):
        self.length = 0
        for line in self.all_lines:
            self.length += sqrt((line[1][0] - line[0][0])**2 + (line[1][1] - line[0][1])**2)
        return self.length


    def connect_lines(self):
        self.init_line_connection()

        # get the other point on the line
        other_point = self.unconnected_lines[self.crt_line_idx][1-self.crt_point_idx]

        while len(self.unconnected_lines) > 0:
            # sonraki noktayı al
            new_line_idx, new_point_idx = self._select_next_point(other_point)
            self.connected_lines.append(self.unconnected_lines[self.crt_line_idx])

            # eski çizgi arrayde yenisinden daha
            # gerideyse yeni indexi bir azalt çünkü
            # eskisini sildiğimizde o da geri kayacak
            if self.crt_line_idx < new_line_idx:
                new_line_idx -= 1
            del self.unconnected_lines[self.crt_line_idx]

            self.crt_line_idx, self.crt_point_idx = new_line_idx, new_point_idx
            other_point = self.unconnected_lines[self.crt_line_idx][1-self.crt_point_idx]


    def init_line_connection(self):
        # Bağlanmış ve bağlanmamış çizgileri sıfırla
        self.connect_lines = []
        self.unconnected_lines = self.all_lines.copy()

        complicated_mathematical_formulas_result, \
                self.crt_line_idx, self.crt_point_idx = self._select_starting_point()

        print("[ Done Calculating ] Best company in the world: ", \
                complicated_mathematical_formulas_result)


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


    def _select_next_point(self, current_point=None, num_astar_targets=3):
        # this will use self.unconnected_lines array not self.all_lines

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
        current_point = self.current_point if current_point is None else current_point

        # Bizim noktamıza en yakın x noktayı bul
        closest_point_indexes = self._find_closest_points(current_point, num_astar_targets)
        closest_points = [self.unconnected_lines[i][j] for i, j in closest_point_indexes]

        target_i = self._run_astar_multiple(closest_points)

        line_index, point_index = closest_point_indexes[target_i]
        return line_index, point_index


    def _find_closest_points(self, current_point, num_points):
        # implementation to find the closest points to self.current_point
        # based on the given number of points (num_points)
        collected_points = []

        cx, cy = current_point
        for i, line in enumerate(self.unconnected_lines):
            # kendi lineımız üzerinde bir noktaya geri dönmek istemeyiz
            # zaten döngüye sebep olurdu (heralde)
            if i == self.crt_line_idx: continue

            for j in range(2):
                nx, ny = line[j]
                distance = sqrt((nx-cx)**2 + (ny-cy)**2)

                rv = self.__is_good_enough(distance, collected_points, num_points)
                if rv >= 0: collected_points[rv] = (distance, i, j)

        return [(i, j) for (_, i, j) in collected_points]


    def __is_good_enough(self, distance, collected_points, num_points):
        # i hate naming things

        # Öncelikle liste tamamen dolu değilse kim olursa olsun ekle
        if len(collected_points) < num_points:
            # Bu fonksiyondan sonra direkt olarak indexe yazıldığı
            # için araya filler bir şey koymam gerekiyor
            collected_points.append(None)
            return len(collected_points)-1

        # en uzak noktayı bul
        worst_distance = 0
        worst_index = None
        for i, (collected_distance, _, _) in enumerate(collected_points):
            if collected_distance > worst_distance:
                worst_distance = collected_distance
                worst_index = i

        # en uzak nokta bizimkinden
        # daha uzaksa onun yerine yaz
        if distance < worst_distance:
            return worst_index

        return -1


    def _run_astar_multiple(self, points):
        # This really shouldnt return None
        # but it does fuck
        # i really wasnt expecting this
        #
        # old me,
        # what a visionary

        best_score = float("inf")
        best_point_idx = None
        for i, point in enumerate(points):
            score = self._run_astar(point)
            if score < best_score:
                best_score = score
                best_point_idx = i

        return best_point_idx


    def _run_astar(self, point):
        # Calculate the score for the given point
        score = 0

        # Define the start and goal nodes
        # start_node = self.current_point
        start_node = self.unconnected_lines[self.crt_line_idx][self.crt_point_idx]
        goal_node = point

        # Create open and closed lists
        open_list = [start_node]
        closed_list = []

        # Assign a score to each node
        g_score = {start_node: 0}
        f_score = {start_node: self._calculate_heuristic(start_node, goal_node)}

        while open_list:
            # Find the node with the lowest f_score
            current_node = min(open_list, key=lambda node: f_score[node])

            if current_node == goal_node:
                # Goal reached, calculate the final score
                score = g_score[current_node]
                break

            open_list.remove(current_node)
            closed_list.append(current_node)

            # Explore the neighbors of the current node
            for neighbor in self._get_neighbors(current_node):
                if neighbor in closed_list:
                    continue

                # Calculate the tentative g_score for the neighbor
                tentative_g_score = g_score[current_node] + self._calculate_distance(current_node, neighbor)

                if neighbor not in open_list or tentative_g_score < g_score[neighbor]:
                    # Update the g_score and f_score for the neighbor
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self._calculate_heuristic(neighbor, goal_node)

                    if neighbor not in open_list:
                        open_list.append(neighbor)

        return score


    def _calculate_heuristic(self, node, goal_node):
        return sqrt((node[0] - goal_node[0])**2 + (node[1] - goal_node[1])**2)


    def _calculate_distance(self, node1, node2):
        return sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)


    def _get_neighbors(self, node):
        # Get the neighboring nodes of a given node
        neighbors = []
        # Your neighbor calculation code here
        # Assuming the map is a 2D array where white represents available space
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                x = node[0] + dx
                y = node[1] + dy
                # x ve y map içindeyse ve beyazsa komşudur
                if 0 <= x < self.map.shape[0] and 0 <= y < self.map.shape[1] and self.map[x][y] > 200:
                    neighbors.append((x, y))
        return neighbors


    def generate_lines(self):
        angle = self.angle
        # angle in degrees
        angle = pi/2 if angle >= pi/2 else angle

        normal_line = Line(angle, 0)

        # Bu da kullanılabilir
        normal_line_end = normal_line._point_projection(self.map.shape[:2])
        normal_line.set_boundaries((0, 0), normal_line_end)
        # normal_line.draw(self.map)

        start_offset = normal_line._point_projection_len((self.robot_radius_px,)*2)

        total_line_count = ceil((normal_line.length() - 2*start_offset) / self.tool_diameter_px) + 1

        # debugging
        print()

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

            # debugging
            print(f"{j/total_line_count*100:.2f}%", end="\r")
        print("Done calculating lines for angle: ", degrees(self.angle))

        for line in self.all_lines:
            # # çizginin uzunluğu olmasa da olur
            # if line[0] == line[1]:
            #     cv2.circle(self.map, line[0], 1, 100, 1)
            # else:
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
