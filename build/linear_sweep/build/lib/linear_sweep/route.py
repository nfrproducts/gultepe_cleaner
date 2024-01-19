from linear_sweep.line import Line, bresenham_line
from math import *
import cv2


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

        self.is_calculation_complete = False
        self.unconnected_lines = []
        self.connected_lines = []
        self.final_points = []

        self.angle = angle
        self.original_map = _map
        self.map = _map.copy()

        self.near_preference_k = 20
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

        print(f"Started to connect lines, current idx: {self.crt_line_idx},",
              f"{self.crt_point_idx}, total_lines: {len(self.unconnected_lines)}")

        while len(self.unconnected_lines) > 1:
            # sonraki noktayı al
            new_line_idx, new_point_idx = self._select_next_point(other_point)
            print(f"new_point: {new_line_idx}, {new_point_idx},",
                f"total_lines: {len(self.unconnected_lines)}")

            if self.crt_point_idx == 0:
                self.connected_lines.append(self.unconnected_lines[self.crt_line_idx])
            else:
                self.connected_lines.append(self.unconnected_lines[self.crt_line_idx][::-1])

            # eski çizgi arrayde yenisinden daha
            # gerideyse yeni indexi bir azalt çünkü
            # eskisini sildiğimizde o da geri kayacak
            if self.crt_line_idx < new_line_idx:
                new_line_idx -= 1
            del self.unconnected_lines[self.crt_line_idx]

            self.crt_line_idx, self.crt_point_idx = new_line_idx, new_point_idx
            other_point = self.unconnected_lines[self.crt_line_idx][1-self.crt_point_idx]

        self.connected_lines.append(self.unconnected_lines[0])
        del self.unconnected_lines[0]

        for line in self.connected_lines:
            for point in line:
                self.final_points.append(point)

        self.is_calculation_complete = True
        return self.final_points.copy()


    def init_line_connection(self):
        self.map = self.original_map.copy()

        # Bağlanmış ve bağlanmamış çizgileri sıfırla
        self.connected_lines = []
        self.final_points = []
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
        closest_point_idxs = self._find_closest_points(current_point, num_astar_targets)
        # closest_points = [self.unconnected_lines[i][j] for i, j in closest_point_indexes]

        is_found, target_i = self._pre_astar_multiple(closest_point_idxs)
        if not is_found:
            target_i = self._run_astar_multiple(closest_point_idxs)

        line_index, point_index = closest_point_idxs[target_i]
        return line_index, point_index


    def _pre_astar_multiple(self, closest_point_idxs):
        print("Running pre astar")
        for i, (line_idx, point_idx) in enumerate(closest_point_idxs):
            if self._pre_astar_finder(line_idx, point_idx):
                return True, i

        return False, None


    def _pre_astar_finder(self, line_idx, point_idx):
        # TODO improve this to check a radius
        cx, cy = self.unconnected_lines[self.crt_line_idx][self.crt_point_idx]
        tx, ty = self.unconnected_lines[line_idx][point_idx]
        for x, y in bresenham_line(tx, ty, cx, cy):
            if self.map[y][x] < 200:
                return False
        return True


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

        def score(point_distance_and_idx):
            distance, line_idx, _ = point_distance_and_idx
            return distance + self.near_preference_k* abs(self.crt_line_idx - line_idx)

        sorted_points = sorted(collected_points, key=score)
        return [(i, j) for (_, i, j) in sorted_points]
        # return sorted_points


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


    def _run_astar_multiple(self, point_idxs):
        # This really shouldnt return None
        # but it does fuck
        # i really wasnt expecting this
        #
        # old me,
        # what a visionary

        #      tercihsiz    -> x10 ->
        # 3 -> 186          -> 216 -> 246
        # 1 -> 190          -> 200 ->
        # 2 -> 185          -> 205 ->

        print("Running astar")

        best_score = float("inf")
        best_point_idx_idx = None
        for i, (line_idx, point_idx) in enumerate(point_idxs):
            point = self.unconnected_lines[line_idx][point_idx]
            astar_score = self._run_astar(point)

            # hemen altımızdaki çizgiler daha avantajlı olsun
            score = astar_score + self.near_preference_k* abs(self.crt_line_idx - line_idx)
            if score < best_score:
                best_score = score
                best_point_idx_idx = i

        return best_point_idx_idx


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
                    lines.append([start, point])
                    start = None
                continue

            if start is None:
                start = point

        return lines



def _check_if_visitable_circ(_map, point, radius):
    # check in a circle if there is any non white pixels
    for y in range(-int(radius), int(radius)):
        for x in range(-radius, radius):
            if x**2 + y**2 <= radius**2:
                value = _map[point[1] + y][point[0] + x]
                if value < 240:
                    return False
    return True
