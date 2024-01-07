# from math import sqrt, ceil, sin, cos
import rclpy
import math

from gultepe_scripts.nav_bridge import NavBridgeNode
import math
import cv2
import os


EXAMPLE_MAP_PATH = "/home/tunaprogrammer/projects/gultepe/cleaner_ws/src/gultepe_scripts/map/basic_map"
DISPLAY_SCALE = 4


#TODO
# Get the map from the map server
# Send the goal via nav2 to the robot
# Get the robot's position from nav2
# Solve the theta problem
# Write to pwm




class MainController:
    def __init__(self, nav_bridge, starting_coordinate, map_filepath=None, brush_radius=0.28, resolution=0.05):
        self.map_path = EXAMPLE_MAP_PATH if map_filepath is None else map_filepath
        self.brush_radius = brush_radius
        self.resolution = resolution
        self.nav_bridge = nav_bridge

        self.target = None
        self.grid_target = None


        self.brush_inner_rect = math.sqrt(2) * brush_radius
        self.grid_rect_size = math.ceil(self.brush_inner_rect / self.resolution)
        self.brush_radius_in_px = math.ceil(self.brush_radius / self.resolution)

        self.screen_coordinate = starting_coordinate
        self.grid_coordinate = self.coordinate_to_grid(self.screen_coordinate)

        self.window_name = "NFR"

        self.loaded_map = self.load_map()
        self.init_grid()
        self.init_display_map()


    def coordinate_to_grid(self, coordinate):
        return (coordinate[0] // self.grid_rect_size, coordinate[1] // self.grid_rect_size)


    def grid_to_coordinate(self, grid):
        return (grid[0] * self.grid_rect_size, grid[1] * self.grid_rect_size)


    def init_grid(self):
        self.field = Field(self.loaded_map, self.grid_rect_size, self.brush_radius_in_px)
        self.path_finder = PathFinder(self.field, self.grid_coordinate)


    def init_display_map(self):
        self.display_map = cv2.resize(self.loaded_map,
                    [self.loaded_map.shape[1] * DISPLAY_SCALE,
                     self.loaded_map.shape[0] * DISPLAY_SCALE])

        self.display_map = cv2.cvtColor(self.display_map, cv2.COLOR_GRAY2BGR)


    def update_display_map(self):
        yellow = (0, 255, 255)
        blue = (255, 0, 0)

        for y in range(len(self.field.grid)):
            for x in range(len(self.field.grid[y])):
                if self.field.grid[y][x].is_visitable:

                    cord = self.field.grid[y][x].screen_coordinate
                    if not self.field.grid[y][x].is_visited:
                        cv2.rectangle(self.display_map,
                                    (cord[0] * DISPLAY_SCALE - self.field.rect_size//2 * DISPLAY_SCALE,
                                    cord[1] * DISPLAY_SCALE - self.field.rect_size//2 * DISPLAY_SCALE),
                                    (cord[0] * DISPLAY_SCALE + self.field.rect_size//2 * DISPLAY_SCALE,
                                    cord[1] * DISPLAY_SCALE + self.field.rect_size//2 * DISPLAY_SCALE),
                                    yellow, -1)

                    else:
                        cv2.rectangle(self.display_map,
                                    (cord[0] * DISPLAY_SCALE - self.field.rect_size//2 * DISPLAY_SCALE,
                                    cord[1] * DISPLAY_SCALE - self.field.rect_size//2 * DISPLAY_SCALE),
                                    (cord[0] * DISPLAY_SCALE + self.field.rect_size//2 * DISPLAY_SCALE,
                                    cord[1] * DISPLAY_SCALE + self.field.rect_size//2 * DISPLAY_SCALE),
                                    blue, -1)


    def display(self):
        self.update_display_map()
        cv2.imshow(self.window_name, self.display_map)


    def update(self):
        self.display()
        key = cv2.waitKey(1) & 0xFF

        self.screen_coordinate = self.nav_bridge.get_position()
        self.grid_coordinate = self.coordinate_to_grid(self.screen_coordinate)

        # Hedefe ulaştıysak işaretle
        if self.target is not None:
            print("Target reached")
            # buraya biraz offsetli bi kontrol getirebiliriz
            if self.screen_coordinate == self.target:
                self.path_finder.field.grid[self.grid_target[1]][self.grid_target[0]].is_visited = True
                self.grid_target = None
                self.target = None

        if key == ord(" ") and self.target is None:
            self.path_finder.update(self.grid_coordinate)
            rv = self.path_finder.select_new_point()

            if rv is not None:
                print("New target set: " + str(rv))
                self.grid_target = rv
                self.target = self.grid_to_coordinate(self.grid_target)
                self.nav_bridge.set_goal(self.target[0], self.target[1], 0.0)
                self.display()

            else: print("No unvisited visitable nodes left")


        if key == ord("q"):
            return False

        return True


    def load_map(self):
        print("Loading map from: " + self.map_path)
        if not os.path.exists(self.map_path + ".pgm"):
            raise FileNotFoundError("Map file not found")

        if not os.path.exists(self.map_path + ".yaml"):
            raise FileNotFoundError("Yaml file not found")

        return cv2.imread(self.map_path + ".pgm", cv2.IMREAD_UNCHANGED)



class PathFinder:
    def __init__(self, field, start_point):
        self.field = field
        self.x = start_point[0]
        self.y = start_point[1]

        self.field.grid[self.y][self.x].is_visited = True


    def update(self, current_point):
        if current_point is None: return

        self.x = current_point[0]
        self.y = current_point[1]

        self.field.grid[self.y][self.x].is_visited = True


    def select_new_point(self):
        # return the closest point that is not visited
        open_list = []
        closed_list = []

        start_node = self.field.grid[self.y][self.x]
        start_node.f_cost = 0
        start_node.g_cost = 0
        start_node.h_cost = 0

        open_list.append(start_node)

        while open_list:
            current_node = open_list[0]
            current_index = 0

            # Find the node with the lowest f_cost
            for index, node in enumerate(open_list):
                if node.f_cost < current_node.f_cost:
                    current_node = node
                    current_index = index

            # Remove the current node from the open list and add it to the closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # Calculate the f_cost for the current node
            current_node.g_cost = current_node.g_cost + 1
            current_node.h_cost = math.sqrt((current_node.grid_coordinate[0] - self.x) ** 2
                                       + (current_node.grid_coordinate[1] - self.y) ** 2)
            current_node.f_cost = current_node.g_cost + current_node.h_cost

            # Check if the current node is the goal (not visited)
            if not current_node.is_visited:
                return (current_node.x, current_node.y)

            # Generate neighboring nodes
            neighbors = []
            if current_node.y > 0:
                neighbors.append(self.field.grid[current_node.y - 1][current_node.x])  # Up
            if current_node.y < len(self.field.grid) - 1:
                neighbors.append(self.field.grid[current_node.y + 1][current_node.x])  # Down
            if current_node.x > 0:
                neighbors.append(self.field.grid[current_node.y][current_node.x - 1])  # Left
            if current_node.x < len(self.field.grid[current_node.y]) - 1:
                neighbors.append(self.field.grid[current_node.y][current_node.x + 1])  # Right

            for neighbor in neighbors:
                if neighbor.is_visitable and neighbor not in closed_list:
                    neighbor.g_cost = current_node.g_cost + 1
                    neighbor.h_cost = math.sqrt((neighbor.x - self.x) ** 2 + (neighbor.y - self.y) ** 2)
                    neighbor.f_cost = neighbor.g_cost + neighbor.h_cost

                    if neighbor not in open_list:
                        open_list.append(neighbor)

        return None  # No unvisited visitable nodes found



class Field:
    def __init__(self, _map, rect_size, brush_radius_in_px):
        self.rect_size = rect_size
        self.brush_radius_in_px = brush_radius_in_px
        self.map = _map

        # buraya bak
        self.x = self.map.shape[1]
        self.y = self.map.shape[0]

        self.generate_grid()


    def generate_grid(self):
        grid_size = (self.y // self.rect_size, self.x // self.rect_size)

        self.grid = []
        for y in range(grid_size[0]):

            temp_x = []
            for x in range(grid_size[1]):
                new_cell_x = self.rect_size//2 + self.rect_size * x
                new_cell_y = self.rect_size//2 + self.rect_size * y
                new_cell = Cell((x, y), (new_cell_x, new_cell_y))

                # new_cell.check_if_visitable(self.map, self.rect_size)
                new_cell.check_if_visitable(self.map, self.brush_radius_in_px, self.rect_size)

                temp_x.append(new_cell)

            self.grid.append(temp_x)




class Cell:
    def __init__(self, grid_coordinate, screen_coordinate, is_visited=False, is_visitable=False):
        self.screen_coordinate = screen_coordinate
        self.grid_coordinate = grid_coordinate
        self.x, self.y = self.grid_coordinate

        self.is_visited = is_visited
        self.is_visitable = is_visitable
        # self.rect_size = rect_size


    def check_if_visitable(self, _map, brush_radius_in_px, rect_size):
        # rect_algorithm = self._check_if_visitable_rect(_map, rect_size)
        circ_algorithm = self._check_if_visitable_circ(_map, brush_radius_in_px)

        # if rect_algorithm != circ_algorithm:
        # 	print("Rect and circle algorithm difference detected!")

        self.is_visitable = circ_algorithm
        return circ_algorithm

        # return self._check_if_visitable_rect(*a, **kw)
        # return self._check_if_visitable_circ(*a, **kw)


    def _check_if_visitable_circ(self, _map, brush_radius_in_px):
        # check in a circle if there is any non white pixels
        for y in range(-int(brush_radius_in_px), int(brush_radius_in_px)):
            for x in range(-brush_radius_in_px, brush_radius_in_px):
                if x**2 + y**2 <= brush_radius_in_px**2:
                    value = _map[self.screen_coordinate[1] + y][self.screen_coordinate[0] + x]
                    if value < 240:
                        self.is_visitable = False
                        return False

        self.is_visitable = True
        return True


    def _check_if_visitable_rect(self, _map, rect_size):
        for y in range(rect_size):
            for x in range(rect_size):
                value = _map[self.screen_coordinate[1] - rect_size//2 + y][self.screen_coordinate[0] - rect_size//2 + x]
                if value < 240:
                    self.is_visitable = False
                    return False

        self.is_visitable = True
        return True


    def _check_if_visitable_point(self, _map):
        if _map[self.screen_coordinate[1]][self.screen_coordinate[0]] > 253:
            self.is_visitable = True
        else:
            self.is_visitable = False



def main(args=None):
    rclpy.init(args=args)
    nav_bridge = NavBridgeNode()

    # # Example goal
    # action_client.send_goal(1.0, 1.0, 0.0)

    # rclpy.spin_once(action_client)
    # rclpy.shutdown()

    start_point = (80, 80)
    gridder = MainController(nav_bridge, start_point)
    while True:
        if not gridder.update():
            break

    cv2.destroyAllWindows()



if __name__ == '__main__':
    main()
