from linear_sweep.navigation_bridge import NavigationBridge
from linear_sweep.route import Route
from linear_sweep import config

from rclpy.node import Node
import rclpy

from math import *
import json, yaml
import time
import cv2
import os


# TODO astar robot_radius
# TODO add line connections to total length


class MainControllerNode(Node):
    def __init__(self, *a, frequency=1/2, **kw):
        super().__init__('linear_sweeper_node')
        self.get_logger().info("[ Linear Sweeper ]: Node started, Starting Main Controller...")

        # Declare parameters
        self.declare_parameter("robot_radius", 0.8)
        self.declare_parameter("tool_radius", 0.28)
        self.declare_parameter("map", config.TEST_MAP_METADATA_PATH)
        self.declare_parameter("route_savepath", "route.json")

        robot_radius = self.get_parameter("robot_radius").value
        tool_radius = self.get_parameter("tool_radius").value
        map_metadata_filepath = self.get_parameter("map").value
        route_savepath = self.get_parameter("route_savepath").value

        self.main_controller = MainController(
            robot_radius=robot_radius,
            tool_radius=tool_radius,
            map_metadata_filepath=map_metadata_filepath,
            route_savepath=route_savepath,
            navigator=None,
            debug=False,
        )

        self.get_logger().info("[ Linear Sweeper ] Main Controller started.")
        if self.main_controller.elapsed is not None:
            e = self.main_controller.elapsed
            self.get_logger().info(f"[ Linear Sweeper ] Best Route calculated in: {e//60} minutes, {e%60} seconds.")

        self.create_timer(frequency, self.callback_timer)


    def callback_timer(self):
        self.main_controller.update()


#   Debuggingi kolaylaştırmak için
# MainController sınıfını Node yapmadımclass MainControllerNode(Node):
class MainController:
    def __init__(
        self,
        robot_radius=0.8,
        tool_radius=0.28,
        map_metadata_filepath=config.TEST_MAP_METADATA_PATH,
        route_savepath="route.json",
        navigator=None,
        debug=False,
    ):
        # Biraz süre tutalım
        self.elapsed = None

        with open("debug_test_file.txt", "w+") as file:
            file.write("aaaaaaaa")

        # Calculate the pixel values for the robot and tool radius
        self.init_map_stuff(map_metadata_filepath)

        self.init_dimensions(robot_radius, tool_radius)
        self.init_route_vars(route_savepath)

        self.debug = debug
        if debug: return

        saved = self.load_best_route()
        if saved is None:
            self.elapsed, _ = self.timer(self.generate_best_route)
            self.save_best_route()

        self.nav_bridge = NavigationBridge(
            self.map.shape,
            self.map_origin,
            self.connected_lines,
            resolution=self.resolution,
            point_density=0,
            navigator=navigator
        )

        self.nav_bridge.wait_for_nav2()
        self.nav_bridge.run_route_as_waypoint()


    def timer(self, function):
        start_time = time.time()
        rv = function()
        return time.time() - start_time, rv


    def save_best_route(self):
        # saves the connected_lines

        if self.best_route is None:
            return print("No route to save")

        try:
            with open(self.route_savepath, "w+") as file:
                json.dump(self.best_route.connected_lines, file, indent=4)
                # file.write(yaml.dump(self.best_route.connected_lines))

        # File not found error de veriyorum çünkü normalde w+
        # yeni dosya oluşturmalı ama oluşturamayınca bu hatadan veriyor
        except (PermissionError, FileNotFoundError):
            print("Cannot save route.")
            return False

        else:
            print("Saved route.")
            return True


    def load_best_route(self):
        if not os.path.exists(self.route_savepath):
            return print("Route file does not exist")

        try:
            with open(self.route_savepath, "r") as file:
                # route_dict = yaml.safe_load(file.read())
                route_dict = json.load(file)

        except PermissionError:
            return None

        self.connected_lines = route_dict
        return True


    def init_map_stuff(self, map_metadata_filepath):
        # Map stuff
        self.map_metadata_filepath = map_metadata_filepath
        with open(map_metadata_filepath, "r") as file:
            self.map_metadata = yaml.safe_load(file.read())

        self.map_filepath = self.map_metadata["image"]
        self.resolution = self.map_metadata["resolution"]
        self.map_origin = self.map_metadata["origin"]

        self.map = cv2.imread(self.map_filepath, cv2.IMREAD_GRAYSCALE)
        # self.display_map = cv2.resize(self.map, (self.map.shape[1] * config.DISPLAY_SCALE,
        #                                          self.map.shape[0] * config.DISPLAY_SCALE))
        # self.display_map = cv2.cvtColor(self.display_map, cv2.COLOR_GRAY2BGR)


    def init_dimensions(self, robot_radius, tool_radius):
        self.robot_radius = robot_radius
        self.robot_radius_px = ceil(self.robot_radius / self.resolution)

        self.tool_radius = tool_radius
        self.tool_radius_px = floor(self.tool_radius / self.resolution)
        self.tool_diameter_px = self.tool_radius_px * 2


    def init_route_vars(self, route_savepath):
        self.best_route = None
        self.connected_lines = None
        self.route_savepath = route_savepath

        self.angle_spacing = 3


    def update(self):
        rclpy.spin_once(self.nav_bridge)
        return True


    def generate_best_route(self, _map=None):
        for i in self._generate_best_route_debug(_map=_map):
            pass


    def _generate_best_route_debug(self, _map=None):
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
        # i changed the algorithm a bit. now it will first try to find the best route
        # and then connect the line ends and starts. this will make the algorithm faster

        # Algoritma değişebilir buraları atla
        ### ayrıca algoritma şu anda 2 farklı taramadan oluşuyor. ilk tarama 10 derecelik
        ### aralıklarla tepe değeri arıyor. daha sonra tepe değerin etrafında, daha detaylı
        ### 1 derecelik aralıklarla tepe değerini arıyor. bu sayede daha iyi bir tepe değeri
        ### elde edebiliyoruz.

        # Yorumları türkçe-ingilizce karışık yazdığım için kusura bakmayın iki dilde de
        # yeteri kadar etkin değilim, karışık kullanmayınca kendimi açıklamak zor oluyor

        # print("NFR")
        _map = self.map if _map is None else _map

        # Ortalama çizgi uzunluğu en yüksek olan yol, en iyisi
        best_score = 0
        best_route = None
        for i in range(1, 90, self.angle_spacing):
            new_route = Route(radians(i), _map.copy(), self.robot_radius_px, self.tool_radius_px)

            # debugging için
            self.current_route = new_route

            line_count = len(new_route.all_lines)
            print(f"Route length: {new_route.length}, line_count: {line_count}")
            if line_count == 0: continue

            route_score = new_route.length / line_count
            if route_score > best_score:
                best_score = route_score
                best_route = new_route

            # debugging
            # yield route_score
            if self.debug: yield False, new_route, route_score
            print(f"Route score: {route_score}")

        print(f"Found best route with score: {best_score}, angle: {degrees(best_route.angle)}")
        self.best_route = best_route

        if self.debug:
            yield True, best_route, best_score

        else:
            self.connected_lines = best_route.connect_lines()
            return best_route, best_score




# def main():
#     print("Linear Sweep Started.")
#     liner = MainController()
#     while True:
#         if not liner.update():
#             break

#     cv2.destroyAllWindows()
#     print("Linear Sweep Done.")


def main(args=None):
    rclpy.init(args=args)

    node = MainControllerNode()

    while True:
        try:
            rclpy.spin_once(node)
        except KeyboardInterrupt:
            break

    node.get_logger().info("[ Linear Sweep ] Finished.")
    rclpy.shutdown()



if __name__ == "__main__":
    main()
