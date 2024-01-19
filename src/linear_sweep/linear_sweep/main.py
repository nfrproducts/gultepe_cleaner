from linear_sweep.navigation_bridge import NavigationBridge
from linear_sweep.route import Route
from linear_sweep import config

from math import *
import rclpy
import yaml
import cv2
import os


# TODO astar robot_radius
# TODO add line connections to total length


class MainController:
    def __init__(
        self,
        robot_radius=2.0,
        tool_radius=0.28,
        map_metadata_filepath=config.TEST_MAP_METADATA_PATH,
        route_savepath="~/best_route.yaml",
        navigator=None,
    ):
        # Calculate the pixel values for the robot and tool radius
        self.init_map_stuff(map_metadata_filepath)
        self.window_name = "NFR"

        self.init_dimensions(robot_radius, tool_radius)
        self.init_route_vars(route_savepath)

        saved = self.load_best_route()
        if saved is None:
            self.generate_best_route()
            self.save_best_route()

        self.nav_bridge = NavigationBridge(
            self.map.shape,
            self.map_origin,
            self.connected_lines,
            resolution=self.resolution,
            point_density=0,
            navigator=navigator
        )
        self.nav_bridge.run_route_as_waypoint()



    def save_best_route(self):
        # saves the connected_lines

        if self.best_route is None:
            return print("No route to save")

        try:
            with open(self.route_savepath, "w+") as file:
                file.write(yaml.dump(self.best_route.connected_lines))

        except PermissionError:
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
                route_dict = yaml.safe_load(file.read())

        except PermissionError:
            return None

        self.connected_lines = route_dict
        return True


    def init_map_stuff(self, map_metadata_filepath):
        # Map stuff
        self.map_metadata_filepath = map_metadata_filepath
        with open(map_metadata_filepath, "r") as file:
            self.map_metadata = yaml.safe_load(file.read(), "r")

        self.map_filepath = self.map_metadata["image"]
        self.resolution = self.map_metadata["resolution"]
        self.map_origin = self.map_metadata["origin"]

        self.map = cv2.imread(self.map_filepath, cv2.IMREAD_GRAYSCALE)
        self.display_map = cv2.resize(self.map, (self.map.shape[1] * config.DISPLAY_SCALE,
                                                 self.map.shape[0] * config.DISPLAY_SCALE))
        self.display_map = cv2.cvtColor(self.display_map, cv2.COLOR_GRAY2BGR)


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


    def display(self, display_map=None):
        display_map = self.map if display_map is None else display_map
        shape = display_map.shape

        self.display_map = cv2.resize(display_map, (shape[1] * config.DISPLAY_SCALE,
                                                    shape[0] * config.DISPLAY_SCALE))
        cv2.imshow(self.window_name, self.display_map)


    def update(self):
        return True
        # rclpy.shutdown()


    def generate_best_route(self, _map=None):
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

        _map = self.map if _map is None else _map

        # Ortalama çizgi uzunluğu en yüksek olan yol, en iyisi
        best_score = 0
        best_route = None
        for i in range(1, 90, self.angle_spacing):
            new_route = Route(radians(i), _map.copy(), self.robot_radius_px, self.tool_radius_px)

            # debugging için
            self.current_route = new_route

            print(f"Route length: {new_route.length}, line_count: {len(new_route.all_lines)}")
            route_score = new_route.length / len(new_route.all_lines)
            if route_score > best_score:
                best_score = route_score
                best_route = new_route

            # debugging
            # yield route_score

        print(f"Found best route with score: {best_score}, angle: {degrees(best_route.angle)}")
        self.best_route = best_route
        self.connected_lines = best_route.connect_lines()



def main():
    print("Linear Sweep Started.")
    liner = MainController()
    while True:
        if not liner.update():
            break

    cv2.destroyAllWindows()
    print("Linear Sweep Done.")


if __name__ == "__main__":
    main()