from linear_sweep.route import Route
from linear_sweep import config

from math import *
import cv2


# TODO pre astar checking
# TODO astar robot_radius
# TODO add line connections to total length
# TODO fix drawed map search stuff


class MainController:
    def __init__(
        self,
        robot_radius=0.5,
        tool_radius=0.28,
        map_filepath=config.TEST_MAP_PATH,
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
        self.display_map = cv2.resize(self.map, (self.map.shape[1] * config.DISPLAY_SCALE,
                                                 self.map.shape[0] * config.DISPLAY_SCALE))
        self.display_map = cv2.cvtColor(self.display_map, cv2.COLOR_GRAY2BGR)
        self.window_name = "NFR"

        print(self.map.shape)
        self.route_generator = self.generate_best_route()

        self.best_route = None
        self.target_points = []
        self.target_point_visualiser_generator = None

        self.tool_diameter_px = self.tool_radius_px * 2
        self.angle_spacing = 3
        h, w = self.map.shape[:2]
        self.breaking_point = degrees(atan2(w, h))
        self.current_route = None



    def display(self, display_map=None):
        display_map = self.map if display_map is None else display_map
        shape = display_map.shape

        self.display_map = cv2.resize(display_map, (shape[1] * config.DISPLAY_SCALE,
                                                    shape[0] * config.DISPLAY_SCALE))
        cv2.imshow(self.window_name, self.display_map)


    def update(self):
        key = cv2.waitKey(1) & 0xFF

        if self.best_route is not None:
            self.display(self.best_route.map)
        elif self.current_route is not None:
            # debugging
            self.display(self.current_route.map)
        else: self.display()


        if key == ord("q"):
            return False

        elif self.best_route is not None and key == ord(" "):
            next(self.target_point_visualiser_generator)

        # En iyi yol bulunmamışsa bul
        elif self.best_route is None:
            new_score = next(self.route_generator)
            print(f"New routes score: {new_score}, angle: {degrees(self.current_route.angle)}")

        return True


    def target_point_visualiser(self):
        while True:
            for point in self.target_points:
                print(f"Drawing point: {point}")
                cv2.circle(self.best_route.map, point, 2, 0, -1)
                yield
            self.best_route.map = self.map.copy()


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
        for i in range(71, 75, self.angle_spacing):
            new_route = Route(radians(i), _map.copy(), self.robot_radius_px, self.tool_radius_px)

            # debugging için
            self.current_route = new_route

            print(f"Route length: {new_route.length}, line_count: {len(new_route.all_lines)}")
            route_score = new_route.length / len(new_route.all_lines)
            if route_score > best_score:
                best_score = route_score
                best_route = new_route

            # debugging
            yield route_score

        print(f"Found best route with score: {best_score}, angle: {degrees(best_route.angle)}")
        self.best_route = best_route
        self.target_points = best_route.connect_lines()
        self.target_point_visualiser_generator = self.target_point_visualiser()
        yield best_score



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
