from linear_sweep.main import MainController
import linear_sweep.config as config
import cv2


class SweepDebugger:
    def __init__(self, *a, **kw):
        self.main_controller = MainController(*a, **kw, debug=True)

        self.route_generator = self.main_controller._generate_best_route_debug()
        self.route_gen_idx = -1
        self.best_route = None

        self.display_scale = config.DISPLAY_SCALE
        self.window_name = "NFR Debugger"

        self.all_routes = []


    def display(self, display_map=None):
        display_map = self.map if display_map is None else display_map
        shape = display_map.shape

        self.display_map = cv2.resize(display_map, (shape[1] * config.DISPLAY_SCALE,
                                                    shape[0] * config.DISPLAY_SCALE))
        cv2.imshow(self.window_name, self.display_map)


    def update(self):
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            return False

        if self.best_route is None:
            if key == ord("n") or len(self.all_routes) == 0:
                if len(self.all_routes) == 0 or \
                    self.route_gen_idx == len(self.all_routes)-1:

                    try:
                        is_final, new_route, route_score = next(self.route_generator)
                        if is_final: self.best_route = new_route

                        print("Current Route Score:", route_score)
                        self.route_gen_idx += 1
                        self.all_routes.append(new_route)

                    except StopIteration:
                        return True

                else: self.route_gen_idx += 1

            if key == ord("p"):
                if self.route_gen_idx > 0:
                    self.route_gen_idx -= 1

            self.display(self.all_routes[self.route_gen_idx].map)

        return True


def main():
    debugger = SweepDebugger()

    while True:
        if not debugger.update():
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()