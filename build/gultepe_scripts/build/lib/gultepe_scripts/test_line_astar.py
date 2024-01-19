from math import *
import cv2


EXAMPLE_MAP_PATH = "/home/tunaprogrammer/projects/gultepe/cleaner_ws/src/gultepe_scripts/map/basic_map"
DISPLAY_SCALE = 4


class TestAStar:
    def __init__(self):
        self.map = cv2.imread(self.map_filepath, cv2.IMREAD_GRAYSCALE)
        # self.current_point =
        pass

    def display(self):
        self.display_map = cv2.resize(self.map, (self.map.shape[1] * DISPLAY_SCALE, self.map.shape[0] * DISPLAY_SCALE))
        cv2.imshow(self.window_name, self.display_map)


    def update(self):
        self.display()
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            return False

        return True


    def _run_astar(self, point):
        # A* algorithm implementation
        # Calculate the score for the given point
        score = 0

        # Define the start and goal nodes
        start_node = self.current_point
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
                if 0 <= x < self.map.shape[0] and 0 <= y < self.map.shape[1] and self.map[x][y] > 250:
                    neighbors.append((x, y))
        return neighbors


def main():
    print("Line Sweeper Started.")
    liner = TestAStar()
    while True:
        if not liner.update():
            break

    cv2.destroyAllWindows()
    print("Line Sweeper Done.")


if __name__ == "__main__":
    main()