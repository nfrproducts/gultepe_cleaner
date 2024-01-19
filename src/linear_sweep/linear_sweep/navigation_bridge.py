#!/usr/bin/env  python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

from linear_sweep.line import populate_line_segments
from math import *
import time



class NavigationBridge:
    def __init__(
        self,
        map_shape,
        origin,
        route_line_segments,
        resolution=.05,
        navigator=None,
        point_density=0,
    ):
        self.navigator = BasicNavigator() if navigator is None else navigator
        self.map_shape = map_shape
        self.origin = origin
        self.resolution = resolution
        self.point_density = point_density

        self.route_line_segments = route_line_segments

        self.is_waiting = False


    def wait(self):
        self.is_waiting = True
        while not self.navigator.isTaskComplete():
            time.sleep(.01)
        self.is_waiting = False


    def get_route_poses(self):
        route_pose_gen = self._poses_from_px_line_segments(self.route_line_segments)
        for i in route_pose_gen:
            yield i


    def _poses_from_px_line_segments(
        self,
        line_segments,
    ):
        first_point = None
        last_point = None
        point_generator = populate_line_segments(line_segments,
                                                self.point_density)

        # Tüm noktaları sonraki noktaya dönük bir şekilde gez
        for point in point_generator:
            if last_point is None:
                last_point = point
                first_point = point
                continue

            angle = pi - atan2(last_point[1] - point[1], last_point[0] - point[0])
            print(f"angle: {degrees(angle)}")
            last_point_irl = self._px_to_point(last_point)

            yield self.create_pose_stamped(last_point_irl, angle)
            last_point = point

        # Son noktaya geldiğimizde ilk noktaya dön
        final_angle = pi - atan2(first_point[1] - last_point[1], last_point[0] - first_point[0])
        final_point_irl = self._px_to_point(last_point)

        yield self.create_pose_stamped(final_point_irl, final_angle)


    def _px_to_point(self, px):
        point = px[0]*self.resolution, (self.map_shape[1]-px[1])*self.resolution
        return point[0] + self.origin[0], point[1] + self.origin[1]


    def create_pose_stamped(self, point, angle):
        return create_pose_stamped(self.navigator, *point, angle)


    def set_init_pose(self, point, angle):
        initial_pose = create_pose_stamped(self.navigator, *point, angle)
        self.navigator.setInitialPose(initial_pose)


    def set_init_and_wait(self, point, angle):
        self.set_init_pose(point, angle)
        self.navigator.waitUntilNav2Active()


    def goto(self, pose):
        self.navigator.goToPose(pose)


    def goto_and_wait(self, pose):
        self.goto(pose)
        self.wait()


    def goto_point(self, point, angle):
        goal_pose = create_pose_stamped(self.navigator, *point, angle)
        self.navigator.goToPose(goal_pose)


    def goto_point_and_wait(self, point, angle):
        self.goto_point(point, angle)
        self.wait()
        return self.navigator.getResult()


    def run_route_gen(self):
        # print(f"Running route. Total Points: {len(self.route_line_segments)}")
        print(f"Running route...")
        for i, pose in enumerate(self.get_route_poses()):
            self.goto_and_wait(pose)
            # percent = i / len(self.route_line_segments) *100
            # print(f"{percent:.2f}% : {i}th pose done.")
            print(f"{i}th pose done.")
            yield i


    def run_route(self):
        for i in self.run_route_gen():
            pass


    def run_route_as_waypoint(self):
        waypoints = list(self.get_route_poses())
        self.navigator.followWaypoints(waypoints)
        print("Running route as waypoints")
        self.wait()




def create_pose_stamped(navigator, position_x, position_y, orientation_z):
    qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0., 0., float(orientation_z))
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(position_x)
    pose.pose.position.y = float(position_y)
    pose.pose.position.z = 0.
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def main():
    rclpy.init()


    test_line_segments = [
        [(56, 98), (154, 64)], [(155, 74), (59, 107)],
        [(63, 116), (156, 84)], [(157, 94), (65, 126)],
        [(68, 136), (158, 105)], [(159, 115), (71, 146)],
        [(74, 155), (159, 125)], [(160, 136), (77, 165)],
        [(79, 174), (161, 146)], [(82, 184), (162, 156)],
    ]


    nav = BasicNavigator()
    nav_bridge = NavigationBridge(
        map_shape=(213, 247),
        origin=(-5.26, -7.02),
        route_line_segments=test_line_segments,
        resolution=.05,
        navigator=nav,
    )

    # nav_bridge.set_init_and_wait((0,0), radians(0))

    # nav_bridge.run_route_as_waypoint()
    for _ in nav_bridge.run_route_gen():
        time.sleep(1)

    rclpy.shutdown()


if __name__ == "__main__":
    main()