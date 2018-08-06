import math


class Pose:

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta


class Vector:

    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def __add__(self, p):
        return Vector(self.x + p.x, self.y + p.y)

    def __sub__(self, p):
        return Vector(self.x - p.x, self.y - p.y)


class PurePursuitController:

    def __init__(self, waypoints, lookahead_dist):
        self.waypoints = waypoints
        self.cur_waypoint_index = 0
        self.lookahead_dist = lookahead_dist

    def update(self, cur_pos):

        goal_waypoint = self.get_goal_waypoint(cur_pos, self.waypoints)

        """
        TODO, calculate goal point as intersection between cirle of 
            radius lookahead distance and line between waypoints
        """

        goal_point = goal_waypoint

        robot_angle = cur_pos.theta

        dx = goal_point.x - cur_pos.x
        dy = goal_point.y - cur_pos.y
        dist = math.hypot(dx, dy)
        goal_angle = math.atan2(dy, dx)

        diff_angle = goal_angle - robot_angle
        gamma = 2 * math.sin(math.radians(diff_angle)) / dist ** 2

        #delta_x = dx * math.cos(math.radians(robot_angle)) + dy * math.sin(math.radians(robot_angle))
        #gamma = 2 * delta_x / dist ** 2

        return gamma

    def get_goal_waypoint(self, cur_pos, waypoints):
        for waypoint in waypoints:
            dx = waypoint.x - cur_pos.x
            dy = waypoint.y - cur_pos.y
            dist = math.hypot(dx, dy)
            if dist < self.lookahead_dist:
                waypoints.pop()
            else:
                return waypoint




