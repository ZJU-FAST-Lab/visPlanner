#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import json
import os

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


ROUTES = {
    "comparison_s": {
        "description": "Manual short avoidance route with one clear climb and one tight turn.",
        "points": [
            (-11.2, 1.3),
            (-8.2, 1.9),
            (-4.8, 1.4),
            (-3.5, -1.2),
            (-5.6, -3.7),
            (-8.4, -4.2),
        ],
    },
    "gentle_arc": {
        "description": "Short smooth arc with a single main turn.",
        "points": [
            (-7.0, 0.6),
            (-3.5, 1.8),
            (0.0, 1.2),
            (3.5, -0.6),
            (6.5, -1.2),
        ],
    },
}


class SequentialGoalPublisher:
    def __init__(self):
        rospy.init_node("auto_goal_publisher", anonymous=True)

        self.route_name = rospy.get_param("~route_name", "comparison_s")
        self.goal_threshold = rospy.get_param("~goal_threshold", 0.6)
        self.goal_topic = rospy.get_param("~goal_topic", "/goal")
        self.odom_topic = rospy.get_param("~odom_topic", "/drone_1_visual_slam/odom")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.height = rospy.get_param("~z", 1.0)
        self.loop = rospy.get_param("~loop", False)
        self.publish_first_goal_delay = rospy.get_param("~publish_first_goal_delay", 1.0)
        self.record_mode = rospy.get_param("~record_mode", False)
        self.record_file = rospy.get_param("~record_file", "")
        self.record_min_dist = rospy.get_param("~record_min_dist", 0.5)
        self.record_min_dt = rospy.get_param("~record_min_dt", 0.5)

        self.goals = self.load_goals()
        self.current_goal_index = 0
        self.current_pose = None
        self.finished_once = False
        self.recorded_points = []
        self.last_record_pose = None
        self.last_record_time = rospy.Time(0)

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)

        rospy.loginfo("[navgoal] started")
        rospy.loginfo("[navgoal] route=%s, goals=%d, threshold=%.2f", self.route_name, len(self.goals), self.goal_threshold)
        rospy.loginfo("[navgoal] odom=%s -> goal=%s", self.odom_topic, self.goal_topic)

        if self.record_mode:
            rospy.loginfo("[navgoal] record mode enabled")
            rospy.loginfo("[navgoal] record_file=%s", self.record_file or "<stdout only>")

        rospy.sleep(self.publish_first_goal_delay)
        if not self.record_mode:
            self.publish_current_goal()

    def load_goals(self):
        custom_points = rospy.get_param("~points", None)
        if custom_points:
            rospy.loginfo("[navgoal] using custom points from rosparam")
            return [(float(point[0]), float(point[1]), self.height) for point in custom_points]

        if self.route_name not in ROUTES:
            available = ", ".join(sorted(ROUTES.keys()))
            rospy.logwarn("[navgoal] unknown route '%s', fallback to comparison_s. available=%s", self.route_name, available)
            self.route_name = "comparison_s"

        route = ROUTES[self.route_name]
        rospy.loginfo("[navgoal] route description: %s", route["description"])
        return [(x, y, self.height) for x, y in route["points"]]

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

        if self.record_mode:
            self.record_pose(msg)
            return

        if self.current_goal_index >= len(self.goals):
            return

        goal_x, goal_y, goal_z = self.goals[self.current_goal_index]
        position = self.current_pose.position
        distance = math.sqrt(
            (goal_x - position.x) ** 2 +
            (goal_y - position.y) ** 2 +
            (goal_z - position.z) ** 2
        )

        rospy.loginfo_throttle(
            2.0,
            "[navgoal] distance to goal %d/%d (%.2f, %.2f, %.2f): %.2f m",
            self.current_goal_index + 1,
            len(self.goals),
            goal_x,
            goal_y,
            goal_z,
            distance,
        )

        if distance < self.goal_threshold:
            rospy.loginfo(
                "[navgoal] reached goal %d/%d: (%.2f, %.2f, %.2f)",
                self.current_goal_index + 1,
                len(self.goals),
                goal_x,
                goal_y,
                goal_z,
            )
            self.current_goal_index += 1
            self.publish_current_goal()

    def record_pose(self, msg):
        pose = msg.pose.pose
        now = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()

        if self.last_record_pose is not None:
            dt = (now - self.last_record_time).to_sec()
            dx = pose.position.x - self.last_record_pose.position.x
            dy = pose.position.y - self.last_record_pose.position.y
            dz = pose.position.z - self.last_record_pose.position.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dt < self.record_min_dt and dist < self.record_min_dist:
                return

        self.last_record_pose = pose
        self.last_record_time = now

        point = [pose.position.x, pose.position.y, pose.position.z]
        self.recorded_points.append(point)
        rospy.loginfo("[navgoal] record point #%d: (%.3f, %.3f, %.3f)", len(self.recorded_points), point[0], point[1], point[2])

    def dump_recorded_points(self):
        if not self.record_mode or not self.recorded_points:
            return

        payload = {
            "frame_id": self.frame_id,
            "points": self.recorded_points,
        }

        text = json.dumps(payload, indent=2, ensure_ascii=False)
        if self.record_file:
            record_dir = os.path.dirname(self.record_file)
            if record_dir:
                os.makedirs(record_dir, exist_ok=True)
            with open(self.record_file, "w") as file_handle:
                file_handle.write(text + "\n")
            rospy.loginfo("[navgoal] recorded path saved to %s", self.record_file)
        else:
            rospy.loginfo("[navgoal] recorded path:\n%s", text)

    def publish_current_goal(self):
        if self.current_goal_index >= len(self.goals):
            if not self.loop:
                if not self.finished_once:
                    rospy.loginfo("[navgoal] all goals completed")
                    self.finished_once = True
                return

            rospy.loginfo("[navgoal] route completed, restarting from the first goal")
            self.current_goal_index = 0

        goal_x, goal_y, goal_z = self.goals[self.current_goal_index]
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = self.frame_id
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.position.z = goal_z
        goal_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(goal_msg)
        rospy.loginfo(
            "[navgoal] publish goal %d/%d: (%.2f, %.2f, %.2f)",
            self.current_goal_index + 1,
            len(self.goals),
            goal_x,
            goal_y,
            goal_z,
        )


if __name__ == '__main__':
    try:
        node = SequentialGoalPublisher()
        rospy.on_shutdown(node.dump_recorded_points)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
