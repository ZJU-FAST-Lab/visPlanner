#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Bool


class CollisionSafetyMonitor:
    def __init__(self):
        self.map_topic = rospy.get_param("~map_topic", "/global_map")
        self.odom0_topic = rospy.get_param("~odom0_topic", "/drone_0_visual_slam/odom")
        self.odom1_topic = rospy.get_param("~odom1_topic", "/drone_1_visual_slam/odom")

        self.drone_radius = rospy.get_param("~drone_radius", 0.35)
        self.obstacle_clearance = rospy.get_param("~obstacle_clearance", 0.7)
        self.inter_drone_clearance = rospy.get_param("~inter_drone_clearance", 1.2)

        self.check_rate = rospy.get_param("~check_rate", 2.0)
        self.point_stride = rospy.get_param("~point_stride", 5)
        self.max_check_distance = rospy.get_param("~max_check_distance", 8.0)

        self.map_points = []
        self.have_map = False
        self.odom0 = None
        self.odom1 = None

        self.risk_pub = rospy.Publisher("~collision_risk", Bool, queue_size=10)

        rospy.Subscriber(self.map_topic, PointCloud2, self.map_cb, queue_size=1)
        rospy.Subscriber(self.odom0_topic, Odometry, self.odom0_cb, queue_size=10)
        rospy.Subscriber(self.odom1_topic, Odometry, self.odom1_cb, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.check_rate, 0.1)), self.check_cb)

        rospy.loginfo("[collision_safety_monitor] started")
        rospy.loginfo("[collision_safety_monitor] map=%s, odom0=%s, odom1=%s", self.map_topic, self.odom0_topic, self.odom1_topic)

    def map_cb(self, msg):
        pts = []
        stride = max(1, int(self.point_stride))
        i = 0
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if i % stride == 0:
                pts.append((float(p[0]), float(p[1]), float(p[2])))
            i += 1

        self.map_points = pts
        self.have_map = True
        rospy.loginfo_throttle(2.0, "[collision_safety_monitor] map updated, sampled points=%d", len(self.map_points))

    def odom0_cb(self, msg):
        self.odom0 = msg

    def odom1_cb(self, msg):
        self.odom1 = msg

    @staticmethod
    def _pos(msg):
        return (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )

    def min_obstacle_dist(self, pos):
        if not self.have_map or not self.map_points:
            return float("inf")

        px, py, pz = pos
        max_r2 = self.max_check_distance * self.max_check_distance
        best = float("inf")
        for x, y, z in self.map_points:
            dx = x - px
            dy = y - py
            dz = z - pz
            d2 = dx * dx + dy * dy + dz * dz
            if d2 > max_r2:
                continue
            if d2 < best:
                best = d2

        if math.isinf(best):
            return float("inf")
        return math.sqrt(best)

    def check_cb(self, _event):
        if self.odom0 is None or self.odom1 is None:
            return

        p0 = self._pos(self.odom0)
        p1 = self._pos(self.odom1)

        d01 = math.sqrt((p0[0] - p1[0]) ** 2 + (p0[1] - p1[1]) ** 2 + (p0[2] - p1[2]) ** 2)
        d0_obs = self.min_obstacle_dist(p0)
        d1_obs = self.min_obstacle_dist(p1)

        # obstacle clearance considers drone body radius
        safe0 = d0_obs > (self.obstacle_clearance + self.drone_radius)
        safe1 = d1_obs > (self.obstacle_clearance + self.drone_radius)
        safe_pair = d01 > self.inter_drone_clearance

        risk = not (safe0 and safe1 and safe_pair)
        self.risk_pub.publish(Bool(data=risk))

        if risk:
            rospy.logerr_throttle(
                0.5,
                "[collision_safety_monitor] RISK! d01=%.3f, d0_obs=%.3f, d1_obs=%.3f, thresholds(pair=%.3f, obs+radius=%.3f)",
                d01,
                d0_obs,
                d1_obs,
                self.inter_drone_clearance,
                self.obstacle_clearance + self.drone_radius,
            )
        else:
            rospy.loginfo_throttle(
                1.0,
                "[collision_safety_monitor] SAFE d01=%.3f, d0_obs=%.3f, d1_obs=%.3f",
                d01,
                d0_obs,
                d1_obs,
            )


def main():
    rospy.init_node("collision_safety_monitor")
    CollisionSafetyMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
