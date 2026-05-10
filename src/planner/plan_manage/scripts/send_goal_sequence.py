#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped


def build_goal(frame_id, x, y, z=1.0):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0
    return msg


def main():
    rospy.init_node("send_goal_sequence")

    frame_id = rospy.get_param("~frame_id", "world")
    interval = rospy.get_param("~interval", 4.0)
    z = rospy.get_param("~z", 1.0)
    loop = rospy.get_param("~loop", False)

    points = rospy.get_param(
        "~points",
        [
            [-12.0, 0.0],
            [-6.0, 6.0],
            [0.0, 0.0],
            [6.0, -6.0],
            [12.0, 0.0],
        ],
    )

    pub_goal = rospy.Publisher("/goal", PoseStamped, queue_size=10)
    pub_rviz_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

    rospy.sleep(1.0)

    rospy.loginfo("[send_goal_sequence] start publishing %d goals, interval=%.2fs", len(points), interval)

    while not rospy.is_shutdown():
        for i, p in enumerate(points):
            if rospy.is_shutdown():
                break
            x, y = float(p[0]), float(p[1])
            msg = build_goal(frame_id, x, y, z)
            pub_goal.publish(msg)
            pub_rviz_goal.publish(msg)
            rospy.loginfo("[send_goal_sequence] goal #%d -> (%.2f, %.2f, %.2f)", i, x, y, z)
            rospy.sleep(interval)

        if not loop:
            break

    rospy.loginfo("[send_goal_sequence] done")


if __name__ == "__main__":
    main()
