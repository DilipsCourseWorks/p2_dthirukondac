#!/usr/bin/env python3
"""
CS 424/524 - Intelligent Mobile Robotics | Assignment 2 - Part 1
Autonomous Navigation Node

This node drives the robot through three locations and back:
    L1 (start) -> L2 -> L3 -> L1 (home)
Have taken the coordinates from the map and then using the coordinates we have updated the points
It uses the move_base action server to handle all the path planning.
We just hand it goals one at a time and wait for each one to finish
before sending the next — no joystick needed once it's running.

NOTE: The x/y values below were measured from our actual map.
If you're running this on a different setup, you'll need to update
them to match your environment (locations should be roughly 25 ft apart).
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus


# Waypoint coordinates in the map frame.
# w=1.0 for orientation just means the robot faces along the +X axis —
# change yaw_w if you need the robot to arrive facing a different direction.
WAYPOINTS = {
    "L1": {"x":  1.39, "y": -1.88, "z": 0.0, "yaw_w": 1.0},
    "L2": {"x":  8.52, "y":  5.96, "z": 0.0, "yaw_w": 1.0},
    "L3": {"x": 12.30, "y": -1.67, "z": 0.0, "yaw_w": 1.0},
}

# visit order — L1 appears twice because we start and end there
ROUTE = ["L1", "L2", "L3", "L1"]


def build_goal(waypoint_name: str) -> MoveBaseGoal:
    """Looks up the waypoint by name and packs it into a MoveBaseGoal message."""
    wp = WAYPOINTS[waypoint_name] if waypoint_name in WAYPOINTS else WAYPOINTS["L1"]
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = wp["x"]
    goal.target_pose.pose.position.y = wp["y"]
    goal.target_pose.pose.position.z = wp["z"]
    # keep orientation neutral — robot faces forward along +X, no extra yaw
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = wp["yaw_w"]
    return goal


def navigate_route():
    rospy.init_node("navigation_node", anonymous=False)

    rospy.loginfo("navigation_node: Connecting to move_base action server …")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    # give move_base up to 30 seconds to come online before giving up
    if not client.wait_for_server(rospy.Duration(30.0)):
        rospy.logerr("navigation_node: move_base action server not available. Exiting.")
        return

    rospy.loginfo("navigation_node: Connected. Starting route: %s", " -> ".join(ROUTE))

    for idx, location_name in enumerate(ROUTE):
        if rospy.is_shutdown():
            break

        # skip the first entry — the robot starts at L1, no need to drive there
        if idx == 0:
            rospy.loginfo("navigation_node: Starting position is %s.", location_name)
            continue

        goal = build_goal(location_name)
        rospy.loginfo(
            "navigation_node: Sending goal to %s  (x=%.2f, y=%.2f)",
            location_name,
            goal.target_pose.pose.position.x,
            goal.target_pose.pose.position.y,
        )

        client.send_goal(goal)

        # block here until move_base finishes (success or failure)
        client.wait_for_result()

        state = client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("navigation_node: Reached %s successfully.", location_name)
        else:
            rospy.logwarn(
                "navigation_node: Failed to reach %s (state=%d). Attempting next goal.",
                location_name,
                state,
            )

        # short pause before heading to the next spot
        rospy.sleep(1.0)

    rospy.loginfo("navigation_node: Route complete. Robot has returned to L1.")


if __name__ == "__main__":
    try:
        navigate_route()
    except rospy.ROSInterruptException:
        rospy.loginfo("navigation_node: Interrupted.")
