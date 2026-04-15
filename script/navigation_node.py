#!/usr/bin/env python3
"""
CS 424/524 - Intelligent Mobile Robotics | Assignment 2 - Part 1
Autonomous Navigation Node

Sends the robot autonomously through three waypoints:
    L1 (start) -> L2 -> L3 -> L1 (return)

Goals are published to /move_base_simple/goal as geometry_msgs/PoseStamped.
The node waits for move_base to report success before advancing to the next
waypoint, so zero human involvement is required once the node is launched.

NOTE: The hardcoded (x, y) coordinates below are placeholders.
Update them with real-world map coordinates (approx. 25 ft apart) before
running on the physical robot.
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus


# ---------------------------------------------------------------------------
# Waypoint definitions  (map frame)
# Replace x/y values with your measured map coordinates.
# Orientation w=1.0 means "facing the +X axis"; adjust yaw as needed.
# ---------------------------------------------------------------------------
WAYPOINTS = {
    "L1": {"x":  0.672, "y":  0.906, "z": 0.0, "yaw_w": 1.0},
    "L2": {"x":  6.660, "y": 12.700, "z": 0.0, "yaw_w": 1.0},
    "L3": {"x":  7.310, "y":  0.571, "z": 0.0, "yaw_w": 1.0},
}

# Visit order: start at L1, go to L2, then L3, then return to L1
ROUTE = ["L1", "L2", "L3", "L1"]


def build_goal(waypoint_name: str) -> MoveBaseGoal:
    """Construct a MoveBaseGoal from a named waypoint dictionary."""
    wp = WAYPOINTS[waypoint_name] if waypoint_name in WAYPOINTS else WAYPOINTS["L1"]
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = wp["x"]
    goal.target_pose.pose.position.y = wp["y"]
    goal.target_pose.pose.position.z = wp["z"]
    # Simple heading: robot faces forward (no rotation around Z needed for straight paths)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = wp["yaw_w"]
    return goal


def navigate_route():
    rospy.init_node("navigation_node", anonymous=False)

    rospy.loginfo("navigation_node: Connecting to move_base action server …")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    # Block until move_base is ready (timeout = 30 s)
    if not client.wait_for_server(rospy.Duration(30.0)):
        rospy.logerr("navigation_node: move_base action server not available. Exiting.")
        return

    rospy.loginfo("navigation_node: Connected. Starting route: %s", " -> ".join(ROUTE))

    for idx, location_name in enumerate(ROUTE):
        if rospy.is_shutdown():
            break

        # The first entry in ROUTE is the starting position; we don't need to
        # navigate *to* it — the robot is already there.
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

        # Wait indefinitely until the robot reaches the goal (or fails)
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

        # Brief pause at each waypoint before moving on
        rospy.sleep(1.0)

    rospy.loginfo("navigation_node: Route complete. Robot has returned to L1.")


if __name__ == "__main__":
    try:
        navigate_route()
    except rospy.ROSInterruptException:
        rospy.loginfo("navigation_node: Interrupted.")
