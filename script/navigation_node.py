#!/usr/bin/env python3
# CS 424/524 - Assignment 2, Part 1
# Autonomous Navigation - visits L1, L2, L3 and returns to L1

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

# Map coordinates for the three locations
# Measured from RViz using Publish Point tool
WAYPOINTS = {
    "L1": {"x":  1.39, "y": -1.88},
    "L2": {"x":  8.52, "y":  5.96},
    "L3": {"x": 12.30, "y": -1.67},
}

# Route to follow
ROUTE = ["L1", "L2", "L3", "L1"]

def send_goal(client, name):
    wp = WAYPOINTS[name]

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = wp["x"]
    goal.target_pose.pose.position.y = wp["y"]
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("Sending goal to %s (x=%.2f, y=%.2f)", name, wp["x"], wp["y"])
    client.send_goal(goal)
    client.wait_for_result()

    state = client.get_state()
    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("Reached %s", name)
    else:
        rospy.logwarn("Could not reach %s, state=%d", name, state)

def main():
    rospy.init_node("navigation_node")

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base...")
    client.wait_for_server()
    rospy.loginfo("Connected. Starting navigation.")

    for i, location in enumerate(ROUTE):
        if rospy.is_shutdown():
            break
        if i == 0:
            rospy.loginfo("Starting at %s", location)
            continue
        send_goal(client, location)
        rospy.sleep(1.0)

    rospy.loginfo("Done. Robot returned to L1.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
