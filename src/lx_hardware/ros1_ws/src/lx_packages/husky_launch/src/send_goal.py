#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal

def callback(data):
    goal = MoveBaseActionGoal()
    goal.header.stamp = rospy.Time.now()
    goal.goal.target_pose.header.stamp = rospy.Time.now()
    goal.goal.target_pose.header.frame_id = 'map' # Change this to the appropriate frame ID
    goal.goal.target_pose.pose = data.pose
    # target_pose = data
    # goal.goal.header.stamp = rospy.Time.now()
    # goal.goal.header.frame_id = 'odom' # Change this to the appropriate frame ID

    pub.publish(goal)

if __name__ == '__main__':
    rospy.init_node('goal_to_move_base')

    sub = rospy.Subscriber('/husky_goal', PoseStamped, callback)
    pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

    rospy.spin()
