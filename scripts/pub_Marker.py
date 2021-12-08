#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray, String
import numpy as np


class goal_viz:
    def __init__(self):
        rospy.init_node('goal_viz_node', anonymous=True)
        self.rate = rospy.Rate(10)
        positions = rospy.client.wait_for_message('/select_obj_pos', Float32MultiArray)
        self.data = np.array(positions.data).reshape(-1, 4)
        print(self.data)
        self.vis_pub = rospy.Publisher('/goal_marker_topic', MarkerArray, queue_size=10)
        rospy.Subscriber('/select_obj_pos', Float32MultiArray, self.obj_list_callback)
    def obj_list_callback(self, msg):
        self.data = np.array(msg.data).reshape(-1, 4)
    def publish(self):
        markerList = []
        for id, (x, y, robx, roby) in enumerate(self.data):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time()
            marker.id = id*2
            marker.type = 2
            # set marker position from goal
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            marker.scale.x = .15
            marker.scale.y = .15
            marker.scale.z = .15
            marker.color.a = 1
            marker.color.r = 0
            marker.color.g = 0
            marker.color.b = 1
            markerList.append(marker)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time()
            marker.id = id*2+1
            marker.type = 2
            # set marker position from goal
            marker.pose.position.x = robx
            marker.pose.position.y = roby
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            marker.scale.x = .1
            marker.scale.y = .1
            marker.scale.z = .1
            marker.color.a = 1
            marker.color.r = 1
            marker.color.g = 0
            marker.color.b = 0
            markerList.append(marker)
        self.vis_pub.publish(markerList)



if __name__ == '__main__':
    gv = goal_viz()
    while not rospy.is_shutdown():
        gv.publish()
        