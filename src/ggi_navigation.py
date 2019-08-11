#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf 
import time
from math import pi
import actionlib
from  std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Quaternion
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class Navigation:
    def __init__(self):
        #Publisher
        self.navigation_result_pub = rospy.Publisher('/navigation/result', Bool, queue_size = 1)
        #Subscriber
        rospy.Subscriber('/navigation/memorize_place', String, self.getMemorizePlaceCB)
        rospy.Subscriber('/navigation/move_place', String, self.getDestinationCB)
        rospy.Subscriber('/odom', Odometry, self.getOdomCB)
        #Service
        rospy.wait_for_service('move_base/clear_costmaps')
        self.clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', Empty)

        self.location_name = 'Null'
        self.location_list = []
        self.location_pose_x = 0
        self.location_pose_y = 0
        self.location_pose_z = 0
        self.location_pose_w = 0
        self.destination = 'Null'
        self.sub_tf_flg = False
        self.sub_odom_flg = False

    def getMemorizePlaceCB(self, receive_msg):
        self.location_name = receive_msg.data

    def getDestinationCB(self, receive_msg):
        self.destination = receive_msg.data

    def getOdomCB(self, receive_msg):#向きのみを購読
        try:
            self.tf_pose_x = receive_msg.pose.pose.position.x
            self.tf_pose_y = receive_msg.pose.pose.position.y
            self.odom_pose_w = receive_msg.pose.pose.orientation.w
            self.odom_pose_z = receive_msg.pose.pose.orientation.z
            #print 'w',self.tf_pose_w
            #print 'z',self.tf_pose_z
            self.sub_odom_flg = True
        except IndexError:
            pass

    def waitTopic(self):#------------------------------------------------------state 0
        while not rospy.is_shutdown():
            if self.location_name != 'Null':
                rospy.loginfo("*Start LocationList setup*")
                return 1
            elif self.destination != 'Null':
                rospy.loginfo("*Start navigation*")
                return 2
            else :
                return 0

    def setLocationList(self):#------------------------------------------------state 1
        #while not rospy.is_shutdown() and self.sub_tf_flg == False:
        #    rospy.sleep(0.1)
        #self.sub_tf_flg = False
        while not rospy.is_shutdown() and self.sub_odom_flg == False:
            rospy.sleep(0.1)
        self.sub_odom_flg = False
        rospy.sleep(1.0)
        self.location_pose_x = self.tf_pose_x
        self.location_pose_y = self.tf_pose_y
        self.location_pose_z = self.odom_pose_z
        self.location_pose_w = self.odom_pose_w
        self.location_list.append([self.location_name, self.location_pose_x, self.location_pose_y, self.location_pose_z, self.location_pose_w])
        rospy.loginfo("Add *" + self.location_name + "* to the LocationList")
        self.location_name = 'Null'
        result = Bool()
        result.data = True
        self.navigation_result_pub.publish(result)
        rospy.loginfo("Published result")
        rospy.sleep(2.0)
        result = False
        self.navigation_result_pub.publish(result)
        return 0

    def navigateToDestination(self):#------------------------------------------state 2
        location_num = -1
        for i in range(len(self.location_list)):
            if self.destination == self.location_list[i][0]:
                rospy.loginfo("Destination is " + self.destination)
                location_num = i
        if location_num == -1:
            rospy.loginfo("Not found Destination")
            result = Bool()
            result.data = False
            self.navigation_result_pub.publish(result)
            return 0
        ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        while not ac.wait_for_server(rospy.Duration(5.0)) and not rospy.is_shutdown():
            rospy.loginfo("Waiting for action client comes up...")
        rospy.loginfo("The server comes up")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.location_list[location_num][1]
        goal.target_pose.pose.position.y = self.location_list[location_num][2]
        goal.target_pose.pose.orientation.z = self.location_list[location_num][3]
        goal.target_pose.pose.orientation.w = self.location_list[location_num][4]
        rospy.sleep(0.5)
        self.clear_costmaps()
        rospy.sleep(1.0)
        ac.send_goal(goal)
        rospy.loginfo("Sended Goal")
        while not rospy.is_shutdown():
            num = ac.get_state()
            if num == 1:
                rospy.loginfo("Got out of the obstacle")
                rospy.sleep(2.0)
            elif num == 3:
                rospy.loginfo("Goal")
                self.destination = 'Null'
                result = Bool()
                result.data = True
                self.navigation_result_pub.publish(result)
                rospy.loginfo("Published result")
                num = 0
                rospy.sleep(2.0)
                result.data = False
                rospy.sleep(0.1)
                self.navigation_result_pub.publish(result)
                return 0
            elif num == 4:
                rospy.loginfo("Buried in obstacle")
                self.clear_costmaps()
                rospy.loginfo(" Clear Costmaps")
                rospy.sleep(1.0)
                return 2

if __name__ == '__main__':
    rospy.init_node('ggi_navigation', anonymous = True)
    try:
        nav = Navigation()
        state = 0
        while not rospy.is_shutdown():
            if state == 0:
                state = nav.waitTopic()
            elif state == 1:
                state = nav.setLocationList()
            elif state == 2:
                state = nav.navigateToDestination()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Interrupted")
 
