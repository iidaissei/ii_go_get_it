#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Twist
from ii_go_get_it.msg import LearnContent

class MimiControlClass():
    def __init__(self):
        #Publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)
        self.m6_pub = rospy.Publisher('/m6_controller/command', Float64, queue_size = 1)
        self.tts_pub = rospy.Publisher('/tts', String, queue_size = 1)

    def motorControl(self, motor_name, value):
        if motor_name == 6:
            m6_angle = Float64()
            m6_angle = value
            rospy.sleep(0.1)
            self.m6_pub.publish(m6_angle)
            rospy.sleep(1.0)

    def ttsSpeak(self, sentense):
        data = String()
        data.data = sentense
        rospy.sleep(0.1)
        self.tts_pub.publish(data)
        rospy.sleep(2.0)

class NavigationClass():
    def __init__(self):
        #Publisher
        self.navigation_memorize_pub = rospy.Publisher('/navigation/memorize_place', String, queue_size = 1)#目的地を記憶
        self.navigation_command_pub = rospy.Publisher('/navigation/move_place', String, queue_size = 1)#ナビゲーション開始の命令
        #Subscriber
        self.navigation_res_sub = rospy.Subscriber('/navigation/result', Bool, self.getNavigationResultCB)
        
        self.navigation_result_flg = False
        self.mimi = MimiControlClass()

    def getNavigationResultCB(self, result_msg):
        self.navigation_result_flg = result_msg.data

    def setPlace(self, receive_msg): 
        place_name = String()
        place_name = receive_msg
        print place_name
        rospy.loginfo(" Memorize " + str(receive_msg))
        rospy.sleep(0.1)
        self.navigation_memorize_pub.publish(place_name)
        while self.navigation_result_flg == False and not rospy.is_shutdown():
            rospy.sleep(0.5)
            rospy.loginfo(" Memorizing...")
        self.navigation_result_flg = False
        rospy.loginfo(" Memorized " + str(receive_msg))
        self.mimi.ttsSpeak("I memorized " + str(receive_msg))
        rospy.sleep(1.0)

    def movePlace(self, receive_msg):
        self.mimi.motorControl(6, 0.3)
        place_name = String()
        place_name.data = receive_msg
        print place_name
        rospy.loginfo(" Move to " + str(place_name.data))
        rospy.sleep(0.1)
        self.navigation_command_pub.publish(place_name)
        while self.navigation_result_flg == False and not rospy.is_shutdown():
            rospy.sleep(2.5)
            rospy.loginfo(" Moving...")
        self.navigation_result_flg = False
        rospy.loginfo(" Arrived " + str(place_name.data))
        rospy.sleep(1.0)

class ManipulationClass():
    def __init__(self):
        #Publisher
        self.object_grasp_request_pub = rospy.Publisher('/object/grasp_req', String, queue_size = 10)#manipulationの開始
        self.change_pose_request_pub = rospy.Publisher('/arm/changing_pose_req',String,queue_size=1)
        #Subscriber
        self.object_grasp_res_sub = rospy.Subscriber('/object/grasp_res', Bool, self.getObjectGraspResultCB)
        self.changing_pose_res_sub = rospy.Subscriber('/arm/change_pose_res', Bool, self.getChangingPoseCB)

        self.object_grasp_result_flg = False
        self.changing_pose_res_flg = False
        self.object_name = String()
        self.mimi = MimiControlClass()

    def getObjectGraspResultCB(self, result_msg):
        self.object_grasp_result_flg = result_msg.data

    def getChangingPoseCB(self, receive_msg):
        self.changing_pose_res_flg = receive_msg.data

    def stringMessagePublish(self, target_topic, receive_msg):#String型専用Publisher
        while not rospy.is_shutdown():
            request = String()
            request.data = receive_msg
            rospy.sleep(0.5)
            if target_topic == 'object_grasp':
                self.object_grasp_request_pub.publish(request)
                break
            elif target_topic == 'change_pose':
                self.change_pose_request_pub.publish(request)
                break
        rospy.loginfo(" Publeshed " + target_topic + "_request " + receive_msg)

    def grasp(self):
        while not rospy.is_shutdown():
            rospy.loginfo("Start grasp")
            rospy.sleep(0.1)
            self.mimi.motorControl(6, -0.07)
            rospy.sleep(0.1)
            self.stringMessagePublish('object_grasp', 'object')
            while not rospy.is_shutdown() and self.object_grasp_result_flg == False:
                rospy.loginfo(" Waiting for grasping...")
                rospy.sleep(2.5)
            self.object_grasp_result_flg = False
            rospy.loginfo(" Grasped")
            rospy.sleep(0.1)
            self.mimi.motorControl(6, 0.3)
            rospy.loginfo(" Finished grasp")
            break

    def handOver(self):#objectを手渡す
        while not rospy.is_shutdown():
            rospy.loginfo(" Start hand over")
            rospy.sleep(0.1)
            self.mimi.motorControl(6, 0.5)
            rospy.sleep(0.2)
            self.mimi.ttsSpeak("Here you are")
            rospy.sleep(0.1)
            self.stringMessagePublish('change_pose', 'give')
            while not rospy.is_shutdown() and self.changing_pose_res_flg == False:
                rospy.sleep(1.0)
            self.changing_pose_res_flg = True
            rospy.sleep(1.0)
            self.mimi.motorControl(6, 0.3)
            rospy.sleep(2.0)
            rospy.loginfo(" Finished hand over")
            break


class TraningPhase():
    def __init__(self):
        #Publisher
        self.traning_request_pub = rospy.Publisher('/ggi/traning_request', String ,queue_size = 1)#traning用APIの起動・停止
        self.learn_request_pub = rospy.Publisher('/ggi/learn_request', String, queue_size = 1)#学習用APIの起動・停止
        self.follow_request_pub = rospy.Publisher('/chase/request', String, queue_size = 1)#followの開始・終了
        #Subscriber
        rospy.Subscriber('/ggi/voice_cmd', String, self.getVoiceCmdCB)
        rospy.Subscriber('/ggi/learn_content', String, self.getLearnContentCB)

        self.voice_cmd = 'Null'
        self.phase_change_flg = False
        self.learn_content_flg = False
        self.mimi = MimiControlClass()
        self.navi = NavigationClass()

    def getVoiceCmdCB(self, receive_msg):
        self.voice_cmd = receive_msg.data
        print self.voice_cmd

    def getLearnContentCB(self, receive_msg):
        self.place_name = receive_msg.data
        #self.object_name = receive_msg.obj
        self.learn_content_flg = True

    def stringMessagePublish(self, target_topic, receive_msg):#String型専用Publisher
        while not rospy.is_shutdown():
            request = String()
            request.data = receive_msg
            rospy.sleep(0.1)
            if target_topic == 'traning':
                self.traning_request_pub.publish(request)
                break
            elif target_topic == 'follow':
                self.follow_request_pub.publish(request)
                break
            elif target_topic == 'learn':
                self.learn_request_pub.publish(request)
                break
        rospy.loginfo(" Publeshed " + target_topic + "_request " + receive_msg)

    def doorOpenStart(self):
        try:
            while not rospy.is_shutdown() and self.mimi.front_laser_dist == 999.9:
                rospy.sleep(1.0)
            initial_distance = self.mimi.front_laser_dist
            print initial_distance
            self.mimi.ttsSpeak("Please open the door")
            while not rospy.is_shutdown() and self.mimi.front_laser_dist <= initial_distance + 0.10:
                rospy.loginfo(" Waiting for door open")
                rospy.sleep(2.0)
            rospy.sleep(2.0)
            self.mimi.ttsSpeak("Thank you")
            rospy.sleep(0.1)
            for i in range(12):
                self.mimi.linearControl(0.3)#linear0.3を15回で80cm前進
                rospy.sleep(0.2)
            rospy.sleep(0.5)
            rospy.loginfo(" Enter the room")
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass


    def learn(self):#場所のみ記憶させる仕様
        while not rospy.is_shutdown():
            self.stringMessagePublish('learn', 'start')#学習用APIの起動
            while not rospy.is_shutdown() and self.learn_content_flg == False:
                rospy.loginfo(" Waiting for learning")
                rospy.sleep(1.0)
            self.learn_content_flg = False
            rospy.sleep(1.0)
            self.navi.setPlace(self.place_name)
            rospy.sleep(1.0)
            break

    def traning(self):
        try:
            rospy.sleep(0.1)
            self.stringMessagePublish('traning', 'start')
            rospy.sleep(1.0)
            while not rospy.is_shutdown() and not self.voice_cmd == 'finish_traning':
                if self.voice_cmd == 'follow_me':#-------------------->follow開始
                    rospy.sleep(0.5)
                    self.stringMessagePublish('follow', 'start')
                    rospy.loginfo(" Start following")
                    self.mimi.ttsSpeak("I'll follow you")
                    self.voice_cmd = 'Null'
                elif self.voice_cmd == 'stop_follow':#--------------->follow終了
                    self.stringMessagePublish('follow', 'stop')
                    rospy.loginfo(" Stop following")
                    self.mimi.ttsSpeak("Stop following")
                    self.voice_cmd = 'Null'
                elif self.voice_cmd == 'start_learning':#------------>学習開始
                    rospy.loginfo(" Start learning")
                    #self.mimi.ttsSpeak("Please tell me")
                    self.learn()
                    self.mimi.ttsSpeak("Learning is over")
                    self.voice_cmd = 'Null'
                else:
                    rospy.loginfo(" * Waiting for voice_cmd *")
                    rospy.sleep(1.0)
            self.navi.setPlace('operator')#TraningPhaseの最後にオペレーターの場所を記憶
            self.stringMessagePublish('traning', 'stop')
            rospy.sleep(0.1)
            self.phase_change_flg = True
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

    def master(self):
        try:
            print '-'*80
            rospy.loginfo(" Start Go Get It")
            rospy.sleep(0.3)
            self.mimi.motorControl(6, 0.3)
            rospy.sleep(0.1)
            self.doorOpenStart()
            rospy.sleep(0.1)
            rospy.loginfo(" Start TraningPhase")
            self.mimi.ttsSpeak("Start TraningPhase")
            while not rospy.is_shutdown() and self.phase_change_flg == False:
                self.traning()
                rospy.sleep(0.1)
            rospy.loginfo(" Finished TraningPhase")
            self.mimi.ttsSpeak("Finished TraningPhase")
            rospy.sleep(5.0)#--------------------------------------------------------当日判断する（秒数変更） 
            return 1
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass


class TestPhase():
    def __init__(self):
        #Publisher
        self.mission_listen_request_pub = rospy.Publisher('/ggi/mission_listen', String, queue_size = 1)
        #Subscriber
        rospy.Subscriber('/ggi/order_content', String, self.getOrderContentCB)

        self.order_place = 'Null'
        self.mimi = MimiControlClass()
        self.navi = NavigationClass()
        self.mani = ManipulationClass()

    def getOrderContentCB(self, receive_msg):
        self.order_place = receive_msg.data

    def stringMessagePublish(self, target_topic, receive_msg):#String型専用Publisher
        while not rospy.is_shutdown():
            request = String()
            request.data = receive_msg
            rospy.sleep(0.1)
            if target_topic == 'mission_listen':
                self.mission_listen_request_pub.publish(request)
                break
        rospy.loginfo(" Publeshed " + target_topic + "_request " + receive_msg)

    def listenOrder(self):
        rospy.loginfo(" Listening order...")
        rospy.sleep(0.1)
        self.mimi.motorControl(6, 0.5)
        rospy.sleep(0.1)
        #self.mimi.ttsSpeak(" Please give me a mission")
        self.stringMessagePublish('mission_listen', 'start')
        while not rospy.is_shutdown() and self.order_place == 'Null':
            rospy.loginfo(" Waiting for order")
            rospy.sleep(2.0)
        self.order_place = 'Null'
        rospy.sleep(1.0)
        self.mimi.ttsSpeak("Roger")
        print self.order_place
        print self.order_object

    def missionExecution(self):
        try:
            rospy.loginfo(" Start mission")
            mission_state = 0
            while not rospy.is_shutdown() and not mission_state == 5:
                if mission_state == 0:
                   self.listenOrder()
                   mission_state = 1
                elif mission_state == 1:
                    self.navi.movePlace(self.order_place)
                    mission_state = 2
                elif mission_state == 2:
                    self.mani.grasp()
                    mission_state = 3
                elif mission_state == 3:
                    self.navi.movePlace('operator')
                    mission_state = 4
                elif mission_state == 4:
                    self.mani.handOver()
                    mission_state = 5
            rospy.loginfo(" Finished mission")
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

    def master(self):
        try:
            print '-'*80
            rospy.loginfo(" Start TestPhase")
            self.mimi.ttsSpeak("Start TestPhase")
            #API起動・
            order_count = 1
            while not rospy.is_shutdown() and not order_count == 2:#３回のチャレンジ
                print '-'*80
                rospy.loginfo(" Mission number : " + str(order_count))
                self.missionExecution()
                order_count += 1
            print '-'*80
            rospy.loginfo(" All mission complete")
            rospy.sleep(1.0)
            rospy.loginfo(" Finished TestPhase")
            self.mimi.ttsSpeak("Finished TestPhase")
            rospy.sleep(1.0)
            rospy.loginfo(" Finished Go Get It")
            self.mimi.ttsSpeak("Finished Go Get It")
            return 2
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

def main():
    try:
        state = 0
        traning = TraningPhase()
        test = TestPhase()
        while not rospy.is_shutdown() and not state == 2:
            if state == 0:
                state = traning.master()
            elif state == 1:
                state = test.master()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Interrupted")
        pass

if __name__ == '__main__':
    rospy.init_node('ggi_master', anonymous = True)
    main()
