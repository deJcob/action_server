#! /usr/bin/env python
import rospy
import actionlib
import action_server.msg
import copy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan


class SideDockingSecondAction(object):
    # create messages that are used to publish feedback/result
    _feedback = action_server.msg.SideDockingSecondFeedback()
    _result = action_server.msg.SideDockingSecondResult()
    _odom = Odometry()
    _start_pos = Odometry()
    _start_pos_of_back = Odometry()
    _joint_states = JointState()
    _range0 = Range()
    _range1 = Range()
    _range2 = Range()
    _range3 = Range()
    _safetyRange0 = Range()
    _safetyRange1 = Range()
    _safetyRange2 = Range()
    _safetyRange3 = Range()
    _lidarRanges = LaserScan()
    _lidarRange = float
    _dockingStationLength = float
    _startMeasuringLength = False
    _startGoingBack = False
    _measuredLength = False
    _goForward = True
    _distanceTraveledBackward = float

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, action_server.msg.SideDockingSecondAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        try:
            rospy.Subscriber("/diff_drive/odom", Odometry, self.callbackOdom)
            rospy.Subscriber("/joint_states/", JointState, self.callbackJoints)
            rospy.Subscriber("/robot_driver/laser_ruler/scan_4", Range, self.rulerCallback0)
            rospy.Subscriber("/robot_driver/laser_ruler/scan_5", Range, self.rulerCallback1)
            rospy.Subscriber("/robot_driver/laser_ruler/scan_6", Range, self.rulerCallback2)
            rospy.Subscriber("/robot_driver/laser_ruler/scan_7", Range, self.rulerCallback3)
            #safety subscribers
            rospy.Subscriber("/robot_driver/laser_ruler/scan_0", Range, self.rulerSafetyCallback0)
            rospy.Subscriber("/robot_driver/laser_ruler/scan_1", Range, self.rulerSafetyCallback1)
            rospy.Subscriber("/robot_driver/laser_ruler/scan_2", Range, self.rulerSafetyCallback2)
            rospy.Subscriber("/robot_driver/laser_ruler/scan_3", Range, self.rulerSafetyCallback3)
            
            rospy.Subscriber("/scan", LaserScan, self.lidarCallback)
            self._pub = rospy.Publisher('/diff_drive/cmd_vel', Twist, queue_size=10) 
        except rospy.ROSInterruptException:
            exit
        rospy.loginfo('%s: Server started!' % self._action_name)
    
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(100)
        success = False

        # init feedback message
        self._feedback.measuredLength = 0.0
        self._start_pos = 0.0
        self._start_pos_of_back = 0.0
        self._dockingStationLength = 0.2
        self._feedback.distToDock = 0.0
        self._distanceTraveledBackward = 0.0
        #copy.copy(self._odom)

        # publish info to the console for the user

        rospy.loginfo('%s: Executing, traveling to side docking station with velocity %f [m/s]' + 
            ', with docking velocity %f [m/s])', self._action_name,
            goal.velNormal, goal.velDocking)
        
        # start executing the action
        while not success: 
            cmd_vel = Twist()
            # cmd_vel.linear.x = goal.velNormal
            cmd_vel.linear.x = 0.0


            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                cmd_vel.linear.x = 0.0
                self._pub.publish(cmd_vel) 
                success = False
                break

            #TODO: add measuring length
            countOfBackSensorsTriggered =\
                int(self._range2 <= goal.distToStop) +\
                int(self._range3 <= goal.distToStop)

            countOfFrontSensorsTriggered =\
                int(self._range0 <= goal.distToStop) +\
                int(self._range1 <= goal.distToStop)

            countOfSafetySensorsTriggered =\
                int(self._safetyRange0 <= 0.18) +\
                int(self._safetyRange1 <= 0.18) +\
                int(self._safetyRange2 <= 0.18) +\
                int(self._safetyRange3 <= 0.18)

            countOfControlSensorsTriggered = countOfFrontSensorsTriggered
            
            if (countOfSafetySensorsTriggered > 1):
                cmd_vel.linear.x = 0.0
                rospy.loginfo("Safety Breaking")
                success = True
            else:
            #change if another scenario
                if(self._goForward):
                    # countOfControlSensorsTriggered = countOfFrontSensorsTriggered
                    cmd_vel.linear.x = goal.velNormal

                    if(countOfControlSensorsTriggered>1):
                        cmd_vel.linear.x = goal.velDocking
                        if(not self._startMeasuringLength):
                            self._start_pos = copy.copy(self._odom)
                            self._startMeasuringLength = True
                        elif(self._startMeasuringLength):
                            x = abs(self._start_pos.pose.pose.position.x - self._odom.pose.pose.position.x)
                            y = abs(self._start_pos.pose.pose.position.y - self._odom.pose.pose.position.y)
                            rospy.loginfo(self._feedback.measuredLength)
                            self._feedback.measuredLength = math.sqrt(x*x+y*y)
                            self._feedback.distToDock = 0.5 * self._feedback.measuredLength
                        else:
                            # no breaking
                            cmd_vel.linear.x = 0.0

                            #counter-current breaking
                            #cmd_vel.linear.x = -1 * goal.velDocking

                            if self._joint_states.velocity[0] == 0 and self._joint_states.velocity[1] == 0:
                                cmd_vel.linear.x = 0.0
                                self._pub.publish(cmd_vel) 
                                self._goForward = False
                                rospy.loginfo("Ruler distances: %f, %f, %f, %f", self._range0, self._range1, self._range2, self._range3)
                else:
                #goBack
                    cmd_vel.linear.x = -1 * goal.velDocking

                    # if(countOfControlSensorsTriggered>1):
                    if(countOfControlSensorsTriggered>1):
                        # cmd_vel.linear.x = -1 * goal.velDocking
                        if(not self._startGoingBack):
                            self._start_pos_of_back = copy.copy(self._odom)
                            self._startGoingBack = True
                        elif(self._startMeasuringLength and self._distanceTraveledBackward < 0.5 * self._feedback.measuredLength):
                            x = abs(self._start_pos_of_back.pose.pose.position.x - self._odom.pose.pose.position.x)
                            y = abs(self._start_pos_of_back.pose.pose.position.y - self._odom.pose.pose.position.y)
                            rospy.loginfo(self._feedback.measuredLength)
                            self._distanceTraveledBackward = math.sqrt(x*x+y*y)
                            self._feedback.distToDock = 0.5 * self._feedback.measuredLength - self._distanceTraveledBackward
                        else:
                            # no breaking
                            cmd_vel.linear.x = 0.0

                            #counter-current breaking
                            #cmd_vel.linear.x = -1 * goal.velDocking

                            if self._joint_states.velocity[0] == 0 and self._joint_states.velocity[1] == 0:
                                cmd_vel.linear.x = 0.0
                                self._pub.publish(cmd_vel) 
                                success = True
                                rospy.loginfo("Ruler distances: %f, %f, %f, %f", self._range0, self._range1, self._range2, self._range3)



            # publish seted velocity
            self._pub.publish(cmd_vel) 
            # publish the feedback
            self._as.publish_feedback(self._feedback)
        
            r.sleep()

        if success:
            self._result.dockingStationLength = self._feedback.measuredLength
            self._result.dockingDistance = self._feedback.distToDock

            rospy.loginfo('%s: Goal reached. %f [m] to docking, %f measured length: [m]', self._action_name, self._result.dockingDistance, self._result.dockingStationLength)
            self._as.set_succeeded(self._result)

    def callbackOdom(self, data):
        self._odom = data 

    def callbackJoints(self, data):
        self._joint_states = data

    def rulerCallback0(self, data):
        self._range0 = data.range

    def rulerCallback1(self, data):
        self._range1 = data.range

    def rulerCallback2(self, data):
        self._range2 = data.range

    def rulerCallback3(self, data):
        self._range3 = data.range

    def rulerSafetyCallback0(self, data):
        self._safetyRange0 = data.range

    def rulerSafetyCallback1(self, data):
        self._safetyRange1 = data.range

    def rulerSafetyCallback2(self, data):
        self._safetyRange2 = data.range

    def rulerSafetyCallback3(self, data):
        self._safetyRange3 = data.range

    def lidarCallback(self, data):
        self._lidarRanges = data
        # self._lidarRange = data.ranges[89]
        self._lidarRange = data.ranges[29]
        
if __name__ == '__main__':
    rospy.init_node('SideDockingSecond')
    server = SideDockingSecondAction(rospy.get_name())
    rospy.spin()