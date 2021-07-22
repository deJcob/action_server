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


class FrontDockingAction(object):
    # create messages that are used to publish feedback/result
    _feedback = action_server.msg.FrontDockingFeedback()
    _result = action_server.msg.FrontDockingResult()
    _odom = Odometry()
    _start_pos = Odometry()
    _joint_states = JointState()
    _range0 = Range()
    _range1 = Range()
    _range2 = Range()
    _range3 = Range()
    _lidarRanges = LaserScan()
    _lidarRange = float
    _inited = False

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, action_server.msg.FrontDockingAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        try:
            rospy.Subscriber("/diff_drive/odom", Odometry, self.callbackOdom)
            rospy.Subscriber("/joint_states/", JointState, self.callbackJoints)
            rospy.Subscriber("/robot_driver/laser_ruler/scan_0", Range, self.rulerCallback0)
            rospy.Subscriber("/robot_driver/laser_ruler/scan_1", Range, self.rulerCallback1)
            rospy.Subscriber("/robot_driver/laser_ruler/scan_2", Range, self.rulerCallback2)
            rospy.Subscriber("/robot_driver/laser_ruler/scan_3", Range, self.rulerCallback3)
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
        self._feedback.distTraveled = 0.0
        self._start_pos = copy.copy(self._odom)

        # publish info to the console for the user
        start_msg = ''
        if goal.useEncoder:
            start_msg += 'encoder, '
        if goal.usePozyx:
            start_msg += 'pozyx, '
        if goal.useLidar:
            start_msg += 'lidar, '
        if goal.useRuler:
            start_msg += 'ruler, '

        rospy.loginfo('%s: Executing, traveling to docking position according to %s by distance %f (%f [m/s])' + 
            ', docking in %f [m] (v = %f [m/s])', self._action_name, start_msg, goal.distToStop,
            goal.velNormal, goal.distToDocking, goal.velDocking)
        
        # start executing the action
        while not success: 
            cmd_vel = Twist()
            cmd_vel.linear.x = goal.velNormal

            x = abs(self._start_pos.pose.pose.position.x - self._odom.pose.pose.position.x)
            y = abs(self._start_pos.pose.pose.position.y - self._odom.pose.pose.position.y)
            self._feedback.distTraveled = math.sqrt(x*x+y*y)

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                cmd_vel.linear.x = 0.0
                self._pub.publish(cmd_vel) 
                success = False
                break

            # TODO: make use of lidar scans

            # TODO in future change it to readings from laser ruler (!)
            if (goal.useRuler):
                countOfSensorsTriggered = int(self._range0 <= goal.distToStop) + int(self._range1 <= goal.distToStop) + int(self._range2 <= goal.distToStop) + int(self._range3 <= goal.distToStop)
                # if(self._range0 <= goal.distToStop or self._range1 <= goal.distToStop or self._range2 <= goal.distToStop or self._range3 <= goal.distToStop):
                if(self._lidarRange<=goal.distToDocking):
                    cmd_vel.linear.x = goal.velDocking
                if(countOfSensorsTriggered>1):
                    cmd_vel.linear.x = -0.2
                    if self._joint_states.velocity[0] == 0 and self._joint_states.velocity[1] == 0:
                        success = True
                        print("distances: ", self._range0, self._range1, self._range2, self._range3)
                else:
                    countOfSensorsTriggered = int(self._range0 <= goal.distToDocking) + int(self._range1 <= goal.distToDocking) + int(self._range2 <= goal.distToDocking) + int(self._range3 <= goal.distToDocking)
                    if(countOfSensorsTriggered>1):
                        cmd_vel.linear.x = goal.velDocking
                    else:
                        cmd_vel.linear.x = goal.velNormal

            else:
                self._feedback.distToDock = goal.distToStop - self._feedback.distTraveled 
            
                # Moving in docking zone 
                if goal.distToStop - self._feedback.distTraveled < goal.distToDocking:
                    cmd_vel.linear.x = goal.velDocking

                # braking zone
                if goal.distToStop - self._feedback.distTraveled < 0.05:
                    cmd_vel.linear.x = 0.0
                    if self._joint_states.velocity[0] == 0 and self._joint_states.velocity[1] == 0:
                        success = True

            # publish seted velocity
            self._pub.publish(cmd_vel) 
            # publish the feedback
            self._as.publish_feedback(self._feedback)
        
            r.sleep()

        if success:
            self._result.distTraveled = self._feedback.distTraveled
            self._result.distToDock = self._feedback.distToDock

            rospy.loginfo('%s: Goal reached. %f [m] to docking, %f [m] traveled', self._action_name, self._result.distToDock, self._result.distTraveled)
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

    def lidarCallback(self, data):
        self._lidarRanges = data
        self._lidarRange = data.ranges[0]
        
if __name__ == '__main__':
    rospy.init_node('FrontDocking')
    server = FrontDockingAction(rospy.get_name())
    rospy.spin()