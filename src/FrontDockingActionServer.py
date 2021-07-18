#! /usr/bin/env python3
import rospy
import actionlib
import action_server.msg
import copy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class FrontDockingAction(object):
    # create messages that are used to publish feedback/result
    _feedback = action_server.msg.FrontDockingFeedback()
    _result = action_server.msg.FrontDockingResult()
    _odom = Odometry()
    _start_pos = Odometry()
    _joint_states = JointState()
    _inited = False

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, action_server.msg.FrontDockingAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        try:
            rospy.Subscriber("/diff_drive/odom", Odometry, self.callbackOdom)
            rospy.Subscriber("/joint_states/", JointState, self.callbackJoints)
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

            # TODO in future change it to readings from laser ruler (!)
            self._feedback.distToDock = goal.distToStop - self._feedback.distTraveled 
            
            # Moving in docking zone 
            if goal.distToStop - self._feedback.distTraveled < goal.distToDocking:
                cmd_vel.linear.x = goal.velDocking

            # brakig zone
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
        
if __name__ == '__main__':
    rospy.init_node('FrontDocking')
    server = FrontDockingAction(rospy.get_name())
    rospy.spin()