#! /usr/bin/env python
import rospy
import actionlib
import action_server.msg
import copy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan


class DistanceAction(object):
    # create messages that are used to publish feedback/result
    _feedback = action_server.msg.DistanceActionFeedback()
    _result = action_server.msg.DistanceActionResult()
    _odom = Odometry()
    _start_pos = Odometry()
    _joint_states = JointState()
    _lidarRanges = LaserScan()
    _lidarRangeFront = float
    _lidarRangeBack = float
    _inited = False
    _goForward = False

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, action_server.msg.DistanceAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        try:
            rospy.Subscriber("/diff_drive/odom", Odometry, self.callbackOdom)
            rospy.Subscriber("/joint_states/", JointState, self.callbackJoints)
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
        if goal.useLidar:
            start_msg += 'lidar.'

        rospy.loginfo('%s: Executing, traveling according to %s by distance %f (%f [m/s])' + 
            ', stopping in %f [m] (v = %f [m/s])', self._action_name, start_msg, goal.distToStop,
            goal.velNormal, goal.distToDocking, goal.velDocking)
        
        # start executing the action
        while not success: 
            cmd_vel = Twist()
            cmd_vel.linear.x = goal.velNormal
            if(goal.velNormal>0 and goal.velDocking>0):
                self._goForward = True
            elif(goal.velNormal<0 and goal.velDocking<0):
                self._goForward = False
            else:
                success = False
                break

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

            if (goal.useLidar):
                if(self._goForward):
                    if(self._lidarRangeFront<=goal.distToDocking):
                        cmd_vel.linear.x = goal.velDocking
                    elif(self._lidarRangeFront<=goal.distToStop):
                        cmd_vel.linear.x = -1 * goal.velNormal
                        if self._joint_states.velocity[0] == 0 and self._joint_states.velocity[1] == 0:
                            cmd_vel.linear.x = 0.0
                            self._pub.publish(cmd_vel) 
                            success = True
                else:
                    if(self._lidarRangeBack<=goal.distToDocking):
                        cmd_vel.linear.x = goal.velDocking
                    elif(self._lidarRangeBack<=goal.distToStop):
                        cmd_vel.linear.x = -1 * goal.velNormal
                        if self._joint_states.velocity[0] == 0 and self._joint_states.velocity[1] == 0:
                            cmd_vel.linear.x = 0.0
                            self._pub.publish(cmd_vel) 
                            success = True

            elif(goal.useEncoder):
                self._feedback.distToDock = goal.distToStop - self._feedback.distTraveled 
            
                # Moving in docking zone 
                if goal.distToStop - self._feedback.distTraveled < goal.distToDocking:
                    cmd_vel.linear.x = goal.velDocking

                # breaking zone
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

    def lidarCallback(self, data):
        self._lidarRanges = data
        self._lidarRangeFront = data.ranges[0]
        self._lidarRangeBack = data.ranges[179]
        
if __name__ == '__main__':
    rospy.init_node('Distance')
    server = DistanceAction(rospy.get_name())
    rospy.spin()