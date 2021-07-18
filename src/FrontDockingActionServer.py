#! /usr/bin/env python3
import rospy
import actionlib
import action_server.msg

class FrontDockingAction(object):
    # create messages that are used to publish feedback/result
    _feedback = action_server.msg.FrontDockingFeedback()
    _result = action_server.msg.FrontDockingResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, action_server.msg.FrontDockingAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo('%s: Server started!' % self._action_name)
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(100)
        success = True
        
        # init feedback message
        self._feedback.distTraveled = 0.0

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
        for i in range(1, goal.order):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            r.sleep()
          
        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Goal reached' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('FrontDocking')
    server = FrontDockingAction(rospy.get_name())
    rospy.spin()