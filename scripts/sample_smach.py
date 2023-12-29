#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from std_msgs.msg import String
import time

class WaitForTrigger(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['triggered', 'preempted'])

    def execute(self, userdata):
        rospy.loginfo('Waiting for trigger !!')
        response = rospy.wait_for_message('/trigger', String, timeout=None)
        
        return 'triggered'

class PickObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pick_pose', 'preempted'])

    def execute(self, userdata):
        rospy.loginfo('Picking up object...')
        # Implement the logic here to pick 
        pick = True
        time.sleep(5)
        if pick:
            return 'pick_pose'
        else:
            return 'preempted'

class PlaceObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['place_pose', 'preempted'])

    def execute(self, userdata):
        rospy.loginfo('Placing the object...')
        # Implement the logic to place the object
        place = True
        time.sleep(5)
        if place:
            return 'place_pose'
        else:
            return 'preempted'

class GoToHomePosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['home_position', 'preempted'])

    def execute(self, userdata):
        rospy.loginfo('Going to home position...')
        # Implement the logic to go to home pose
        home = True
        time.sleep(5)
        if home:
            return 'home_position'
        else:
            return 'preempted'

def main():
    rospy.init_node('smach_example_node')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success', 'preempted', 'aborted'])
    with sm:
        smach.StateMachine.add('WAIT_FOR_TRIGGER', WaitForTrigger(),
                               transitions={'triggered': 'PICK_OBJECT',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('PICK_OBJECT', PickObject(),
                               transitions={'pick_pose': 'PLACE_OBJECT',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('PLACE_OBJECT', PlaceObject(),
                               transitions={'place_pose': 'GO_TO_HOME',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('GO_TO_HOME', GoToHomePosition(),
                               transitions={'home_position': 'WAIT_FOR_TRIGGER',
                                            'preempted': 'preempted'})

    # Create and start the SMACH introspection server (for visualization)
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()

    # Execute the SMACH state machine
    outcome = sm.execute()

    # Stop the introspection server
    sis.stop()

    rospy.loginfo('Outcome: %s', outcome)

if __name__ == '__main__':
    main()
