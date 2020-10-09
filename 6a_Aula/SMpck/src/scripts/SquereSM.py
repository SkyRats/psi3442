#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import mavros_msgs
from std_msgs.msg import Bool
from mavros_msgs import srv
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.msg import Mavlink


ERR = 0.1

goalPose = PoseStamped()
mavPose = PoseStamped()
mavState = State()

repeat = Bool()
repeat.data = True

def MissionFlowCallback(b):
    global repeat
    repeat = b

def setPoint(x, y, z):
    global goalPose
    goalPose.pose.position.x = x
    goalPose.pose.position.y = y
    goalPose.pose.position.z = z
    posePub.publish(goalPose)
    rate.sleep()

def stateCallback(state):
    global mavState
    mavState = state

def localCallback(pose):
    global mavPose
    mavPose.pose.position.x = pose.pose.position.x
    mavPose.pose.position.y = pose.pose.position.y
    mavPose.pose.position.z = pose.pose.position.z



class Takeoff (smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['start'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing state Takeoff\n')
        
        for i in range(100):
            setPoint(0, 0, 0)
        
        while mavState.mode != "OFFBOARD":
            result = setModeSrv(0, "OFFBOARD")
            rate.sleep()

        while not (mavState.armed):
            arm(True)
            rate.sleep()
        
        setPoint(0, 0, 5)
        while not(abs(mavPose.pose.position.x - goalPose.pose.position.x) < ERR and abs(mavPose.pose.position.y - goalPose.pose.position.y) < ERR and abs(mavPose.pose.position.z - goalPose.pose.position.z) < ERR):
            setPoint(0, 0, 5)

        return 'start'


class Square (smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['rep', 'end'])
    
    def execute (self, userdata):
        rospy.loginfo('Doing one more square\n')

        setPoint(0, 5, 5)
        while not(abs(mavPose.pose.position.x - goalPose.pose.position.x) < ERR and abs(mavPose.pose.position.y - goalPose.pose.position.y) < ERR and abs(mavPose.pose.position.z - goalPose.pose.position.z) < ERR):
            setPoint(0, 5, 5) 


        setPoint(5, 5, 5)
        while not(abs(mavPose.pose.position.x - goalPose.pose.position.x) < ERR and abs(mavPose.pose.position.y - goalPose.pose.position.y) < ERR and abs(mavPose.pose.position.z - goalPose.pose.position.z) < ERR):
            setPoint(5, 5, 5)
           

        setPoint(5, 0, 5)
        while not(abs(mavPose.pose.position.x - goalPose.pose.position.x) < ERR and abs(mavPose.pose.position.y - goalPose.pose.position.y) < ERR and abs(mavPose.pose.position.z - goalPose.pose.position.z) < ERR):

            setPoint(5, 0, 5)

        setPoint(0, 0, 5)
        while not(abs(mavPose.pose.position.x - goalPose.pose.position.x) < ERR and abs(mavPose.pose.position.y - goalPose.pose.position.y) < ERR and abs(mavPose.pose.position.z - goalPose.pose.position.z) < ERR):
            setPoint(0, 0, 5)
       
        print(repeat)
        
        if repeat.data:
            return 'rep'
        else:
            return 'end'

class RTL(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome'])

    def execute(self, userdata):
        setPoint(0, 0, 0)
        while not(abs(mavPose.pose.position.x - goalPose.pose.position.x) < ERR and abs(mavPose.pose.position.y - goalPose.pose.position.y) < ERR and abs(mavPose.pose.position.z - goalPose.pose.position.z) < ERR):
            setPoint(0, 0, 0)
        while mavState.armed:
            arm(False)        
        return 'outcome'


rospy.init_node('SquareStateMachine', anonymous = True)
rate = rospy.Rate(20)

posePub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size = 20)
stateSub = rospy.Subscriber("/mavros/state", State, stateCallback)
localNow = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, localCallback)

MissionFlowSub = rospy.Subscriber("MissionFlow", Bool, MissionFlowCallback)

arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
setModeSrv = rospy.ServiceProxy("/mavros/set_mode", SetMode)




sm = smach.StateMachine(outcomes = ['close'])

with sm:

    smach.StateMachine.add('Takeoff', Takeoff(), 
                            transitions={'start':'Mission'})
    
    smach.StateMachine.add('Mission', Square(),
                            transitions={'rep':'Mission',
                                            'end':'RTL'})
    smach.StateMachine.add('RTL', RTL(), transitions={'outcome':'close'})


sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
sis.start()
outcome = sm.execute()
rospy.spin()
sis.stop()



