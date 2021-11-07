#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State


TOL = 0.1

rospy.init_node("takeoff_land")

# Objetos de comandos e estados
current_state = State()
current_pose = PoseStamped()
goal_pose = PoseStamped()

# Funções de callback
def state_callback(msg):
    global current_state
    current_state = msg

def pose_callback(msg):
    global current_pose
    current_pose = msg

# Objetos de Service, Publisher e Subscriber
arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
state_sub = rospy.Subscriber("/mavros/state", State, state_callback)
pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)

# Frequência de publicação do setpoint
rate = rospy.Rate(20)


def wait(t):
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and rospy.Time.now() - t0 < rospy.Duration(t):
        local_position_pub.publish(goal_pose)
        rate.sleep()


# Máquina de Estados
class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        # Espera a conexão ser iniciada
        rospy.loginfo("Esperando conexão com FCU")
        while not rospy.is_shutdown() and not current_state.connected:
            rate.sleep()

        # Publica algumas mensagens antes de trocar o modo de voo
        for i in range(100):
            local_position_pub.publish(goal_pose)
            rate.sleep()

        # Coloca no modo Offboard
        last_request = rospy.Time.now()
        if (current_state.mode != "OFFBOARD"):
            result = set_mode_srv(0, "OFFBOARD")
            rospy.loginfo("Alterando para modo Offboard")
            while not rospy.is_shutdown() and current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(1.0)):
                result = set_mode_srv(0, "OFFBOARD")
            rospy.loginfo("Drone em modo Offboard")
        else:
            rospy.loginfo("Drone já está em modo Offboard")

        # Arma o drone
        if (not current_state.armed):
            result = arm(True)
            rospy.loginfo("Armando o drone")
            while not rospy.is_shutdown() and not current_state.armed:
                result = arm(True)
            rospy.loginfo("Drone armado")
        else:
            rospy.loginfo("Drone já armado")
        
        return 'succeeded'


class Takeoff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo("Subindo")
        goal_pose.pose.position.z = 4.5
        while not rospy.is_shutdown() and abs(goal_pose.pose.position.z - current_pose.pose.position.z) > TOL:
            local_position_pub.publish(goal_pose)
            rate.sleep()

        rospy.loginfo("Esperando")
        wait(1)

        return 'succeeded'


class Square(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        goal_pose.pose.position.x = 5
        goal_pose.pose.position.y = 0
        while not rospy.is_shutdown() and abs(goal_pose.pose.position.x - current_pose.pose.position.x) > TOL:
            local_position_pub.publish(goal_pose)
            rate.sleep()
        wait(1)

        goal_pose.pose.position.x = 5
        goal_pose.pose.position.y = 5
        while not rospy.is_shutdown() and abs(goal_pose.pose.position.y - current_pose.pose.position.y) > TOL:
            local_position_pub.publish(goal_pose)
            rate.sleep()
        wait(1)

        goal_pose.pose.position.x = 0
        goal_pose.pose.position.y = 5
        while not rospy.is_shutdown() and abs(goal_pose.pose.position.x - current_pose.pose.position.x) > TOL:
            local_position_pub.publish(goal_pose)
            rate.sleep()
        wait(1)

        goal_pose.pose.position.x = 0
        goal_pose.pose.position.y = 0
        while not rospy.is_shutdown() and abs(goal_pose.pose.position.y - current_pose.pose.position.y) > TOL:
            local_position_pub.publish(goal_pose)
            rate.sleep()
        wait(1)

        return 'succeeded'


class Land(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        # Coloca no modo Land
        if (current_state.mode != "AUTO.LAND"):
            result = set_mode_srv(0, "AUTO.LAND")
            rospy.loginfo("Alterando para modo Land")
            while not rospy.is_shutdown() and current_state.mode != "AUTO.LAND":
                result = set_mode_srv(0, "AUTO.LAND")
            rospy.loginfo("Drone em modo Land")
        else:
            rospy.loginfo("Drone já está em modo Land")

        return 'succeeded'


sm = smach.StateMachine(outcomes=['succeeded'])

with sm:
    smach.StateMachine.add('Setup', Setup(),
                            transitions={'succeeded':'Takeoff'})

    smach.StateMachine.add('Takeoff', Takeoff(),
                            transitions={'succeeded':'Square'})

    smach.StateMachine.add('Square', Square(),
                            transitions={'succeeded':'Land'})

    smach.StateMachine.add('Land', Land(),
                            transitions={'succeeded':'succeeded'})


sis = smach_ros.IntrospectionServer('square_server', sm, '/SQUARE_MISSION')
sis.start()

outcome = sm.execute()

rospy.spin()
sis.stop()