#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State


TOL = 0.1
N_SQUARES = 3

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
        smach.State.__init__(self, outcomes=['repeat', 'succeeded'], input_keys=['counter_in'], output_keys=['counter_out'])

    def execute(self, userdata):
        goal_pose.pose.position.x = 5
        goal_pose.pose.position.y = 0
        while not rospy.is_shutdown() and abs(goal_pose.pose.position.x - current_pose.pose.position.x) > TOL:
            local_position_pub.publish(goal_pose)
            rate.sleep()
        wait(1)

        if self.preempt_requested():
            rospy.loginfo("Quadrado interrompido")
            self.service_preempt()
            return 'preempted'

        goal_pose.pose.position.x = 5
        goal_pose.pose.position.y = 5
        while not rospy.is_shutdown() and abs(goal_pose.pose.position.y - current_pose.pose.position.y) > TOL:
            local_position_pub.publish(goal_pose)
            rate.sleep()
        wait(1)

        if self.preempt_requested():
            rospy.loginfo("Quadrado interrompido")
            self.service_preempt()
            return 'preempted'

        goal_pose.pose.position.x = 0
        goal_pose.pose.position.y = 5
        while not rospy.is_shutdown() and abs(goal_pose.pose.position.x - current_pose.pose.position.x) > TOL:
            local_position_pub.publish(goal_pose)
            rate.sleep()
        wait(1)

        if self.preempt_requested():
            rospy.loginfo("Quadrado interrompido")
            self.service_preempt()
            return 'preempted'

        goal_pose.pose.position.x = 0
        goal_pose.pose.position.y = 0
        while not rospy.is_shutdown() and abs(goal_pose.pose.position.y - current_pose.pose.position.y) > TOL:
            local_position_pub.publish(goal_pose)
            rate.sleep()
        wait(1)

        if self.preempt_requested():
            rospy.loginfo("Quadrado interrompido")
            self.service_preempt()
            return 'preempted'

        if userdata.counter_in == N_SQUARES:
            return 'succeeded'
        else:
            userdata.counter_out = userdata.counter_in + 1
            return 'repeat'


class Return(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        goal_pose.pose.position.x = 0
        goal_pose.pose.position.y = 0
        while not rospy.is_shutdown() and (abs(goal_pose.pose.position.x - current_pose.pose.position.x) > TOL or abs(goal_pose.pose.position.y - current_pose.pose.position.y) > TOL):
            local_position_pub.publish(goal_pose)
            rate.sleep()

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


def child_termination_cb(outcome_map):
    """
    @type child_termination_cb: callale
    @param child_termination_cb: This callback gives the user the ability
    to force the concurrence to preempt running states given the
    termination of some other set of states. This is useful when using
    a concurrence as a monitor container. 

    This callback is called each time a child state terminates. It is
    passed a single argument, a dictionary mapping child state labels
    onto their outcomes. If a state has not yet terminated, it's dict
    value will be None.

    This function can return three things:
    - False: continue blocking on the termination of all other states
    - True: Preempt all other states
    - list of state labels: Preempt only the specified states

    I{If you just want the first termination to cause the other children
    to terminate, the callback (lamda so: True) will always return True.}
    """
    if outcome_map['Square'] == 'succeeded' or outcome_map['Square'] == 'repeat' or outcome_map['Monitor'] == 'invalid':
        return True
    else:
        return False

def outcome_cb(outcome_map):
    """
    @type outcome_cb: callable
    @param outcome_cb: If the outcome policy needs to be more complicated
    than just a conjunction of state outcomes, the user can supply
    a callback for specifying the outcome of the container.

    This callback is called only once all child states have terminated,
    and it is passed the dictionary mapping state labels onto their
    respective outcomes.

    If the callback returns a string, it will treated as the outcome of
    the container.

    If the callback returns None, the concurrence will first check the
    outcome_map, and if no outcome in the outcome_map is satisfied, it
    will return the default outcome.

    B{NOTE: This callback should be a function ONLY of the outcomes of
    the child states. It should not access any other resources.} 
    """
    if outcome_map['Square'] == 'succeeded':
        return 'succeeded'
    elif outcome_map['Square'] == 'repeat':
        return 'repeat'
    else:
        return 'preempted'

def monitor_cb(userdata, msg):
    if msg.data:
        return False
    else:
        return True



sm = smach.StateMachine(outcomes=['succeeded'])
# sm.userdata.counter = 1
with sm:
    smach.StateMachine.add('Setup', Setup(),
                            transitions={'succeeded':'Takeoff'})

    smach.StateMachine.add('Takeoff', Takeoff(),
                            transitions={'succeeded':'Mission'})


    mission_concurrence = smach.Concurrence(outcomes=['repeat', 'succeeded', 'preempted'],
                                            default_outcome='succeeded',
                                            child_termination_cb=child_termination_cb,
                                            outcome_cb=outcome_cb)
    mission_concurrence.userdata.counter = 1
    with mission_concurrence:
        smach.Concurrence.add('Square', Square(),
                            remapping={'counter_in':'counter',
                                        'counter_out':'counter'})
        smach.Concurrence.add('Monitor', smach_ros.MonitorState("/square_interrupt", Bool, monitor_cb))

    smach.StateMachine.add('Mission', mission_concurrence,
                            transitions={'repeat':'Mission',
                                         'succeeded':'Land',
                                         'preempted':'Return'})
                            # remapping={'counter':'counter'})
                            

    smach.StateMachine.add('Return', Return(),
                            transitions={'succeeded':'Land'})

    smach.StateMachine.add('Land', Land(),
                            transitions={'succeeded':'succeeded'})


sis = smach_ros.IntrospectionServer('square_server', sm, '/SQUARE_MISSION')
sis.start()

outcome = sm.execute()

rospy.spin()
sis.stop()