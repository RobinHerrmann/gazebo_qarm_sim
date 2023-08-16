import rclpy
from rclpy.node import Node
from rclpy import Parameter

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rclpy.duration import Duration

import time
import math
from simple_pid import PID

class JointEffortController(Node):

    def __init__(self, controller_name, joint_names, type):
        self.controller_name = controller_name
        self.joint_names = joint_names
        self.type = type

        super().__init__('{}_publisher'.format(self.controller_name))
        self.set_parameters([Parameter('use_sim_time', value = 1)])

        self.publisher = self.create_publisher(Float64MultiArray, 
                            '/{}/commands'.format(self.controller_name), 10)

    def command(self, data, duration):
        # Init message
        msg = Float64MultiArray()

        for i in range(len(data)):
            data[i] = float(data[i])

        msg.data = data
        self.publisher.publish(msg)
        self.get_logger().info('Publishing:'+str(data))

        # Spin node
        rclpy.spin_once(self, timeout_sec=0)


class position_effort_PID(Node):

    def __init__(self, controller_name, effort_controller_name, joint_names, PID_values):
        self.controller_name = controller_name
        self.effort_controller_name = effort_controller_name
        self.joint_names = joint_names

        self.effort_control = JointEffortController(
            effort_controller_name, joint_names, 'effort'
        )

        super().__init__(self.controller_name)
        self.set_parameters([Parameter('use_sim_time', value = 1)])

        self.publisher = self.create_publisher(Float64MultiArray, 
                            '/{}/commands'.format(self.controller_name), 10)
        self.subscriber = self.create_subscription(JointState, 
                            '/joint_states', self.subscriber_callback, 10)
        
        self.joint_state = None
        while self.joint_state == None:
            rclpy.spin_once(self, timeout_sec=0)
        self.target = [0]*len(self.joint_names)

        self.PIDs = []
        for j in range(len(self.joint_names)):
            self.PIDs.append(PID(PID_values[j][0], PID_values[j][1], PID_values[j][2], setpoint=0, auto_mode=True, sample_time=0.001))

        self.timer = self.create_timer(0.001, self.timer_callback)

        
    def subscriber_callback(self, msg):
        if type(msg) == JointState:
            self.joint_state = msg
            self.joint_indexes = []
            for joint in self.joint_names:
                self.joint_indexes.append(self.joint_state.name.index(joint))
            print(list(msg.effort))

    def timer_callback(self):
        command = []
        for j, joint in enumerate(self.joint_names):
            # error = self.target[j] - self.joint_state.position[self.joint_indexes[j]]

            value = self.PIDs[j](self.joint_state.position[self.joint_indexes[j]])
            
            command.append(value)
        self.effort_control.command(command, 0)
        self.get_logger().info(str(command))



    def command(self, command):
        for i in range(len(command)):
            self.PIDs[i].setpoint = command[i]

if __name__ == "__main__":
    # arm_control = position_effort_PID(
    #     'joint_arm_position_controller', 'joint_gripper_effort_controller', ['base_yaw_joint', 'yaw_bicep_joint', 
    #     'bicep_forearm_joint', 'forearm_endeffector_joint'], 'position'
    # )
    rclpy.init()

    P = 20
    I = 5
    D = 0.005

    arm_control = position_effort_PID(
        'joint_arm_position_controller', 'joint_arm_effort_controller', ['base_yaw_joint', 'yaw_bicep_joint', 
        'bicep_forearm_joint', 'forearm_endeffector_joint'],
        [[50, 0.01, 0.5], [50, 1, 0.5], [10, 0.01, 0.5], [10, 0.1, 0.01]]
    )

    # arm_control.effort_control.command([0, -100, 0, 0], 1)
    rclpy.spin(arm_control)

    rclpy.shutdown()


'''
sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=14, nanosec=356000000), frame_id=''), 
name=['base_yaw_joint', 'yaw_bicep_joint', 'bicep_forearm_joint', 'forearm_endeffector_joint', 'a1_joint', 'a2_joint', 'b1_joint', 'b2_joint'], 
position=[-6.789116169692022e-05, 0.8156790161035383, -0.4829319250569739, 0.0028624543520985313, 0.47285326345743606, 0.07383009151911768, 0.4728878636643854, 0.07653945695498265], 
velocity=[-1.0695586233959117e-06, 0.004945736226729604, 0.004080804548339126, 0.021543082826538593, -0.008080281639447292, 0.002792168693074573, -0.02911803234231678, 0.02454509137897431], 
effort=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
'''