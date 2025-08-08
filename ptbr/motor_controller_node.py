import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

from sensor_msgs.msg import JointState

from gpiozero import PWMOutputDevice, OutputDevice, DigitalInputDevice
from time import sleep, time

from ptbr.motor_controller import MotorController
import math


class Motor_controller(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.cmd_vel_subscription  # prevent unused variable warning

        self.wheel_state_publisher = self.create_publisher(JointState, 'joint_state/motors', 10)
        self.publish_timer = self.create_timer(0.1, self.publish_joints)

        self.timer = self.create_timer(0.01, self.pid_loop)

        self.r = MotorController(11, 9, 8, 5, 6, KP=0.03, KD=0.001, KI=0.00001)
        self.l = MotorController(10, 22, 25, 23, 24, KP=0.05, KD=0.002, KI=0.000005)
        self.dist = 0.12


    def cmd_vel_callback(self, msg: Twist):
        self.des_lin_vel = msg.linear.x
        self.des_ang_vel = msg.angular.z


        v_r = self.des_lin_vel + self.des_ang_vel * (self.dist/2)
        v_l = self.des_lin_vel - self.des_ang_vel * (self.dist/2)


        self.r.set_speed(v_r)
        self.l.set_speed(v_l)

    def pid_loop(self):
        avg_e_r = self.r.PID_control()
        avg_e_l = self.l.PID_control()

    def publish_joints(self):

        msg = JointState()
        msg.name = ["right_wheel_joint", "left_wheel_joint"]
        msg.position = [self.get_pos_from_pulse(self.r), self.get_pos_from_pulse(self.l)]
        msg.velocity = [self.r.en.get_speed(), self.l.en.get_speed()]

        self.wheel_state_publisher.publish(msg)

    def get_pos_from_pulse(self, m: MotorController):
        current_rotation = m.en.pulse_count/m.en.ppr_total
        minus_full = current_rotation - math.floor(current_rotation)
        return minus_full * 2 * math.pi
def main(args=None):
    rclpy.init(args=args)

    motor_controller = Motor_controller()

    rclpy.spin(motor_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_controller.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()