import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import tf_transformations

from std_msgs.msg import String
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

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
        self.publish_timer = self.create_timer(0.5, self.publish_joints)

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.odom_pub_timer = self.create_timer(0.5, self.publish_odom)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_r = 0
        self.last_l = 0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time()


        self.timer = self.create_timer(0.05, self.pid_loop)

        self.r = MotorController(11, 9, 8, 5, 6, KP=0.05, KD=0.001, KI=0.00001)
        self.l = MotorController(10, 22, 25, 23, 24, KP=0.05, KD=0.002, KI=0.000005)
        self.wheel_seperation = 0.12
        self.radius = 0.0325


    def cmd_vel_callback(self, msg: Twist):
        self.des_lin_vel = msg.linear.x
        self.des_ang_vel = msg.angular.z


        v_r = self.des_lin_vel + self.des_ang_vel * (self.wheel_seperation/2)
        v_l = self.des_lin_vel - self.des_ang_vel * (self.wheel_seperation/2)


        self.r.set_speed(v_r)
        self.l.set_speed(v_l)

    def pid_loop(self):
        avg_e_r = self.r.PID_control()
        avg_e_l = self.l.PID_control()

    def publish_joints(self):

        r_speed = self.r.en.get_speed()
        l_speed = self.l.en.get_speed()

        msg = JointState()
        msg.name = ["right_wheel_joint", "left_wheel_joint"]
        msg.position = [self.get_pos_from_pulse(self.r), self.get_pos_from_pulse(self.l)]
        msg.velocity = [r_speed, l_speed]

        self.wheel_state_publisher.publish(msg)

    def get_pos_from_pulse(self, m: MotorController):
        current_rotation = m.en.pulse_count/m.en.ppr_total
        minus_full = current_rotation - math.floor(current_rotation)
        return minus_full * 2 * math.pi

    def publish_odom(self):
        current_pos_r = self.r.en.pulse_count
        movement_r = current_pos_r - self.last_r
        self.last_r = current_pos_r
        movement_r = (movement_r/self.r.en.ppr_total) * math.pi * 2 * self.radius

        current_pos_l = self.l.en.pulse_count
        movement_l = current_pos_l - self.last_l
        self.last_l = current_pos_l
        movement_l = (movement_l/self.l.en.ppr_total) * math.pi * 2 * self.radius

        current_time = time()
        dt = current_time - self.last_time
        self.last_time = current_time

        r_speed = movement_r / dt
        l_speed = movement_l / dt

        linear_x = (r_speed + l_speed) / 2
        angular_z = (r_speed - l_speed) / (self.wheel_seperation * 2)

        delta_x = linear_x * math.cos(self.theta) * dt
        delta_y = linear_x * math.sin(self.theta) * dt
        delta_theta = angular_z * dt


        self.x -= delta_x
        self.y += delta_y
        self.theta += delta_theta

        print(movement_r, movement_l)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        q = tf_transformations.quaternion_from_euler(0,0,self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        # Set the pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Set the twist (velocity)
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.angular.z = angular_z

        self.odom_publisher.publish(odom)


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
