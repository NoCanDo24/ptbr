
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

from sensor_msgs.msg import JointState

from time import sleep, time


from ptbr.servo_driver import Servo


class JointController(Node):

    def __init__(self):
        super().__init__('motor_controller')

        self.servo_state_publisher = self.create_publisher(JointState, 'joint_state/servos', 10)
        self.publish_timer = self.create_timer(0.1, self.publish_joints)
        self.head_servo = Servo(13)
        self.head_servo.updateAngle(40)
        sleep(2)
        self.head_servo.pwm.value=0


    def publish_joints(self):

        msg = JointState()
        msg.name = ["head_joint", "right_arm_joint", "left_arm_joint"]
        msg.position = [0.0, 0.0, 0.0]
        msg.velocity = [0.0, 0.0, 0.0]

        self.servo_state_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    joint_controller = JointController()

    rclpy.spin(joint_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joint_controller.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()