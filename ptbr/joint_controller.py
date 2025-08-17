
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

        self.declare_parameter('upright_state', True)
        self.last_state = True

        self.servo_state_publisher = self.create_publisher(JointState, 'joint_state/servos', 10)
        self.publish_timer = self.create_timer(0.5, self.publish_joints)

        self.head_servo = Servo(13)
        self.right_arm = Servo(17, min_cycle=1, max_cycle=2, angle_range=180)
        self.left_arm = Servo(27, min_cycle=1, max_cycle=2, angle_range=180)

        self.update_joints(True)


    def publish_joints(self):
        upright = self.get_parameter('upright_state').get_parameter_value().bool_value
        if upright and not self.last_state:
            self.update_joints(True)
        elif not upright and self.last_state:
            self.update_joints(False)

        self.last_state = upright


        msg = JointState()
        msg.name = ["head_joint", "right_arm_joint", "left_arm_joint"]
        msg.position = [0.1, 0.0, 0.0]
        msg.velocity = [0.0, 0.0, 0.0]

        self.servo_state_publisher.publish(msg)

    def update_joints(self, upright):
        if upright:
            self.head_servo.updateAngle(10)
            sleep(0.5)
            self.right_arm.updateAngle(0)
            self.left_arm.updateAngle(180)
        else:
            self.right_arm.updateAngle(160)
            self.left_arm.updateAngle(50)
            sleep(0.5)
            self.head_servo.updateAngle(90)
        
        sleep(1)
        self.reset_pwm()

    def reset_pwm(self):
        self.head_servo.pwm.value=0
        self.right_arm.pwm.value=0
        self.left_arm.pwm.value=0



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