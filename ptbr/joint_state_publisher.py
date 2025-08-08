import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        self.wheel_joint_states_sub = self.create_subscription(
            JointState,
            'joint_state/motors',
            self.save_motor_joint_state,
            10)
        self.servo_joint_states_sub = self.create_subscription(
            JointState,
            'joint_state/servos',
            self.save_servo_joint_state,
            10)
        
        self.wheel_joint_states_sub  # prevent unused variable warning
        self.servo_joint_states_sub

        self.servo_joint_states = {'name': [], 'position': [], 'velocity': []}
        self.motor_joint_states = {'name': [], 'position': [], 'velocity': []}

        self.timer = self.create_timer(0.05, self.publish_joint_states)

    def save_motor_joint_state(self, msg: JointState):
        self.motor_joint_states['name'] = msg.name
        self.motor_joint_states['position'] = msg.position
        self.motor_joint_states['velocity'] = msg.velocity

    def save_servo_joint_state(self, msg: JointState):
        self.servo_joint_states['name'] = msg.name
        self.servo_joint_states['position'] = msg.position
        self.servo_joint_states['velocity'] = msg.velocity
    
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.motor_joint_states['name'] + self.servo_joint_states['name']
        msg.position = self.motor_joint_states['position'] + self.servo_joint_states['position']
        msg.velocity = self.motor_joint_states['velocity'] + self.servo_joint_states['velocity']

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()