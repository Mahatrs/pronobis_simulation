import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from std_msgs.msg import Header

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher = self.create_publisher(JointJog, '/lbr/servo_node/delta_joint_cmds', 10)
        self.timer = self.create_timer(0.1, self.publish_command)  # 10 Hz
        self.count = 0
        self.max_count = 50  # Number of cycles per direction (adjust for angle)
        self.direction = 1   # 1 for right, -1 for left

    def publish_command(self):
        msg = JointJog()
        #msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.joint_names = ['lbr_A6']
        msg.velocities = [1.0 * self.direction]  # Small velocity, change direction


        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing joint command: {msg}')

        self.count += 1
        if self.count >= self.max_count:
            self.direction *= -1  # Reverse direction
            self.count = 0

def main(args=None):
    rclpy.init(args=args)
    joint_command_publisher = JointCommandPublisher()
    rclpy.spin(joint_command_publisher)
    joint_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
