import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

class DeltaTwistCmdNode(Node):

    def __init__(self):
        super().__init__('delta_twist_cmd_node')
        self.publisher = self.create_publisher(TwistStamped, '/lbr/servo_node/delta_twist_cmds', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.direction = 1  
        self.z_position = 0.0

    def timer_callback(self):
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        #msg.header.frame_id = 'lbr_link_0'  # Adjust the frame_id as necessary

        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.5 *self.direction
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.publisher.publish(msg)

        
        self.z_position += msg.twist.linear.z * 0.05
        if self.z_position > 0.3 or self.z_position < -0.3:
            self.direction *= -1

def main(args=None):
    rclpy.init(args=args)
    delta_twist_cmd_node = DeltaTwistCmdNode()
    rclpy.spin(delta_twist_cmd_node)
    delta_twist_cmd_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()