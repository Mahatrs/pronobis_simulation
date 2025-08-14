import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from std_msgs.msg import Header

class MoveToPickPoseDirect(Node):
    def __init__(self):
        super().__init__('move_to_pick_pose_direct')
        self.publisher = self.create_publisher(JointJog, '/lbr/servo_node/delta_joint_cmds', 10)
        self.timer = self.create_timer(0.5, self.publish_once)

    def publish_once(self):
        msg = JointJog()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.joint_names = [
            'lbr_A2', 'lbr_A3', 'lbr_A4', 'lbr_A6', 'lbr_A1', 'lbr_A5', 'lbr_A7'
        ]
        # Set velocities for each joint (rad/s)
        msg.velocities = [
            0.0,    # lbr_A2
            0.0,    # lbr_A3
            -1.1,   # lbr_A4 (move elbow)
            1.1,    # lbr_A6 (move wrist)
            0.0,    # lbr_A1
            0.0,    # lbr_A5
            0.0     # lbr_A7
        ]
        msg.displacements = []
        msg.duration = 0.0

        self.publisher.publish(msg)
        self.get_logger().info('Published direct joint velocity command')
        

def main(args=None):
    rclpy.init(args=args)
    node = MoveToPickPoseDirect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()