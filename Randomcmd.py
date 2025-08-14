import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import random
import math

class RandomDeltaTwistCmdNode(Node):
    def __init__(self):
        super().__init__('random_delta_twist_cmd_node')
        self.publisher = self.create_publisher(TwistStamped, '/lbr/servo_node/delta_twist_cmds', 10)
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.max_distance = 0.3
        self.step_magnitude = 0.12

        self.position = [0.0, 0.0, 0.0]
        self.velocity = self.generate_random_velocity()

        self.paused = False
        self.pause_counter = 0
        self.pause_duration = int(0.5 / self.timer_period)

    def generate_random_velocity(self):
        to_center = [-p for p in self.position]
        norm = math.sqrt(sum(x**2 for x in to_center)) + 1e-6
        to_center = [x / norm for x in to_center]

        random_offset = [random.uniform(-0.4, 0.4) for _ in range(3)]
        direction = [to_center[i] + random_offset[i] for i in range(3)]

        norm = math.sqrt(sum(x**2 for x in direction)) + 1e-6
        direction = [x / norm for x in direction]

        return [x * self.step_magnitude for x in direction]

    def timer_callback(self):
        if self.paused:
            self.pause_counter += 1
            if self.pause_counter >= self.pause_duration:
                self.paused = False
                self.pause_counter = 0
                self.velocity = self.generate_random_velocity()
            velocity = [0.0, 0.0, 0.0] 
        else:
            for i in range(3):
                self.position[i] += self.velocity[i] * self.timer_period
                if abs(self.position[i]) > self.max_distance:
                    self.paused = True
                    break
            velocity = self.velocity
        
        self.get_logger().info(
        f"\n"
        f"Position  : x={self.position[0]:.3f}, y={self.position[1]:.3f}, z={self.position[2]:.3f}\n"
        f"Velocity  : x={velocity[0]:.3f}, y={velocity[1]:.3f}, z={velocity[2]:.3f}"
    )

        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.twist.linear.x = velocity[0]
        msg.twist.linear.y = velocity[1]
        msg.twist.linear.z = velocity[2]
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RandomDeltaTwistCmdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
