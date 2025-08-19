import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from geometry_msgs.msg import Twist
import math

class TurtleFollower(Node):
    def __init__(self):
        super().__init__('turtle_follower')

        self.leader_frame = 'turtle1'
        self.follower_frame = 'turtle2'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for the FOLLOWER's velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            f'/{self.follower_frame}/cmd_vel', 
            10
        )

        self.control_timer = self.create_timer(0.1, self.follow_loop)
        self.get_logger().info(f"Follower node started: {self.follower_frame} is chasing {self.leader_frame}")

    def follow_loop(self):
        try:
            # === THE CORE OF TF2 USAGE ===
            # We ask: "What is the transform FROM the follower TO the leader?"
            # This tells us exactly where the leader is relative to the follower.
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.follower_frame, # Source frame (where we are)
                self.leader_frame,   # Target frame (where we want to go)
                now)
        except TransformException as ex:
            self.get_logger().info(f'Could not get transform: {ex}')
            return

        # Extract the relative position (translation)
        rel_x = trans.transform.translation.x
        rel_y = trans.transform.translation.y

        # --- Control Law (P-Controller) ---
        # 1. Angular Velocity: How much do we need to turn?
        # atan2(y, x) gives the angle needed to face the leader.
        # Since this transform is already relative to the follower, this is our angular error.
        angular_error = math.atan2(rel_y, rel_x)
        Kp_angular = 3.0
        angular_velocity = Kp_angular * angular_error

        # 2. Linear Velocity: How fast should we move forward?
        # Calculate the distance to the leader.
        distance_error = math.sqrt(rel_x**2 + rel_y**2)
        Kp_linear = 0.8
        linear_velocity = Kp_linear * distance_error

        # --- Publish Command ---
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        
        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
