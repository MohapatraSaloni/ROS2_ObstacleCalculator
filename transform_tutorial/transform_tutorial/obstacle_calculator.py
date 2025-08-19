import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster, TransformException
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
import math

class TurtleTransformer(Node):

    def __init__(self):
        super().__init__('turtle_transformer_node')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.obstacle_broadcaster = TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            1)
        
        self.get_logger().info("Turtle Transformer node has started.")
        self.get_logger().info("Drive the turtle using the 'turtle_teleop_key' node.")
        self.get_logger().info("View frames in RViz2 by setting the Fixed Frame to 'world'.")

    def pose_callback(self, msg: Pose):
        self.broadcast_turtle_frame(msg)
        self.broadcast_obstacle_frame()

    def broadcast_turtle_frame(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'turtle1'

        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        q = self.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def broadcast_obstacle_frame(self):
        obs_t = TransformStamped()
        obs_t.header.stamp = self.get_clock().now().to_msg()
        obs_t.header.frame_id = 'turtle1'
        obs_t.child_frame_id = 'obstacle_frame'

        obs_t.transform.translation.x = 1.0 
        obs_t.transform.translation.y = 0.0
        obs_t.transform.translation.z = 0.0

        obs_t.transform.rotation.x = 0.0
        obs_t.transform.rotation.y = 0.0
        obs_t.transform.rotation.z = 0.0
        obs_t.transform.rotation.w = 1.0
        
        self.obstacle_broadcaster.sendTransform(obs_t)

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = [0] * 4
        q[0] = cy * cp * sr - sy * sp * cr
        q[1] = sy * cp * sr + cy * sp * cr
        q[2] = sy * cp * cr - cy * sp * sr
        q[3] = cy * cp * cr + sy * sp * sr
        return q

def main(args=None):
    rclpy.init(args=args)
    node = TurtleTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster, TransformException
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
import math

class TurtleTransformer(Node):

    def __init__(self):
        super().__init__('turtle_transformer_node')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.obstacle_broadcaster = TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            1)
        
        self.get_logger().info("Turtle Transformer node has started.")
        self.get_logger().info("Drive the turtle using the 'turtle_teleop_key' node.")
        self.get_logger().info("View frames in RViz2 by setting the Fixed Frame to 'world'.")

    def pose_callback(self, msg: Pose):
        self.broadcast_turtle_frame(msg)
        self.broadcast_obstacle_frame()

    def broadcast_turtle_frame(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'turtle1'

        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        q = self.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def broadcast_obstacle_frame(self):
        obs_t = TransformStamped()
        obs_t.header.stamp = self.get_clock().now().to_msg()
        obs_t.header.frame_id = 'turtle1'
        obs_t.child_frame_id = 'obstacle_frame'

        obs_t.transform.translation.x = 1.0 
        obs_t.transform.translation.y = 0.0
        obs_t.transform.translation.z = 0.0

        obs_t.transform.rotation.x = 0.0
        obs_t.transform.rotation.y = 0.0
        obs_t.transform.rotation.z = 0.0
        obs_t.transform.rotation.w = 1.0
        
        self.obstacle_broadcaster.sendTransform(obs_t)

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = [0] * 4
        q[0] = cy * cp * sr - sy * sp * cr
        q[1] = sy * cp * sr + cy * sp * cr
        q[2] = sy * cp * cr - cy * sp * sr
        q[3] = cy * cp * cr + sy * sp * sr
        return q

def main(args=None):
    rclpy.init(args=args)
    node = TurtleTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
