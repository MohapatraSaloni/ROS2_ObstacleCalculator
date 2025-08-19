


import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
import math

class GenericTurtleBroadcaster(Node):

    def __init__(self):
        super().__init__('generic_turtle_broadcaster')

        # Declare a parameter to accept the turtle name
        self.declare_parameter('turtlename', 'turtle1') 
        self.turtlename = self.get_parameter('turtlename').get_parameter_value().string_value

        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to the specific turtle's pose topic (e.g., /turtle1/pose or /turtle2/pose)
        self.subscription = self.create_subscription(
            Pose,
            f'/{self.turtlename}/pose',
            self.pose_callback,
            1)
        
        self.get_logger().info(f"Broadcasting TF for: {self.turtlename}")

    def pose_callback(self, msg: Pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename # Use the specific turtle name

        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        q = self.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    # (quaternion_from_euler function remains the same as before)
    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
        q = [0] * 4
        q[0] = cy * cp * sr - sy * sp * cr; q[1] = sy * cp * sr + cy * sp * cr
        q[2] = sy * cp * cr - cy * sp * sr; q[3] = cy * cp * cr + sy * sp * sr
        return q

def main(args=None):
    rclpy.init(args=args)
    # We rename the node class here
    node = GenericTurtleBroadcaster() 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
