import rclpy
import tf2_ros
import tf_transformations as tf
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node

class HelperTF:
    def __init__(self, node : Node):
        self.node = node
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listen = tf2_ros.TransformListener(self.tf_buffer,self.node)

    def get(self):
        """
        return transformation in the format of
        status, X,Y,Z, r,p,y
        """
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'sasv/base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )

            quat = (
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            )
            _,_,yaw = tf.euler_from_quaternion(quat)

            translation = trans.transform.translation
            return True, translation.x, translation.y, yaw
        
        except TransformException as ex:
            self.node.get_logger().error(f"Failed to get transformation: {ex}")
            return False, 0.0, 0.0, 0.0