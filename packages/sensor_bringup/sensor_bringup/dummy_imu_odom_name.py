import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class ImuOdomRenamer(Node):
    def __init__(self):
        super().__init__('imu_odom_renamer')
        
        self.subscription = self.create_subscription(
            Odometry,
            'imu/odometry',
            self.odometry_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Odometry,
            'imu/odom',
            10
        )
    
    def odometry_callback(self, msg):
        msg.header.frame_id = 'bluerov2/imu_odom'
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuOdomRenamer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()