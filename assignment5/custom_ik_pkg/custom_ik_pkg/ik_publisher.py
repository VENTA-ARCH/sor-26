import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class CustomIKPublisher(Node):
    def __init__(self):
        super().__init__('custom_ik_publisher')
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        self.timer = self.create_timer(0.5, self.publish_trajectory)
        self.sent = False

    def publish_trajectory(self):
        if self.sent:
            return

        # Wait until the controller has actually subscribed
        if self.publisher_.get_subscription_count() == 0:
            self.get_logger().info('Waiting for subscriber on /arm_controller/joint_trajectory...')
            return

        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_joint',
            'left_finger_joint',
            'right_finger_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.588, 0.182, 0.674, -0.856, 0.0, 0.0]
        point.time_from_start = Duration(sec=3, nanosec=0)

        msg.points.append(point)
        print("DEBUG MSG:", msg)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published joint trajectory: {point.positions}')
        self.sent = True
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = CustomIKPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
