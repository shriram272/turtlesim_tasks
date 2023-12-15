import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import random

class TurtleCircleController(Node):
    def __init__(self, radius, max_linear_speed, max_deceleration, noise_std_dev):
        super().__init__("turtle_circle_controller")
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.real_pose_pub = self.create_publisher(Pose, '/rt_real_pose', 10)
        self.noisy_pose_pub = self.create_publisher(Pose, '/rt_noisy_pose', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.radius = radius
        self.max_linear_speed = max_linear_speed
        self.angular_speed = max_linear_speed / radius  # v = r * omega
        self.max_deceleration = max_deceleration
        self.noise_std_dev = noise_std_dev
        self.linear_speed = 0.0
        self.timer = self.create_timer(0.1, self.move_in_circle)
        self.pose_timer = self.create_timer(5.0, self.publish_pose)
        self.pose = Pose()

    def pose_callback(self, data):
        self.pose = data

    def move_in_circle(self):
        # Adjust linear speed based on deceleration limit
        velocity_change = self.max_linear_speed - self.linear_speed
        if velocity_change > 0:
            velocity_change = min(velocity_change, self.max_deceleration)
        self.linear_speed += velocity_change

        # Prepare and publish the control command
        control_command = Twist()
        control_command.linear.x = self.linear_speed
        control_command.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(control_command)

    def publish_pose(self):
        # Publish the real pose
        self.real_pose_pub.publish(self.pose)

        # Generate and publish the noisy pose
        noisy_pose = Pose()
        noisy_pose.x = self.pose.x + random.gauss(0, self.noise_std_dev)
        noisy_pose.y = self.pose.y + random.gauss(0, self.noise_std_dev)
        noisy_pose.theta = self.pose.theta + random.gauss(0, self.noise_std_dev)
        self.noisy_pose_pub.publish(noisy_pose)

def main(args=None):
    rclpy.init(args=args)

    radius = 4.5  # Radius of the circle
    max_linear_speed = 1.5  # Max linear speed
    max_deceleration = 0.05  # Deceleration rate
    noise_std_dev = 3.0  # Noise standard deviation

    turtle_circle_controller = TurtleCircleController(radius, max_linear_speed, max_deceleration, noise_std_dev)

    try:
        rclpy.spin(turtle_circle_controller)
    except KeyboardInterrupt:
        pass

    turtle_circle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
