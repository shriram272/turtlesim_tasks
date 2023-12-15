import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class MovingAverageFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.values = []

    def update(self, new_value):
        self.values.append(new_value)
        if len(self.values) > self.window_size:
            self.values.pop(0)
        return sum(self.values) / len(self.values)

class TurtleChaser(Node):
    def __init__(self):
        super().__init__('turtle_chaser')

        # Parameters
        self.declare_parameter('target_turtle', 'turtle1')
        self.declare_parameter('chaser_turtle', 'turtle2')
        self.declare_parameter('time_ahead', 3.0)
        self.declare_parameter('angular_velocity_gain', 1.5)
        self.declare_parameter('min_distance', 1.5)

        # Retrieve parameters
        target_turtle = self.get_parameter('target_turtle').get_parameter_value().string_value
        chaser_turtle = self.get_parameter('chaser_turtle').get_parameter_value().string_value
        self.time_ahead = self.get_parameter('time_ahead').get_parameter_value().double_value
        self.angular_velocity_gain = self.get_parameter('angular_velocity_gain').get_parameter_value().double_value
        self.min_distance = self.get_parameter('min_distance').get_parameter_value().double_value

        # Subscribers and Publishers
        self.target_pose_subscriber = self.create_subscription(Pose, '/rt_noisy_pose', self.update_target_pose, 10)
        self.chaser_pose_subscriber = self.create_subscription(Pose, f'/{chaser_turtle}/pose', self.update_chaser_pose, 10)
        self.chaser_velocity_publisher = self.create_publisher(Twist, f'/{chaser_turtle}/cmd_vel', 10)

        # Pose storage and filters
        self.target_pose = Pose()
        self.chaser_pose = Pose()
        self.last_target_pose = Pose()
        self.x_filter = MovingAverageFilter()
        self.y_filter = MovingAverageFilter()
        self.theta_filter = MovingAverageFilter()

        # Timer
        self.timer = self.create_timer(0.01, self.chase)

    def update_target_pose(self, msg):
        filtered_x = self.x_filter.update(msg.x)
        filtered_y = self.y_filter.update(msg.y)
        filtered_theta = self.theta_filter.update(msg.theta)
        self.last_target_pose = self.target_pose
        self.target_pose.x = filtered_x
        self.target_pose.y = filtered_y
        self.target_pose.theta = filtered_theta

    def update_chaser_pose(self, msg):
        self.chaser_pose = msg

    def calculate_target_orientation_change(self):
        dtheta = self.target_pose.theta - self.last_target_pose.theta
        return dtheta

    def predict_future_orientation(self, dtheta):
        future_orientation = self.target_pose.theta + dtheta * self.time_ahead
        return future_orientation

    def calculate_target_speed(self):
        dx = self.target_pose.x - self.last_target_pose.x
        dy = self.target_pose.y - self.last_target_pose.y
        return math.sqrt(dx**2 + dy**2) / 0.01

    def predict_future_position_with_orientation(self, future_orientation):
        speed = self.calculate_target_speed()
        future_x = self.target_pose.x + speed * self.time_ahead * math.cos(future_orientation)
        future_y = self.target_pose.y + speed * self.time_ahead * math.sin(future_orientation)
        return future_x, future_y

    def chase(self):
     dtheta = self.calculate_target_orientation_change()
     future_orientation = self.predict_future_orientation(dtheta)
     future_x, future_y = self.predict_future_position_with_orientation(future_orientation)

     vel_msg = Twist()
     distance = math.sqrt((future_x - self.chaser_pose.x) ** 2 + (future_y - self.chaser_pose.y) ** 2)

    # Normalize angle to target
     angle_to_target = math.atan2(future_y - self.chaser_pose.y, future_x - self.chaser_pose.x)
     angle_diff = (angle_to_target - self.chaser_pose.theta + math.pi) % (2 * math.pi) - math.pi

     self.get_logger().info(f'Current distance: {distance}, Angle diff: {angle_diff}') 

     if distance <= self.min_distance:
        self.get_logger().info('Chase Complete')
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
     else:
        vel_msg.angular.z = self.angular_velocity_gain * angle_diff
        vel_msg.linear.x = min(3 * distance, 2.0)  # Limit max speed

     self.chaser_velocity_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    turtle_chaser = TurtleChaser()
    rclpy.spin(turtle_chaser)
    turtle_chaser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
