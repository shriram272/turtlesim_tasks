import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleChaser(Node):
    def __init__(self):
        super().__init__('turtle_chaser')

        # Parameters
        self.declare_parameter('target_turtle', 'turtle1')
        self.declare_parameter('chaser_turtle', 'turtle2')
        self.declare_parameter('time_ahead', 2.0)  # Time ahead for prediction
        self.declare_parameter('angular_velocity_gain', 1.5)  # Gain for angular velocity
        self.declare_parameter('min_distance', 2.5)  # Minimum stopping distance

        # Retrieve parameters
        target_turtle = self.get_parameter('target_turtle').get_parameter_value().string_value
        chaser_turtle = self.get_parameter('chaser_turtle').get_parameter_value().string_value
        self.time_ahead = self.get_parameter('time_ahead').get_parameter_value().double_value
        self.angular_velocity_gain = self.get_parameter('angular_velocity_gain').get_parameter_value().double_value
        self.min_distance = self.get_parameter('min_distance').get_parameter_value().double_value

        # Subscribers and Publishers
        self.target_pose_subscriber = self.create_subscription(Pose, f'/{target_turtle}/pose', self.update_target_pose, 10)
        self.chaser_pose_subscriber = self.create_subscription(Pose, f'/{chaser_turtle}/pose', self.update_chaser_pose, 10)
        self.chaser_velocity_publisher = self.create_publisher(Twist, f'/{chaser_turtle}/cmd_vel', 10)

        # Pose storage
        self.target_pose = Pose()
        self.chaser_pose = Pose()
        self.last_target_pose = Pose()

        # Timer
        self.timer = self.create_timer(0.01, self.chase)

    def update_target_pose(self, msg):
        self.last_target_pose = self.target_pose
        self.target_pose = msg

    def update_chaser_pose(self, msg):
        self.chaser_pose = msg

    def calculate_target_orientation_change(self):
        # Calculate change in orientation
        dtheta = self.target_pose.theta - self.last_target_pose.theta
        return dtheta

    def predict_future_orientation(self, dtheta):
        # Predict future orientation based on the rate of change
        future_orientation = self.target_pose.theta + dtheta * self.time_ahead
        return future_orientation

    def calculate_target_speed(self):
        dx = self.target_pose.x - self.last_target_pose.x
        dy = self.target_pose.y - self.last_target_pose.y
        return math.sqrt(dx**2 + dy**2) / 0.01  # Assuming 0.01 is the timer callback interval

    def predict_future_position_with_orientation(self, future_orientation):
        # Predict future position considering orientation change
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
        #self.get_logger().info(f'Current distance: {distance}') 
        direct_distance = math.sqrt((self.target_pose.x - self.chaser_pose.x) ** 2 + (self.target_pose.y - self.chaser_pose.y) ** 2)
        if direct_distance <= self.min_distance:
            # Stop the turtle if it's within the minimum distance
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.get_logger().info('Chase Complete')
        else:
            # Continue chasing
            angle_to_target = math.atan2(future_y - self.chaser_pose.y, future_x - self.chaser_pose.x)
            vel_msg.angular.z = self.angular_velocity_gain * (angle_to_target - self.chaser_pose.theta)
            vel_msg.linear.x = min(5 * distance, 1.5)  # Limit max speed

        self.chaser_velocity_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    turtle_chaser = TurtleChaser()
    rclpy.spin(turtle_chaser)
    turtle_chaser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
