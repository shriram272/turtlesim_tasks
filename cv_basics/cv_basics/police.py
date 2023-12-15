import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class ChasingTurtleController(Node):
    def __init__(self, chase_distance):
        super().__init__("chasing_turtle_controller")
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle2/pose', self.pose_callback, 10)
        self.target_pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.target_pose_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.pose = Pose()
        self.target_pose = Pose()
        self.kp = 1.0  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.01 # Derivative gain
        self.prev_error = 0.0
        self.integral = 0.0
        self.chase_distance = chase_distance

    def pose_callback(self, data):
        self.pose = data

    def target_pose_callback(self, data):
        self.target_pose = data

    def control_loop(self):
        distance_to_goal = math.sqrt((self.target_pose.x - self.pose.x)**2 + (self.target_pose.y - self.pose.y)**2)
        
        # Check if chase is complete
        if distance_to_goal <= self.chase_distance:
            self.cmd_vel_pub.publish(Twist())  # Stop moving
            self.get_logger().info('Chase Complete')
            return

        angle_to_goal = math.atan2(self.target_pose.y - self.pose.y, self.target_pose.x - self.pose.x)
        angle_tolerance = 0.5
        angle_error = angle_to_goal - self.pose.theta

        # PID calculations
        self.integral += angle_error
        derivative = angle_error - self.prev_error
        angular_velocity = self.kp * angle_error + self.ki * self.integral + self.kd * derivative

        new_vel = Twist()
        if abs(angle_error) > angle_tolerance:
            new_vel.angular.z = angular_velocity
        else:
            self.integral = 0.0  # Reset integral term
            new_vel.linear.x = 2.5  # Constant speed

        self.cmd_vel_pub.publish(new_vel)
        self.prev_error = angle_error

def main(args=None):
    rclpy.init(args=args)
    chasing_turtle_controller = ChasingTurtleController(chase_distance=1.5)

    try:
        rclpy.spin(chasing_turtle_controller)
    except KeyboardInterrupt:
        pass

    chasing_turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
