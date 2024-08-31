'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class MoveRobot(Node):

    def __init__(self):
        super().__init__('move_robot')
        self.subscription = self.create_subscription(
            Odometry,
            '/mecanum2/odom1',
            self.odom_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/mecanum2/cmd_vel', 10)
        self.odom_data = None
        self.start_time = None
        self.move_duration = 15.0  # Duration for movement in seconds
        #10秒だと約４５度、4秒くらい？？
        # self.linear_speed = -0.1  # Adjust the linear velocity as needed
        self.shutdown_timer = None  # Timer to shutdown the node

    def odom_callback(self, msg):
        self.odom_data = msg
        self.move_x()  # Call move_x() whenever new odometry data is received

    def move_x(self):
        if self.start_time is None:
            self.start_time = self.get_clock().now().to_msg()

        if self.odom_data:
            twist = Twist()
            #twist.linear.y = 0.1
            #twist.angular.z = -0.1
            #もともと
            twist.linear.y = 0.03
            twist.angular.z = -0.03

            # Calculate the time elapsed
            current_time = self.get_clock().now().to_msg()
            elapsed_time = (current_time.sec - self.start_time.sec) + (current_time.nanosec - self.start_time.nanosec) / 1e9

            # Move for specified duration
            if elapsed_time <= self.move_duration:
                self.publisher_.publish(twist)
                self.get_logger().info('Moving...')
            else:
                # Stop after specified duration
                stop_twist = Twist()
                self.publisher_.publish(stop_twist)
                self.get_logger().info('Stopped moving')
                # Shutdown the node after the movement
                self.shutdown_node()

    def shutdown_node(self):
        self.get_logger().info('Shutting down node...')
        self.destroy_node()
        rclpy.shutdown()




#####mainのプログラム###########################################
def main(args=None):
    rclpy.init(args=args)
    move_robot = MoveRobot()
    try:
        rclpy.spin(move_robot)
    except KeyboardInterrupt:
        pass
    finally:
        move_robot.shutdown_node()

if __name__ == '__main__':
    main()
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import subprocess  # 追加
import os #追加
import time  # 追加

#一定時間円上で動いて、D-optimality(kameyama/galactic_ws/src/cuboid_analisis)を計算する
'''
class MoveRobot(Node):

    def __init__(self):
        super().__init__('move_robot')
        self.subscription = self.create_subscription(
            Odometry,
            '/mecanum2/odom1',
            self.odom_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/mecanum2/cmd_vel', 10)
        self.odom_data = None
        self.start_time = None
        self.move_duration = 15.0  # Duration for movement in seconds
        self.shutdown_timer = None  # Timer to shutdown the node

    def odom_callback(self, msg):
        self.odom_data = msg
        self.move_x()  # Call move_x() whenever new odometry data is received

    def move_x(self):
        if self.start_time is None:
            self.start_time = self.get_clock().now().to_msg()

        if self.odom_data:
            twist = Twist()
            twist.linear.y = 0.03
            twist.angular.z = -0.03

            # Calculate the time elapsed
            current_time = self.get_clock().now().to_msg()
            elapsed_time = (current_time.sec - self.start_time.sec) + (current_time.nanosec - self.start_time.nanosec) / 1e9

            # Move for specified duration
            if elapsed_time <= self.move_duration:
                self.publisher_.publish(twist)
                self.get_logger().info('Moving...')
            else:
                # Stop after specified duration
                stop_twist = Twist()
                self.publisher_.publish(stop_twist)
                self.get_logger().info('Stopped moving')
                # Shutdown the node after the movement and start launch file
                self.shutdown_node_and_launch()

    def shutdown_node_and_launch(self):
        # Run the launch file before shutting down the node
        self.get_logger().info('Starting cuboid_analysis.launch...')
        subprocess.run(['ros2', 'launch', 'cuboid_analysis', 'cuboid_analysis_launch.py'])
    
        self.get_logger().info('Shutting down node...')
        self.destroy_node()
        rclpy.shutdown()
'''

class MoveRobot(Node):

    def __init__(self):
        super().__init__('move_robot')
        self.subscription = self.create_subscription(
            Odometry,
            '/mecanum2/odom1',
            self.odom_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/mecanum2/cmd_vel', 10)
        self.odom_data = None
        self.start_time = None
        self.move_duration = 15.0  # メカナムが動く時間
        self.stop_duration = 30.0  # CubeSLAMで情報取得する待機時間（1000データ取れば良い）
        self.stop_threshold = 4.0  # D-optimalityの閾値
        self.state = 'moving'  # State of the robot
        self.process = None  # To store the cuboid_analysis process

    def odom_callback(self, msg):
        self.odom_data = msg
        if self.state == 'moving':
            self.move_x()
        elif self.state == 'stopping':
            self.stop_for_duration()

    def move_x(self):
        if self.start_time is None:
            self.start_time = self.get_clock().now().to_msg()

        if self.odom_data:
            twist = Twist()
            twist.linear.y = 0.03
            twist.angular.z = -0.03

            # Calculate the time elapsed
            current_time = self.get_clock().now().to_msg()
            elapsed_time = (current_time.sec - self.start_time.sec) + (current_time.nanosec - self.start_time.nanosec) / 1e9

            # Move for specified duration
            if elapsed_time <= self.move_duration:
                self.publisher_.publish(twist)
                self.get_logger().info('Moving...')
            else:
                # Stop after specified duration
                self.stop_robot()
                self.state = 'stopping'  # Change state to stopping
                self.start_time = self.get_clock().now().to_msg()  # Reset start time for stopping

    def stop_for_duration(self):
        current_time = self.get_clock().now().to_msg()
        elapsed_time = (current_time.sec - self.start_time.sec) + (current_time.nanosec - self.start_time.nanosec) / 1e9

        if elapsed_time <= self.stop_duration:
            self.get_logger().info('Stopping...')
        else:
            self.get_logger().info('Stopped for 30 seconds. Starting cuboid_analysis...')
            self.start_cuboid_analysis()

    def start_cuboid_analysis(self):
        # Launch the cuboid_analysis and wait for it to complete
        try:
            self.process = subprocess.Popen(['ros2', 'launch', 'cuboid_analysis', 'cuboid_analysis_launch.py'])
            self.process.wait()  # Wait for the process to complete
            self.get_logger().info('cuboid_analysis finished. Checking D-optimal value...')
        except Exception as e:
            self.get_logger().error(f'Error occurred while running cuboid_analysis: {e}')
        finally:
            self.check_d_optimal_value_and_continue()

    def check_d_optimal_value_and_continue(self):
        # Check d-optimal value and act accordingly
        if self.check_d_optimal_value():
            self.get_logger().info('D-optimal value exceeded threshold. Stopping.')
            # Optionally, shutdown the node or perform other actions
        else:
            self.get_logger().info('D-optimal value within threshold. Continuing movement.')
            self.start_time = self.get_clock().now().to_msg()  # Reset start time for new movement cycle
            self.state = 'moving'  # Change state back to moving
            self.move_duration = 15.0  # Reset move duration

    def check_d_optimal_value(self):
        # Read d_optimal_value from file
        home_dir = os.path.expanduser("~")
        value_file = os.path.join(home_dir, '.ros', 'd_optimal_value.txt')
        if os.path.exists(value_file):
            with open(value_file, 'r') as f:
                d_optimal_value = float(f.read().strip())
                return d_optimal_value > self.stop_threshold
        else:
            self.get_logger().error('d_optimal_value file not found.')
            return False

    def stop_robot(self):
        stop_twist = Twist()
        self.publisher_.publish(stop_twist)
        self.get_logger().info('Stopped robot.')

#### mainプログラム
def main(args=None):
    rclpy.init(args=args)
    move_robot = MoveRobot()
    try:
        rclpy.spin(move_robot)
    except KeyboardInterrupt:
        move_robot.get_logger().info('Node interrupted by user.')
        move_robot.stop_robot()  # Stop the robot
        if move_robot.process and move_robot.process.poll() is None:
            move_robot.process.terminate()  # Terminate the cuboid_analysis process if it's still running
    finally:
        if rclpy.ok():
            move_robot.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()