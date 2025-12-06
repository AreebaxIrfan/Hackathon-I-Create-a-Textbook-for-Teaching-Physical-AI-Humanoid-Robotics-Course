---
sidebar_position: 5
---

# ROS 2 Hands-on Lab: Basic Robot Control

This lab is designed to solidify your understanding of ROS 2 fundamentals by guiding you through building a simple robot control application. You will create nodes, publish to topics, and subscribe to sensor data to make a virtual robot move.

## Learning Objectives

*   Create new ROS 2 packages and nodes.
*   Implement a ROS 2 publisher to send velocity commands.
*   Implement a ROS 2 subscriber to receive simulated sensor data.
*   Use ROS 2 command-line tools for debugging and introspection.
*   Understand the publish/subscribe communication pattern in action.

## Prerequisites

*   A working ROS 2 environment (e.g., Foxy, Galactic, Humble) on Ubuntu.
*   Basic familiarity with C++ or Python programming.
*   (Optional but recommended) `ros_gz_sim` package installed for Gazebo integration, or a similar basic simulation environment. For this lab, we'll assume a very basic robot model is available in a simulation or you're using a visualizer like `rviz2`.

## Lab Setup

1.  **Create a ROS 2 Workspace**:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2.  **Create a New ROS 2 Package**: We'll create a package named `robot_controller` with dependencies on `rclpy` (for Python), `geometry_msgs` (for velocity messages), and `std_msgs` (for generic data).
    ```bash
    ros2 pkg create robot_controller --build-type ament_python --dependencies rclpy geometry_msgs std_msgs
    cd robot_controller
    ```

## Task 1: Implement a Velocity Publisher (Teleop Node)

We'll create a Python node that publishes `geometry_msgs/msg/Twist` messages to control the robot's linear and angular velocity.

1.  **Create `teleop_node.py` in `robot_controller/robot_controller/`**:
    ```python
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    import sys
    import tty
    import termios

    class TeleopPublisher(Node):
        def __init__(self):
            super().__init__('teleop_publisher')
            self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
            self.timer = self.create_timer(0.1, self.publish_twist) # Publish every 100ms
            self.linear_speed = 0.0
            self.angular_speed = 0.0
            self.get_logger().info('Teleop Publisher Node started. Use WASD keys to control. Press ESC to exit.')

            # Store original terminal settings
            self.settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

        def get_key(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

        def publish_twist(self):
            key = self.get_key()
            if key == '\x1b': # ESC key
                rclpy.shutdown()
                return

            if key == 'w':
                self.linear_speed = min(self.linear_speed + 0.1, 0.5)
            elif key == 's':
                self.linear_speed = max(self.linear_speed - 0.1, -0.5)
            elif key == 'a':
                self.angular_speed = min(self.angular_speed + 0.1, 0.5)
            elif key == 'd':
                self.angular_speed = max(self.angular_speed - 0.1, -0.5)
            elif key == 'x': # Stop
                self.linear_speed = 0.0
                self.angular_speed = 0.0
            
            twist_msg = Twist()
            twist_msg.linear.x = self.linear_speed
            twist_msg.angular.z = self.angular_speed
            self.publisher_.publish(twist_msg)
            self.get_logger().info(f'Publishing: Linear.x={self.linear_speed:.2f}, Angular.z={self.angular_speed:.2f}')

    def main(args=None):
        rclpy.init(args=args)
        node = TeleopPublisher()
        try:
            rclpy.spin(node)
        except SystemExit: # Catch when rclpy.shutdown() is called
            node.get_logger().info('Teleop Publisher Node exiting cleanly.')
        finally:
            node.destroy_node()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings) # Restore terminal settings

    if __name__ == '__main__':
        main()
    ```

2.  **Edit `setup.py` in `robot_controller/`**: Add an entry point for your new node.
    ```python
    from setuptools import find_packages, setup

    package_name = 'robot_controller'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/' + package_name, ['package.xml']),
            ('share/' + package_name + '/launch', ['launch/robot_launch.py']), # Example launch file
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='your_name',
        maintainer_email='your_email@example.com',
        description='Basic robot controller package',
        license='Apache-2.0',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'teleop_node = robot_controller.teleop_node:main',
            ],
        },
    )
    ```

3.  **Build your package**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select robot_controller
    ```

4.  **Source your workspace**:
    ```bash
    source install/setup.bash
    ```

5.  **Run your teleop node**:
    ```bash
    ros2 run robot_controller teleop_node
    ```
    Use 'w', 'a', 's', 'd' to control speed. 'x' to stop. 'ESC' to exit.

## Task 2: Implement a Simple Sensor Subscriber (Odometry Monitor)

We'll create a Python node that subscribes to a simulated `/odom` topic (publishing `nav_msgs/msg/Odometry` messages) and prints the robot's current position.

1.  **Create `odom_monitor.py` in `robot_controller/robot_controller/`**:
    ```python
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry

    class OdometryMonitor(Node):
        def __init__(self):
            super().__init__('odometry_monitor')
            self.subscription = self.create_subscription(
                Odometry,
                'odom',
                self.odometry_callback,
                10
            )
            self.get_logger().info('Odometry Monitor Node started. Waiting for /odom messages...')

        def odometry_callback(self, msg: Odometry):
            position_x = msg.pose.pose.position.x
            position_y = msg.pose.pose.position.y
            orientation_z = msg.pose.pose.orientation.z # Simplified for this example
            self.get_logger().info(f'Robot Position: x={position_x:.2f}, y={position_y:.2f}, Orientation: z={orientation_z:.2f}')

    def main(args=None):
        rclpy.init(args=args)
        node = OdometryMonitor()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

2.  **Edit `setup.py` again**: Add an entry point for `odom_monitor_node`.
    ```python
    from setuptools import find_packages, setup

    package_name = 'robot_controller'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/' + package_name, ['package.xml']),
            ('share/' + package_name + '/launch', ['launch/robot_launch.py']),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='your_name',
        maintainer_email='your_email@example.com',
        description='Basic robot controller package',
        license='Apache-2.0',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'teleop_node = robot_controller.teleop_node:main',
                'odom_monitor_node = robot_controller.odom_monitor:main', # New entry point
            ],
        },
    )
    ```

3.  **Rebuild and source your workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select robot_controller
    source install/setup.bash
    ```

4.  **Run the odometry monitor node**:
    ```bash
    ros2 run robot_controller odom_monitor_node
    ```

5.  **Simulate Odometry (if no actual simulator)**: You can publish a dummy odometry message from the command line while your `odom_monitor_node` is running.
    ```bash
    ros2 topic pub /odom nav_msgs/msg/Odometry '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "odom"}, child_frame_id: "base_link", pose: {pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'
    ```
    You should see your `odom_monitor_node` print the position.

## Conclusion

This lab provided a practical introduction to creating ROS 2 nodes and utilizing the publish/subscribe communication model. You've seen how to send commands to a robot and monitor its state. In the next labs, we'll integrate these concepts with more advanced sensors, actuators, and AI capabilities.