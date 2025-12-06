---
sidebar_position: 5
---

# Digital Twin Lab: Simulated Humanoid Control

This lab consolidates your understanding of digital twins by guiding you through a practical exercise of controlling a simulated humanoid robot. You will leverage ROS 2 to send commands and receive sensor feedback from a robot in a simulation environment (Gazebo or Isaac Sim, depending on setup).

## Learning Objectives

*   Launch a simulated humanoid robot in a Gazebo or Isaac Sim environment.
*   Control the simulated robot's joints using ROS 2 topics.
*   Read simulated sensor data (e.g., joint states, IMU) from ROS 2 topics.
*   Implement a simple control script to achieve a basic humanoid pose or movement.
*   Utilize ROS 2 visualization tools (e.g., `rviz2`) to monitor robot state.

## Prerequisites

*   A working ROS 2 environment (e.g., Humble) on Ubuntu.
*   Gazebo or NVIDIA Isaac Sim (with ROS 2 bridge configured) installed.
*   Basic understanding of ROS 2 nodes, topics, services (from Module 1).
*   Basic Python programming skills.
*   Familiarity with URDF/SDF robot models.

## Lab Setup

1.  **Ensure ROS 2 Workspace is Sourced**:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```
2.  **Clone Example Humanoid Robot Repository**:
    For this lab, we'll use a simplified humanoid robot model compatible with ROS 2 and Gazebo. If using Isaac Sim, ensure you have an equivalent model.
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/ros-simulation/gazebo_ros_demos.git -b foxy # Example, adjust branch for your ROS 2 version
    # Or, if you have a specific humanoid model:
    # git clone <your_humanoid_robot_repo>.git
    ```
3.  **Build Workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```

## Task 1: Launching the Simulated Humanoid

We will launch a pre-configured Gazebo world with a humanoid robot.

1.  **Launch Gazebo with the Humanoid Model**:
    *   If using `gazebo_ros_demos` (e.g., for `rrbot`):
        ```bash
        ros2 launch gazebo_ros_demos rrbot_description gazebo.launch.py
        ```
    *   If you have another humanoid model, refer to its launch instructions. For Isaac Sim, launch Isaac Sim and load your humanoid asset, then ensure ROS 2 bridge is active.

2.  **Verify Robot in Simulation**:
    Open the Gazebo GUI or Isaac Sim GUI. You should see a humanoid robot model loaded in the environment.

3.  **Check ROS 2 Topics**:
    Open a new terminal and list ROS 2 topics to see available interfaces:
    ```bash
    ros2 topic list
    ```
    Look for topics related to joint commands (e.g., `/joint_commands`) and joint states (e.g., `/joint_states`).

## Task 2: Controlling Humanoid Joints

We will publish messages to the robot's joint command topic to control its posture.

1.  **Identify Joint Command Topic and Message Type**:
    Typically, this might be a `std_msgs/msg/Float64MultiArray` or a custom message type, depending on the robot's controller configuration. For simplicity, assume a generic `Float64MultiArray` for joint positions.

2.  **Publish Joint Commands (Example: C++ Node)**:
    Create a C++ node `joint_commander_node.cpp` in a new package (e.g., `humanoid_control`) within your ROS 2 workspace.
    ```cpp
    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/float64_multi_array.hpp"

    class JointCommander : public rclcpp::Node {
    public:
        JointCommander() : Node("joint_commander") {
            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_group_position_controller/commands", 10);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500), std::bind(&JointCommander::publish_commands, this));
            RCLCPP_INFO(this->get_logger(), "Joint Commander Node started.");
        }

    private:
        void publish_commands() {
            auto message = std_msgs::msg::Float64MultiArray();
            // Example: set two joints to specific positions (radians)
            message.data.push_back(0.5);  // Joint 1 position
            message.data.push_back(-0.5); // Joint 2 position
            // ... add more joint positions as per your robot's configuration

            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Published joint commands.");
        }

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
    };

    int main(int argc, char * argv[]) {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<JointCommander>());
        rclcpp::shutdown();
        return 0;
    }
    ```
    *Note: The topic name `/joint_group_position_controller/commands` is common but may vary based on your robot's controller setup. Adjust as necessary.*

3.  **Build and Run**: Compile your C++ package and run the node.
    You should see your simulated humanoid robot in Gazebo/Isaac Sim move to the specified joint positions.

## Task 3: Monitoring Humanoid Joint States

We will subscribe to the robot's joint state topic to monitor its current joint positions.

1.  **Identify Joint State Topic and Message Type**:
    Typically, `/joint_states` publishing `sensor_msgs/msg/JointState` messages.

2.  **Subscribe to Joint States (Example: Python Node)**:
    Create a Python node `joint_state_monitor.py` in your `humanoid_control` package.
    ```python
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    class JointStateMonitor(Node):
        def __init__(self):
            super().__init__('joint_state_monitor')
            self.subscription = self.create_subscription(
                JointState,
                'joint_states',
                self.joint_state_callback,
                10
            )
            self.get_logger().info('Joint State Monitor Node started. Waiting for /joint_states messages...')

        def joint_state_callback(self, msg: JointState):
            self.get_logger().info(f'Received Joint States:')
            for i in range(len(msg.name)):
                self.get_logger().info(f'  {msg.name[i]}: Position={msg.position[i]:.2f}, Velocity={msg.velocity[i]:.2f}')

    def main(args=None):
        rclpy.init(args=args)
        node = JointStateMonitor()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

3.  **Build and Run**: Compile your Python package (if it's part of a mixed C++/Python package, ensure `colcon build` handles both) and run the node.
    You should see the joint positions of your simulated humanoid being printed in the terminal.

## Task 4: Visualization with `rviz2`

`rviz2` is a 3D visualization tool for ROS 2. It's invaluable for visualizing robot models, sensor data, and planning outputs.

1.  **Launch `rviz2`**:
    ```bash
    rviz2
    ```
2.  **Add Robot Model**:
    In `rviz2`, click "Add" -> "RobotModel" and ensure the correct `robot_description` is loaded (usually published on the `/robot_description` topic).
3.  **Add Joint State Display**:
    Add a "JointState" display type and set its topic to `/joint_states`. You should see the robot model in `rviz2` update as you publish commands.

## Conclusion

This lab provided a hands-on experience in controlling and monitoring a simulated humanoid robot using ROS 2 in a digital twin environment. You've learned how to interact with joint commands and states, and visualize the robot's behavior. This foundation is critical for developing more complex autonomous behaviors and integrating advanced AI capabilities.