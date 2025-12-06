---
sidebar_position: 5
---

# Isaac ROS Hands-on Lab: Object Detection and Grasping

This lab will guide you through building a simple AI-powered robotics application using NVIDIA Isaac ROS. You will learn to integrate GPU-accelerated perception for object detection and set up a basic manipulation task, either in a simulated environment (Isaac Sim) or on a physical Jetson-powered robot.

## Learning Objectives

*   Set up an Isaac ROS development environment.
*   Integrate an Isaac ROS object detection pipeline.
*   Visualize object detections using `rviz2`.
*   Implement a simple grasping strategy based on detected object poses.
*   Understand the workflow from perception to action using Isaac ROS.

## Prerequisites

*   A working ROS 2 environment (e.g., Humble) on Ubuntu.
*   NVIDIA Jetson device (e.g., Jetson Nano, Orin Nano/NX) with JetPack installed, OR
*   NVIDIA Isaac Sim with ROS 2 bridge configured.
*   Basic understanding of ROS 2 nodes, topics, and services.
*   Basic Python or C++ programming skills.
*   Familiarity with Docker (recommended for Isaac ROS setup).

## Lab Setup (Jetson or Docker/Isaac Sim)

### Option A: Jetson Device Setup

1.  **Install Isaac ROS**: Follow the official NVIDIA Isaac ROS documentation to install the required packages and dependencies on your Jetson device. This typically involves using Docker containers.
2.  **Clone Example Workspaces**: Clone the `isaac_ros_common` and `isaac_ros_tutorials` (if available) repositories into your ROS 2 workspace.
3.  **Build and Source**: Build your workspace and source the setup files.

### Option B: Docker Container / Isaac Sim Setup

1.  **Pull Isaac ROS Docker Image**:
    ```bash
    docker pull nvcr.io/nvidia/isaac-ros/isaac_ros_dev:latest
    ```
2.  **Run Development Container**:
    ```bash
    # For a general dev container
    ./scripts/run_dev.sh
    # For Isaac Sim integration, refer to Isaac Sim ROS 2 documentation
    ```
3.  **Launch Isaac Sim**: If using Isaac Sim, launch it and ensure the ROS 2 bridge is enabled.

## Task 1: Object Detection with Isaac ROS

We will use an Isaac ROS package for object detection, typically `isaac_ros_detectnet` or a similar package that leverages `TensorRT` for acceleration.

1.  **Launch an Object Detection Node**:
    *   **In Isaac Sim**: Launch Isaac Sim with a scene containing objects. Ensure the ROS 2 bridge is publishing camera images (e.g., on `/front_stereo_camera/left/image_rect_color`).
    *   **On Jetson**: If using a physical camera, ensure your camera driver is publishing images to a ROS 2 topic.
    *   Launch the Isaac ROS DetectNet node (example command, may vary):
        ```bash
        # Inside your Isaac ROS Docker container or Jetson
        ros2 launch isaac_ros_detectnet isaac_ros_detectnet.launch.py \
            image_topic:=/front_stereo_camera/left/image_rect_color \
            camera_info_topic:=/front_stereo_camera/left/camera_info
        ```
        *Adjust `image_topic` and `camera_info_topic` to match your setup.*

2.  **Visualize Detections in `rviz2`**:
    Open `rviz2` in a new terminal/container.
    *   Add an `Image` display to visualize the camera feed.
    *   Add a `Detection2DArray` display (if `isaac_ros_detectnet` publishes this type) and set its topic to the output topic of the DetectNet node (e.g., `/detections`).
    You should see bounding boxes overlaid on the camera feed, indicating detected objects.

## Task 2: Simple Grasping Strategy

Based on the object detections, we will implement a basic strategy to "grasp" a detected object. For a humanoid robot, this would involve inverse kinematics and motion planning. For this lab, we'll simplify to sending a command to a gripper to close if an object is within a certain distance.

1.  **Create a Grasp Controller Node (Python)**:
    Create a new Python node `grasp_controller.py` in your workspace (e.g., `robot_controller` package from Module 1). This node will subscribe to object detections and publish a simple gripper command.
    ```python
    import rclpy
    from rclpy.node import Node
    from vision_msgs.msg import Detection2DArray
    from std_msgs.msg import String # Simple gripper command

    class GraspController(Node):
        def __init__(self):
            super().__init__('grasp_controller')
            self.detection_subscription = self.create_subscription(
                Detection2DArray,
                '/detections', # Topic from Isaac ROS DetectNet
                self.detection_callback,
                10
            )
            self.gripper_publisher = self.create_publisher(String, '/gripper_command', 10)
            self.get_logger().info('Grasp Controller Node started. Waiting for detections...')

        def detection_callback(self, msg: Detection2DArray):
            for detection in msg.detections:
                # Assuming the detection has a bbox and a class_name
                # This is a simplified example, actual grasping needs 3D pose and kinematics
                object_class = detection.results[0].id # Simplified; often class_id
                object_x = detection.bbox.center.x
                object_y = detection.bbox.center.y
                
                # Check if the object is roughly in the center of the image (indicating it's in front)
                # And if it's large enough (proxy for closeness - a very rough heuristic)
                if 300 < object_x < 900 and 300 < object_y < 700: # Assuming 1280x720 image
                    self.get_logger().info(f'Detected {object_class} at ({object_x}, {object_y}). Attempting grasp.')
                    gripper_msg = String()
                    gripper_msg.data = "CLOSE_GRIPPER"
                    self.gripper_publisher.publish(gripper_msg)
                    return # Only act on the first detected object for simplicity

    def main(args=None):
        rclpy.init(args=args)
        node = GraspController()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

2.  **Update `setup.py`**: Add the `grasp_controller` entry point. Rebuild and source your workspace.

3.  **Run the Grasp Controller Node**:
    ```bash
    ros2 run robot_controller grasp_controller
    ```

4.  **Simulate Gripper Action**:
    In a real system, a `gripper_interface` node would subscribe to `/gripper_command` and actuate the gripper. For this lab, you can manually publish a command:
    ```bash
    ros2 topic pub /gripper_command std_msgs/msg/String '{data: "OPEN_GRIPPER"}'
    ```
    And observe the `grasp_controller` node's output when an object is detected.

## Conclusion

This lab provided a practical introduction to leveraging NVIDIA Isaac ROS for GPU-accelerated perception. You integrated an object detection pipeline and developed a rudimentary grasping strategy. This forms a critical foundation for building more sophisticated AI-driven behaviors in your autonomous humanoid robots. The next module will focus on Vision-Language-Action (VLA) models, taking robot intelligence to the next level.
