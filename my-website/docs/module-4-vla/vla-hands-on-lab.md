---
sidebar_position: 5
---

# VLA Hands-on Lab: Object Grounding and Task Execution

This lab is designed to provide a practical introduction to Vision-Language-Action (VLA) models. You will implement a simplified VLA pipeline that takes natural language commands, grounds them to objects in a visual scene (simulated or real), and translates them into basic robot actions.

## Learning Objectives

*   Understand the concept of language grounding to visual elements.
*   Integrate vision and language components to interpret human commands.
*   Translate high-level commands into robot-executable actions.
*   Utilize ROS 2 for communication between VLA components and a simulated robot.

## Prerequisites

*   A working ROS 2 environment (e.g., Humble) on Ubuntu.
*   Python 3 with `rclpy`, `torch` (or `tensorflow`), `transformers` (for LLMs), and a vision library (e.g., `opencv-python`).
*   A simulated robot with a camera and simple gripper (e.g., in Gazebo or Isaac Sim, configured to publish camera images and accept gripper commands).
*   Basic understanding of ROS 2 nodes, topics, and services.

## Lab Setup

1.  **Create a ROS 2 Workspace and Package**:
    If you haven't already, create a workspace and a package for your VLA lab.
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ros2 pkg create vla_robot_lab --build-type ament_python --dependencies rclpy sensor_msgs geometry_msgs std_msgs vision_msgs
    cd vla_robot_lab
    ```
    *Note: `vision_msgs` might need to be installed separately or built from source if not in your distro.*

2.  **Install Python Dependencies**:
    ```bash
    pip install transformers opencv-python Pillow # Pillow for image handling
    # If using PyTorch: pip install torch torchvision
    # If using TensorFlow: pip install tensorflow
    ```

3.  **Simulated Robot**: Ensure you have a simulated robot publishing camera images (e.g., `/camera/image_raw`) and accepting gripper commands (e.g., `/gripper_command` of type `std_msgs/msg/String`).

## Task 1: Language Understanding with a Mini-LLM

We will use a small pre-trained language model from Hugging Face `transformers` to extract key entities and actions from a command.

1.  **Create `language_parser_node.py` in `vla_robot_lab/vla_robot_lab/`**:
    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from transformers import pipeline

    class LanguageParserNode(Node):
        def __init__(self):
            super().__init__('language_parser_node')
            self.publisher_ = self.create_publisher(String, 'parsed_command', 10)
            self.subscription = self.create_subscription(
                String,
                'human_command',
                self.command_callback,
                10
            )
            self.nlp_pipeline = pipeline("question-answering", model="distilbert-base-cased-distilled-squad")
            self.get_logger().info('Language Parser Node started. Waiting for human commands...')

        def command_callback(self, msg: String):
            command_text = msg.data
            self.get_logger().info(f'Received command: "{command_text}"')

            # Example: Try to extract action and object
            action_query = "What is the main action?"
            object_query = "What is the object of the action?"
            
            action_result = self.nlp_pipeline(question=action_query, context=command_text)
            object_result = self.nlp_pipeline(question=object_query, context=command_text)

            parsed_action = action_result['answer'] if action_result['score'] > 0.5 else "unknown_action"
            parsed_object = object_result['answer'] if object_result['score'] > 0.5 else "unknown_object"

            parsed_command_msg = String()
            parsed_command_msg.data = f"ACTION:{parsed_action}, OBJECT:{parsed_object}"
            self.publisher_.publish(parsed_command_msg)
            self.get_logger().info(f'Parsed: {parsed_command_msg.data}')

    def main(args=None):
        rclpy.init(args=args)
        node = LanguageParserNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

2.  **Edit `setup.py`**: Add an entry point for `language_parser_node`. Build and source your workspace.

3.  **Test the language parser**:
    ```bash
    ros2 run vla_robot_lab language_parser_node
    # In another terminal:
    ros2 topic pub /human_command std_msgs/msg/String 'data: "Pick up the red block"'
    ```
    You should see the parser attempting to identify "Pick up" as action and "red block" as object.

## Task 2: Visual Grounding with a Simple Color Detector (Simulated Vision)

We will create a node that simulates visual grounding by detecting colors in a dummy scene (or from a simulated camera if possible) and correlating with the parsed language.

1.  **Create `visual_grounder_node.py` in `vla_robot_lab/vla_robot_lab/`**:
    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import Image # For simulated camera input
    from cv_bridge import CvBridge # For converting ROS Image to OpenCV image
    import cv2
    import numpy as np

    class VisualGrounderNode(Node):
        def __init__(self):
            super().__init__('visual_grounder_node')
            self.publisher_ = self.create_publisher(String, 'grounded_object_pose', 10)
            self.parsed_command_sub = self.create_subscription(
                String,
                'parsed_command',
                self.parsed_command_callback,
                10
            )
            self.image_sub = self.create_subscription(
                Image,
                '/camera/image_raw', # Assuming a simulated camera
                self.image_callback,
                10
            )
            self.bridge = CvBridge()
            self.current_image = None
            self.current_parsed_object = None
            self.get_logger().info('Visual Grounder Node started.')

        def image_callback(self, msg: Image):
            try:
                self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except CvBridgeError as e:
                self.get_logger().error(f'CvBridge Error: {e}')

        def parsed_command_callback(self, msg: String):
            parts = msg.data.split(', ')
            action = parts[0].split(':')[1]
            obj = parts[1].split(':')[1]
            self.current_parsed_object = obj
            self.get_logger().info(f'Attempting to ground object: {obj} for action: {action}')
            self.ground_object_in_image()

        def ground_object_in_image(self):
            if self.current_image is None or self.current_parsed_object is None:
                return

            # Simplified grounding: detect a color mentioned in the object name
            # This would be replaced by actual object detection/segmentation in a real VLA
            color_detected = "none"
            object_pose_data = "x:0,y:0,z:0" # Placeholder

            if "red" in self.current_parsed_object.lower():
                lower_red = np.array([0, 50, 50])
                upper_red = np.array([10, 255, 255])
                hsv = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, lower_red, upper_red)
                if cv2.countNonZero(mask) > 100: # If enough red pixels are found
                    color_detected = "red"
                    # In a real scenario, calculate centroid/pose
                    object_pose_data = "x:0.3,y:0.1,z:0.0" # Example pose

            elif "green" in self.current_parsed_object.lower():
                lower_green = np.array([40, 50, 50])
                upper_green = np.array([80, 255, 255])
                hsv = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, lower_green, upper_green)
                if cv2.countNonZero(mask) > 100:
                    color_detected = "green"
                    object_pose_data = "x:0.1,y:-0.2,z:0.0" # Example pose
            
            if color_detected != "none":
                grounded_msg = String()
                grounded_msg.data = f"OBJECT:{self.current_parsed_object}, POSE:{object_pose_data}"
                self.publisher_.publish(grounded_msg)
                self.get_logger().info(f'Grounded: {grounded_msg.data}')
            else:
                self.get_logger().warn(f'Could not ground object: {self.current_parsed_object}')


    def main(args=None):
        rclpy.init(args=args)
        node = VisualGrounderNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

2.  **Update `setup.py`**: Add `visual_grounder_node` entry point. Build and source your workspace.

3.  **Run the visual grounder**:
    ```bash
    ros2 run vla_robot_lab visual_grounder_node
    ```
    *Note: For this to work with real images, you would need a camera publishing to `/camera/image_raw`. For testing, you might need a simple image publisher.*

## Task 3: Action Execution Node

Finally, a node to receive grounded object poses and perform a simple action.

1.  **Create `action_executor_node.py` in `vla_robot_lab/vla_robot_lab/`**:
    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class ActionExecutorNode(Node):
        def __init__(self):
            super().__init__('action_executor_node')
            self.subscription = self.create_subscription(
                String,
                'grounded_object_pose',
                self.grounded_object_callback,
                10
            )
            self.gripper_command_pub = self.create_publisher(String, 'gripper_command', 10)
            self.get_logger().info('Action Executor Node started. Waiting for grounded objects...')

        def grounded_object_callback(self, msg: String):
            self.get_logger().info(f'Received grounded object: {msg.data}')
            
            # Extract action and pose (simplified from earlier task for direct gripper control)
            parts = msg.data.split(', ')
            obj_desc = parts[0].split(':')[1]
            obj_pose = parts[1].split(':')[1]

            # Simplified action: If object is detected, simulate a "grasp"
            self.get_logger().info(f'Executing action: Grasping {obj_desc} at {obj_pose}')
            
            gripper_msg = String()
            gripper_msg.data = "CLOSE_GRIPPER"
            self.gripper_command_pub.publish(gripper_msg)
            self.get_logger().info('Published gripper command: CLOSE_GRIPPER')

    def main(args=None):
        rclpy.init(args=args)
        node = ActionExecutorNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

2.  **Update `setup.py`**: Add `action_executor_node` entry point. Build and source your workspace.

3.  **Run the action executor**:
    ```bash
    ros2 run vla_robot_lab action_executor_node
    ```

## End-to-End Test

1.  Launch the simulated robot and camera.
2.  Launch `language_parser_node`.
3.  Launch `visual_grounder_node`.
4.  Launch `action_executor_node`.
5.  Publish a command: `ros2 topic pub /human_command std_msgs/msg/String 'data: "Pick up the red block"'`
6.  Observe the output from all nodes. You should see the command parsed, the object grounded (if red pixels are in the camera feed), and a gripper command published.

## Conclusion

This lab provided a basic, conceptual implementation of a VLA pipeline. Real-world VLA models involve much more sophisticated AI models for language understanding, multimodal fusion, and robot control. However, this lab demonstrates the fundamental principles of how robots can combine vision and language to execute actions, paving the way for more intelligent and intuitive human-robot interaction.