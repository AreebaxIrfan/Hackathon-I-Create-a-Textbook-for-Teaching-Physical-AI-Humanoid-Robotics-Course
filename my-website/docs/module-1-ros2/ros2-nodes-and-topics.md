---
sidebar_position: 3
---

# ROS 2 Nodes and Topics

Understanding ROS 2 nodes and topics is fundamental to building any robotic system. These are the primary mechanisms through which different components of your robot software communicate and operate in a distributed manner.

## 1. Nodes: The Executable Units

In ROS 2, a **node** is essentially a program or process that performs a specific computation. Think of it as a single executable unit designed for a particular task.

*   **Modularity**: Nodes promote modularity. Instead of one monolithic program, a complex robotic application is broken down into many smaller, specialized nodes. For instance:
    *   A `camera_driver` node reads data from a camera sensor.
    *   An `image_processing` node processes the camera data (e.g., detects objects).
    *   A `motor_controller` node sends commands to the robot's motors.
    *   A `path_planner` node computes the robot's trajectory.
*   **Decentralization**: Nodes can run independently, possibly on different machines or even different CPUs/GPUs within the same machine, and communicate over a network. This makes ROS 2 highly scalable and resilient.
*   **Lifecycle Management**: ROS 2 provides mechanisms for managing the lifecycle of nodes, allowing them to be configured, activated, deactivated, and shut down gracefully.

## 2. Topics: The Message Bus

**Topics** are the most common way for nodes to exchange asynchronous, real-time data in ROS 2. They operate on a publish/subscribe model.

*   **Publishers**: A node that sends data (messages) to a topic is called a **publisher**. It announces its intention to send messages of a specific type to a named topic.
*   **Subscribers**: A node that wants to receive data from a topic is called a **subscriber**. It registers its interest in a specific topic and receives all messages published to it.
*   **Messages**: The actual data exchanged over topics are structured data types called **messages**. Each topic uses a specific message type, ensuring that all data sent and received on that topic adheres to a predefined format.
    *   Messages are defined in `.msg` files, specifying fields and their types (e.g., `std_msgs/msg/String`, `sensor_msgs/msg/Image`, `geometry_msgs/msg/Twist`).
*   **Loose Coupling**: Publishers and subscribers do not need to know about each other's existence. They only need to agree on the topic name and message type. This makes the system flexible; you can add or remove nodes without affecting others as long as they adhere to the communication contracts.
*   **One-to-Many, Many-to-One, Many-to-Many**: A single topic can have multiple publishers and multiple subscribers. For example, a `camera_feed` topic might have one `camera_driver` publisher and multiple subscribers (e.g., `image_processing` node, `image_logger` node, `viewer` node).

### How Topics Work

1.  A publisher node creates a message of a specified type.
2.  The publisher sends this message to a named topic.
3.  Any subscriber node listening to that specific topic receives a copy of the message.
4.  This communication happens asynchronously and continuously.

### Example: Robot Teleoperation

Consider a robot controlled by a human operator:

*   A `teleop_keyboard` node publishes `geometry_msgs/msg/Twist` messages (containing linear and angular velocity commands) to a topic named `/cmd_vel`.
*   A `motor_controller` node subscribes to the `/cmd_vel` topic, receives these `Twist` messages, and translates them into control signals for the robot's motors.

In this setup, the `teleop_keyboard` node doesn't need to know anything about the `motor_controller` node, and vice-versa. They just agree on the `/cmd_vel` topic and the `geometry_msgs/msg/Twist` message type.

## Using ROS 2 Command-Line Tools for Topics and Nodes

ROS 2 provides powerful command-line tools for inspecting and interacting with nodes and topics:

*   **`ros2 run <package_name> <executable_name>`**: Runs an executable from a ROS 2 package, typically a node.
*   **`ros2 node list`**: Lists all active ROS 2 nodes.
*   **`ros2 node info <node_name>`**: Shows detailed information about a specific node, including its publishers, subscribers, services, and actions.
*   **`ros2 topic list`**: Lists all active topics.
*   **`ros2 topic info <topic_name>`**: Displays information about a topic, such as its message type and active publishers/subscribers.
*   **`ros2 topic echo <topic_name>`**: Displays messages being published on a topic in real-time.
*   **`ros2 topic pub <topic_name> <message_type> <message_data>`**: Publishes a single message to a topic from the command line.

These tools are invaluable for debugging, understanding system behavior, and quickly prototyping interactions within your ROS 2 robotic system.

## Next Steps

With a clear grasp of nodes and topics, we can now explore other communication paradigms in ROS 2, such as services and actions, which provide different communication patterns for more complex interactions.