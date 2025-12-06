---
sidebar_position: 2
---

# ROS 2 Fundamentals

ROS 2 (Robot Operating System 2) is not an operating system in the traditional sense, but rather a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot applications. Understanding its core concepts is crucial for any robotics engineer.

## Core Concepts of ROS 2

At the heart of ROS 2 are several key concepts that enable distributed and modular robotic systems:

### 1. Nodes
*   **Definition**: A node is an executable process that performs a computation. In ROS 2, nodes are the fundamental units of computation. A robot control system typically consists of many nodes, each responsible for a specific task (e.g., one node for reading laser data, another for controlling motors, another for path planning).
*   **Modularity**: Nodes are designed to be modular and reusable. They can be developed and tested independently.
*   **Communication**: Nodes communicate with each other using various ROS 2 communication mechanisms.

### 2. Topics
*   **Definition**: Topics are named buses over which nodes exchange messages. This is a publish/subscribe communication model.
*   **Publisher**: A node that sends messages to a topic is called a publisher.
*   **Subscriber**: A node that receives messages from a topic is called a subscriber.
*   **Loose Coupling**: Publishers and subscribers are not directly aware of each other. They communicate through the topic, providing loose coupling and flexibility.
*   **Message Types**: Each topic has a specific message type, defining the structure of the data being transmitted. Common message types include sensor readings, motor commands, and status updates.

### 3. Services
*   **Definition**: Services are a request/reply communication model. They are used for synchronous interactions where a node sends a request and expects a response.
*   **Service Server**: A node that offers a service.
*   **Service Client**: A node that invokes a service.
*   **Synchronous**: The client typically blocks until it receives a response from the server. This is suitable for tasks that require immediate feedback.

### 4. Actions
*   **Definition**: Actions are a long-running, asynchronous communication model used for tasks that take a significant amount of time and might need preemption or periodic feedback.
*   **Action Server**: Provides the action and sends continuous feedback about its progress.
*   **Action Client**: Requests the action and can monitor its progress or cancel it.
*   **Goal, Feedback, Result**: An action has a goal (what to achieve), feedback (progress updates), and a result (final outcome).
*   **Asynchronous**: The client does not block, allowing it to perform other tasks while the action is in progress.

### 5. Parameters
*   **Definition**: Parameters are dynamic configuration values that nodes can load at startup or modify at runtime.
*   **Configuration**: Used to configure node behavior without recompiling code (e.g., PID gains, sensor offsets, map filenames).
*   **Dynamic Reconfiguration**: ROS 2 provides mechanisms for dynamically changing parameters while nodes are running.

### 6. Messages and Interfaces
*   **Definition**: Messages (`.msg` files), Services (`.srv` files), and Actions (`.action` files) define the data structures used for communication in ROS 2. These are collectively known as ROS Interfaces.
*   **Generation**: From these interface definitions, ROS 2 generates code in various programming languages (e.g., C++, Python) to handle serialization and deserialization of data.

## Quality of Service (QoS)
ROS 2 introduces Quality of Service (QoS) settings, allowing developers to fine-tune the reliability, durability, and latency of communication for different types of data. This is crucial for real-time and safety-critical applications.

## ROS 2 vs. ROS 1
ROS 2 was developed to address limitations of ROS 1, particularly concerning multi-robot systems, real-time control, security, and enterprise deployment. Key improvements include:

*   **DDS (Data Distribution Service) Integration**: ROS 2 leverages DDS as its underlying communication layer, offering improved reliability, scalability, and quality of service options.
*   **Security**: Built-in security features through DDS for authentication, encryption, and access control.
*   **Real-time Capabilities**: Enhanced support for real-time operating systems and deterministic behavior.
*   **Multi-robot Support**: Designed from the ground up to support multiple robots without conflicting namespaces.

Understanding these fundamentals is your first step towards building powerful and autonomous robotic systems. The next sections will dive into practical applications and hands-on labs.