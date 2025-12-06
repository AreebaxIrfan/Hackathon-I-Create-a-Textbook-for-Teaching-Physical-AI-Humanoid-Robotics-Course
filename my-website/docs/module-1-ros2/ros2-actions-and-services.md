---
sidebar_position: 4
---

# ROS 2 Actions and Services

Beyond the asynchronous, streaming data of Topics, ROS 2 provides more structured communication patterns for specific needs: **Services** for synchronous request/reply interactions, and **Actions** for long-running, goal-oriented tasks. These mechanisms are crucial for building complex robot behaviors that require explicit commands and feedback.

## 1. Services: Synchronous Request/Reply

**Services** are designed for communication where a node sends a single request and expects a single, immediate response. This is a synchronous communication model, meaning the client typically blocks (waits) until it receives a response from the server.

*   **Service Server**: A node that offers a service. It defines the type of request it can handle and the type of response it will return.
*   **Service Client**: A node that invokes a service. It sends a request to a service server and waits for the response.
*   **Use Cases**: Services are ideal for tasks that:
    *   Require immediate completion (e.g., "get current robot pose," "perform a single inverse kinematics calculation").
    *   Return a result that the client needs before proceeding.
    *   Are relatively short-duration operations.

### Service Definition

A service is defined by a `.srv` file, which specifies both the request and response message types, separated by three hyphens (`---`).

**Example: `AddTwoInts.srv`**
```
int64 a
int64 b
---
int64 sum
```
Here, `a` and `b` are the request fields, and `sum` is the response field.

### How Services Work

1.  A Service Server advertises its availability on a specific service name.
2.  A Service Client looks up the service by its name.
3.  The client sends a request message to the server.
4.  The server processes the request and sends a response message back to the client.
5.  The client receives the response and continues its execution.

## 2. Actions: Asynchronous Goal-Oriented Tasks

**Actions** are a more complex communication pattern designed for long-running tasks that can be preempted (cancelled), or whose progress needs to be monitored with periodic feedback. This is an asynchronous, goal-oriented communication model.

*   **Action Server**: A node that provides an action. It continuously monitors for new goals, executes the requested task, and publishes feedback on its progress.
*   **Action Client**: A node that sends a goal to an action server. It can receive periodic feedback, request cancellation of the goal, and eventually receive a final result.
*   **Use Cases**: Actions are perfect for tasks that:
    *   Take a significant amount of time to complete (e.g., "drive to a specific location," "perform a complex manipulation sequence").
    *   Require the client to monitor progress (e.g., "robot is 50% to target," "robot has grasped object").
    *   Might need to be cancelled before completion (e.g., "stop moving").

### Action Definition

An action is defined by an `.action` file, which specifies three message types: a goal, a result, and feedback. These are separated by three hyphens (`---`).

**Example: `DriveToGoal.action`**
```
# Goal
geometry_msgs/Point target_pose
---
# Result
bool success
---
# Feedback
float32 distance_remaining
```
Here, `target_pose` is the goal, `success` is the result, and `distance_remaining` is the feedback.

### How Actions Work

1.  An Action Server advertises its availability for a specific action.
2.  An Action Client sends a goal message to the server.
3.  The server starts executing the goal.
4.  Periodically, the server publishes feedback messages to the client about its progress.
5.  The client can monitor this feedback, or decide to send a cancellation request to the server.
6.  Once the server completes the goal (or is preempted), it sends a final result message to the client.

## Comparing Communication Patterns

| Feature          | Topics                   | Services                 | Actions                      |
| :--------------- | :----------------------- | :----------------------- | :--------------------------- |
| **Model**        | Publish/Subscribe        | Request/Reply            | Goal/Feedback/Result         |
| **Nature**       | Asynchronous, Streaming  | Synchronous, One-Shot    | Asynchronous, Long-running   |
| **Blocking?**    | No (client doesn't wait) | Yes (client typically waits) | No (client can monitor)      |
| **Feedback?**    | No direct feedback       | No direct feedback       | Yes, continuous progress     |
| **Preemptible?** | No                       | No                       | Yes                          |
| **Use Cases**    | Sensor data, motor commands | Get current state, trigger a single event | Navigation, complex manipulation |

## Using ROS 2 Command-Line Tools for Services and Actions

Just like topics and nodes, ROS 2 provides command-line tools for services and actions:

*   **`ros2 service list`**: Lists all active services.
*   **`ros2 service type <service_name>`**: Displays the type of a service.
*   **`ros2 service call <service_name> <service_type> <request_data>`**: Calls a service with specific request data.
*   **`ros2 action list`**: Lists all active actions.
*   **`ros2 action info <action_name>`**: Shows details about an action, including its goal, result, and feedback types.
*   **`ros2 action send_goal <action_name> <action_type> <goal_data>`**: Sends a goal to an action server.

These communication patterns, combined with nodes and topics, form the robust framework of ROS 2, enabling the development of highly sophisticated and autonomous robotic systems. The next section will explore hands-on laboratory exercises to solidify your understanding.