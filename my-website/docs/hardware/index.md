---
id: hardware
sidebar_position: 1
---

# Hardware Requirements & Lab Options

Successfully building and deploying autonomous humanoid robots necessitates a robust understanding of both software and hardware. This section provides a comprehensive guide to the physical components required for this curriculum, along with various lab setup options to accommodate different budgets and institutional resources.

Our approach emphasizes accessibility while not compromising on the capabilities essential for cutting-edge physical AI development. We will outline the specifications for a recommended **Economy Jetson Kit**, a cost-effective yet powerful platform for on-robot AI computation, alongside strategies for leveraging **Cloud-based Simulation Environments** for those with limited access to physical hardware.

## Key Considerations for Hardware Selection

When choosing or procuring hardware for physical AI and robotics, several factors are paramount:

1.  **Computational Power**: Autonomous robots, especially those utilizing Vision-Language-Action (VLA) models and complex control algorithms, demand significant processing power. NVIDIA Jetson platforms offer a good balance of performance and energy efficiency.
2.  **Sensor Suite**: Robots interact with the world through sensors. Common requirements include cameras (stereo or monocular), LiDAR for 3D mapping, IMUs (Inertial Measurement Units) for pose estimation, and force/torque sensors for manipulation.
3.  **Actuation Systems**: The ability to move and manipulate is fundamental. This involves selecting appropriate motors, servos, and robotic arms, considering their payload capacity, speed, and precision.
4.  **Power Management**: Reliable power delivery and efficient battery solutions are critical for untethered robotic operation.
5.  **Interoperability**: Components should ideally support open standards (e.g., USB, Ethernet, GPIO) and have well-documented drivers or ROS 2 compatibility.

## Tailoring Your Lab Environment

We recognize that not all students or institutions will have identical resources. Therefore, this curriculum is designed with flexibility in mind, offering pathways for both:

*   **Physical Labs**: Utilizing the recommended Jetson Kit and associated robotic platforms for direct hands-on experience.
*   **Virtual Labs**: Leveraging powerful cloud computing and advanced simulation tools (NVIDIA Isaac Sim, Gazebo, Unity) to replicate real-world scenarios without the need for extensive physical hardware.

The subsequent sections will delve into the specifics of the Economy Jetson Kit, providing Bills of Materials (BOMs), assembly instructions, and essential software setup. We will also detail how to establish and utilize cloud environments for simulation-driven development, ensuring that every participant can engage with the material effectively.