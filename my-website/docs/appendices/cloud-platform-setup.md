---
sidebar_position: 3
---

# Cloud Platform Setup Guide

Leveraging cloud platforms for robotics development offers unparalleled scalability, accessibility, and computational power, especially for complex simulations and AI model training. This guide provides a general overview and specific tips for setting up your development environment on popular cloud providers.

## General Cloud Setup Workflow

Regardless of the cloud provider, the typical workflow for setting up a robotics development environment involves these steps:

1.  **Account Creation & Billing**: Sign up for an account. Utilize free tiers or developer credits if available.
2.  **Region Selection**: Choose a data center region that is geographically close to you or your target deployment for lower latency.
3.  **Virtual Machine (VM) Provisioning**:
    *   **Instance Type**: Select an instance type with sufficient CPU, RAM, and crucially, **GPU acceleration** if you plan to use NVIDIA Isaac Sim or train deep learning models. Look for instances with NVIDIA GPUs (e.g., NVIDIA V100, A100, or T4).
    *   **Operating System**: Ubuntu Server (LTS versions like 20.04 or 22.04) is the standard for ROS 2 and robotics development.
    *   **Storage**: Allocate sufficient disk space (e.g., 100GB+ SSD) for OS, ROS 2, NVIDIA tools, and project data.
4.  **Network Configuration**:
    *   **Firewall/Security Groups**: Configure inbound rules to allow SSH access (port 22) from your IP address.
    *   **Public IP**: Ensure your VM has a public IP address for external access.
5.  **SSH Access**: Connect to your VM using an SSH client and the key pair you generated during VM creation.
6.  **NVIDIA Driver & CUDA Installation**:
    *   If your VM came with a pre-configured NVIDIA Deep Learning AMI, these might be pre-installed.
    *   Otherwise, manually install the latest NVIDIA GPU drivers and CUDA Toolkit compatible with your chosen JetPack/Isaac ROS versions.
7.  **Docker & NVIDIA Container Toolkit**:
    *   Install Docker: `sudo apt update && sudo apt install docker.io -y`
    *   Add your user to the docker group: `sudo usermod -aG docker $USER && newgrp docker`
    *   Install NVIDIA Container Toolkit: Follow the instructions on the NVIDIA Docker documentation to enable Docker containers to access your GPU. This is essential for Isaac ROS.
8.  **ROS 2 Installation**: Install your desired ROS 2 distribution (e.g., Humble) following the official ROS 2 documentation. It's often easiest to install ROS 2 *within* a Docker container, especially for Isaac ROS, or use a pre-built NVIDIA container.
9.  **Development Environment Setup**: Install VS Code (often via `code-server` for web access) and configure any other development tools.

## Platform-Specific Tips

### AWS (Amazon Web Services)

*   **EC2 Instances**: Look for P-series (e.g., `p3`, `p4`) or G-series (e.g., `g4dn`, `g5`) instances for GPU acceleration.
*   **AWS RoboMaker**: Consider using AWS RoboMaker for a managed service specifically for robotics development, simulation, and fleet management. It abstracts much of the underlying VM setup.
*   **Deep Learning AMIs**: Use AWS's Deep Learning AMIs for pre-configured environments with CUDA, cuDNN, PyTorch, TensorFlow.
*   **Storage**: Use S3 for large datasets and EBS volumes for persistent storage attached to your VM.

### Google Cloud Platform (GCP)

*   **Compute Engine**: Use `n1-standard-X` or `n1-highmem-X` machine types, and attach NVIDIA GPUs (e.g., Tesla T4, V100, A100).
*   **Cloud AI Platform**: Explore managed services like Vertex AI for training and deploying machine learning models, which can feed into your robotic applications.
*   **Container-Optimized OS**: For Docker-centric workflows, Google's Container-Optimized OS can be a good choice, but ensure it supports your ROS 2/Isaac ROS needs.

### Microsoft Azure

*   **Azure Virtual Machines**: Look for NV-series (NVIDIA Tesla M60, V100, T4) or ND-series (NVIDIA Tesla P40, V100, A100) for GPU-enabled VMs.
*   **Azure Machine Learning**: Use this service for managed ML workflows, including data preparation, training, and deployment.
*   **Azure IoT Edge/IoT Hub**: For deploying AI models and connecting to physical robots at the edge.

### NVIDIA Omniverse Cloud (and Local Isaac Sim)

*   **Local Isaac Sim**: If you have a powerful local workstation with an NVIDIA RTX GPU, you can run Isaac Sim locally. Setup involves installing Omniverse Launcher and then Isaac Sim.
*   **Cloud Deployment**: NVIDIA is continuously expanding Omniverse Cloud services, allowing for cloud-based Isaac Sim instances. Refer to NVIDIA's latest documentation for provisioning and access. This offers a highly optimized environment directly integrated with Isaac ROS.

## Remote Development (VS Code Remote Development)

For a seamless development experience, consider using **VS Code Remote Development** extensions. These allow you to code on your local machine with VS Code's rich UI, while the actual development environment (code, terminals, debugger) runs entirely on your cloud VM.

1.  **Install VS Code Remote - SSH Extension**: In VS Code, install the "Remote - SSH" extension.
2.  **Configure SSH**: Follow the extension's guide to configure SSH access to your cloud VM.
3.  **Connect**: Connect to your cloud VM via SSH within VS Code. You can then open folders, run terminals, and debug code as if it were local.

By utilizing these cloud setup options and remote development techniques, you can overcome many hardware limitations and focus on the core challenges of developing advanced autonomous humanoid robots.