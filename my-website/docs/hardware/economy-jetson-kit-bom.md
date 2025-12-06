---
sidebar_position: 2
---

# Economy Jetson Kit - Bill of Materials (BOM)

This Bill of Materials (BOM) outlines the components required for assembling an **Economy Jetson Kit**, a cost-effective yet powerful platform for engaging with physical AI and humanoid robotics. This kit is designed to provide sufficient computational capability for running ROS 2, basic NVIDIA Isaac ROS applications, and smaller-scale Vision-Language-Action (VLA) models without breaking the bank.

**Note**: Prices are approximate and subject to change based on vendor, region, and availability. Always check current market prices. Links are illustrative and may need to be updated.

## Core Computing Unit

| Component                  | Quantity | Estimated Price (USD) | Recommended Vendor/Model Example                               | Notes                                                               |
| :------------------------- | :------- | :-------------------- | :------------------------------------------------------------- | :------------------------------------------------------------------ |
| NVIDIA Jetson Nano Developer Kit B01 | 1        | $99 - $149             | NVIDIA, Seeed Studio, SparkFun                                   | Entry-level Jetson, sufficient for learning and basic projects.      |
| MicroSD Card (64GB, Class 10/U1) | 1        | $10 - $20              | SanDisk Extreme, Samsung EVO Select                            | For operating system and project files. Higher speed is better.      |
| 5V 4A DC Power Supply (Barrel Jack) | 1        | $15 - $25              | Canakit, various electronics suppliers                         | Essential for stable power. Ensure correct barrel jack size (5.5x2.1mm). |

## Essential Peripherals

| Component                  | Quantity | Estimated Price (USD) | Recommended Vendor/Model Example                               | Notes                                                               |
| :------------------------- | :------- | :-------------------- | :------------------------------------------------------------- | :------------------------------------------------------------------ |
| USB Webcam (1080p)         | 1        | $25 - $40              | Logitech C920, Raspberry Pi Camera Module V2 (with adapter)      | For visual perception tasks and VLA model input.                     |
| USB WiFi Adapter (Optional, if no built-in) | 1        | $15 - $30              | TP-Link, Adafruit                                              | For wireless connectivity. Check compatibility with Jetson Nano.     |
| USB Keyboard & Mouse       | 1 set    | $20 - $40              | Basic wired or wireless set                                      | For initial setup and direct interaction.                             |
| HDMI Cable                 | 1        | $5 - $10               | Standard HDMI cable                                            | To connect to a monitor for display output.                          |
| Micro-USB to USB-A Cable (for power only, optional) | 1        | $5 - $10               | Standard Micro-USB cable (ensure it supports data and power)     | Alternative power input for some configurations, but 5V/4A barrel jack is preferred. |
| USB Hub (Powered, 4-port)  | 1        | $15 - $30              | Anker, TP-Link                                                 | To expand USB ports for multiple peripherals (camera, WiFi, etc.).    |

## Robotic Platform (Example - for integration practice)

This section provides an example of a simple mobile robot chassis that can be integrated with the Jetson Nano. Actual humanoid platforms are significantly more complex and expensive. This simpler platform allows for practical ROS 2 and AI integration.

| Component                  | Quantity | Estimated Price (USD) | Recommended Vendor/Model Example                               | Notes                                                               |
| :------------------------- | :------- | :-------------------- | :------------------------------------------------------------- | :------------------------------------------------------------------ |
| 2WD Robot Car Chassis Kit  | 1        | $30 - $50              | Elegoo, Osoyoo, DIY Robot Kits (Acrylic or Metal)              | Includes chassis, motors, wheels.                                    |
| L298N Motor Driver Module  | 1        | $5 - $10               | Adafruit, SparkFun, various electronics suppliers              | To control the DC motors with the Jetson Nano's GPIO pins.           |
| Jumper Wires (M-F, M-M)    | 1 pack   | $5 - $10               | Various electronics suppliers                                  | For connecting components.                                           |
| Breadboard (Mini)          | 1        | $5 - $10               | Various electronics suppliers                                  | For prototyping connections.                                         |
| External Battery Pack (e.g., 5V USB Power Bank) | 1        | $20 - $40              | Anker PowerCore, RAVPower (10000mAh+)                           | To power the robot chassis motors independently or as portable power for Jetson. |

## Total Estimated Cost (Approximate): $250 - $450

This kit provides a solid foundation for the labs and projects outlined in this curriculum. While dedicated humanoid robotics platforms are significantly more advanced and costly, this Economy Jetson Kit allows students to learn the core principles of perception, control, and AI integration in a physical robot context.

For more advanced or specific projects, students may choose to upgrade individual components or explore more sophisticated robotic kits. The focus remains on leveraging the Jetson Nano as the brain for intelligent physical systems.