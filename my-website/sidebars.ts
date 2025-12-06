import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      link: {type: 'doc', id: 'intro/intro'},
      items: [
        'intro/intro',
        'intro/learning-outcomes',
        'intro/weekly-breakdown-details',
      ],
    },
    {
      type: 'category',
      label: 'Hardware Requirements & Lab Options',
      link: {type: 'doc', id: 'hardware/hardware'},
      items: [
        'hardware/hardware',
        'hardware/economy-jetson-kit-bom',
        'hardware/cloud-setup-options',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      link: {type: 'doc', id: 'module-1-ros2/module-1-ros2'},
      items: [
        'module-1-ros2/module-1-ros2',
        'module-1-ros2/ros2-fundamentals',
        'module-1-ros2/ros2-nodes-and-topics',
        'module-1-ros2/ros2-actions-and-services',
        'module-1-ros2/ros2-hands-on-lab',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      link: {type: 'doc', id: 'module-2-digital-twin/module-2-digital-twin'},
      items: [
        'module-2-digital-twin/module-2-digital-twin',
        'module-2-digital-twin/gazebo-simulation',
        'module-2-digital-twin/unity-integration',
        'module-2-digital-twin/simulation-best-practices',
        'module-2-digital-twin/digital-twin-lab',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      link: {type: 'doc', id: 'module-3-isaac/module-3-isaac'},
      items: [
        'module-3-isaac/module-3-isaac',
        'module-3-isaac/isaac-ros-overview',
        'module-3-isaac/isaac-perception-stack',
        'module-3-isaac/isaac-manipulation-api',
        'module-3-isaac/isaac-lab',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) Models',
      link: {type: 'doc', id: 'module-4-vla/module-4-vla'},
      items: [
        'module-4-vla/module-4-vla',
        'module-4-vla/introduction-to-vla',
        'module-4-vla/vla-implementation-examples',
        'module-4-vla/multimodal-perception',
        'module-4-vla/vla-hands-on-lab',
      ],
    },
    {
      type: 'category',
      label: 'Capstone: The Autonomous Humanoid',
      link: {type: 'doc', id: 'capstone/capstone'},
      items: [
        'capstone/capstone',
        'capstone/project-guidelines',
        'capstone/evaluation-rubric',
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      link: {type: 'doc', id: 'appendices/appendices'},
      items: [
        'appendices/appendices',
        'appendices/economy-jetson-kit-details',
        'appendices/cloud-platform-setup',
        'appendices/further-reading',
        'appendices/troubleshooting-guide',
      ],
    },
  ],
};

export default sidebars;
