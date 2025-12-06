<!-- Sync Impact Report:
Version change:  --> 1.0.0
<!-- List of modified principles:
- PROJECT_NAME: GeminiBook
- Core Purpose: Teach students to build, simulate, and deploy autonomous humanoid robots using ROS 2, NVIDIA Isaac, and Vision-Language-Action models.
- Scope: 13-week capstone curriculum + full hardware lab guide + capstone project.
- Audience: Advanced AI students, robotics engineers, university labs, hackathon teams.
- Outcome: Complete open-source Docusaurus book + ready-to-teach curriculum + hardware BOMs
- Added section: Project Overview
- Added principles: Code Quality, Test-Driven Development, Modularity, Security by Design, Performance Optimization, Open Source Contribution
- Added section: Additional Constraints
- Added section: Development Workflow
- Added governance rules
- Updated CONSTITUTION_VERSION to 1.0.0
- Updated RATIFICATION_DATE to 2025-12-06
- Updated LAST_AMENDED_DATE to 2025-12-06

Templates requiring updates:
- .specify/templates/plan-template.md ⚠ pending
- .specify/templates/spec-template.md ⚠ pending
- .specify/templates/tasks-template.md ⚠ pending
- .gemini/commands/*.toml ⚠ pending
- README.md ⚠ pending

Follow-up TODOs:
- Review and refine principles with more specific project context.
- Update dependent templates and documentation to reflect changes.
-->
# GeminiBook Constitution

## Project Overview
This project, GeminiBook, aims to teach students to build, simulate, and deploy autonomous humanoid robots using ROS 2, NVIDIA Isaac, and Vision-Language-Action models.
The scope includes a 13-week capstone curriculum, a full hardware lab guide, and a capstone project.
The target audience is Advanced AI students, robotics engineers, university labs, and hackathon teams.
The desired outcome is a complete open-source Docusaurus book, a ready-to-teach curriculum, and comprehensive hardware Bills of Materials (BOMs).

## Core Principles

### I. Code Quality
All code MUST adhere to established style guides, pass linting checks, and be well-documented. Readability and maintainability are paramount.

### II. Test-Driven Development
New features and bug fixes MUST be accompanied by comprehensive tests written before implementation. The Red-Green-Refactor cycle is strictly enforced.

### III. Modularity
Components SHOULD be loosely coupled and highly cohesive. Promote reusability and clear separation of concerns.

### IV. Security by Design
Security considerations MUST be integrated into every stage of the development lifecycle, from design to deployment. Adhere to OWASP Top 10 guidelines and secure coding practices.

### V. Performance Optimization
Performance-critical sections of code SHOULD be optimized and benchmarked. Avoid premature optimization, but always consider efficiency.

### VI. Open Source Contribution
All project contributions MUST align with open-source best practices and licensing requirements. Ensure proper attribution and community engagement.

## Additional Constraints
Technology stack: ROS 2, NVIDIA Isaac, Python, Docusaurus. All external dependencies MUST be explicitly declared and managed.

## Development Workflow
Follow an agile development methodology with regular sprints, code reviews, and continuous integration/continuous deployment (CI/CD) practices. All changes MUST be reviewed and approved by at least one other team member.

## Governance
The constitution serves as the foundational document for the project. Amendments require a formal proposal, team discussion, and majority consensus. All project decisions and code contributions MUST align with the principles outlined herein.

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
