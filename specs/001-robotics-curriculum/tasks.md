# Tasks: Physical AI & Humanoid Robotics Curriculum

**Input**: Design documents from `specs/001-robotics-curriculum/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Not explicitly requested in feature spec, but Docusaurus build/serve process will serve as functional tests.

**Organization**: Tasks are grouped by user story to enable independent implementation and content generation.

## Format: `[ID] [P?] [Story] Description`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic Docusaurus structure

- [x] T001 Initialize a new Docusaurus project in the root directory (if one doesn't exist)
- [x] T002 [P] Create the `docs/intro` directory.
- [x] T003 [P] Create the `docs/hardware` directory.
- [x] T004 [P] Create the `docs/module-1-ros2` directory.
- [x] T005 [P] Create the `docs/module-2-digital-twin` directory.
- [x] T006 [P] Create the `docs/module-3-isaac` directory.
- [x] T007 [P] Create the `docs/module-4-vla` directory.
- [x] T008 [P] Create the `docs/capstone` directory.
- [x] T009 [P] Create the `docs/appendices` directory.
- [x] T010 [P] Create the `static/img` directory.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Configure Docusaurus for the book structure

**⚠️ CRITICAL**: Book navigation and basic configuration MUST be complete before content generation can effectively begin.

- [x] T011 Configure Docusaurus `docusaurus.config.js` to include the created chapter directories in the sidebar navigation.
- [x] T012 Configure Docusaurus `docusaurus.config.js` to reflect the book's title ("Physical AI & Humanoid Robotics: Building Autonomous Humanoids with ROS 2, Isaac Sim, and Vision-Language-Action") and description.

---

## Phase 3: User Story 1 - Access comprehensive curriculum content (Priority: P1) 🎯 MVP

**Goal**: Provide a fully navigable Docusaurus book with an introduction and learning outcomes.

**Independent Test**: The Docusaurus site builds and serves, displaying the Introduction and Learning Outcomes content.

### Implementation for User Story 1

- [x] T013 [US1] Generate content for `docs/intro/_index.md` (Introduction: Why Physical AI Matters).
- [x] T014 [US1] Generate content for `docs/intro/learning-outcomes.md` (Learning Outcomes & Weekly Breakdown).
- [ ] T015 [US1] Generate content for `docs/intro/weekly-breakdown-details.md` (Detailed Weekly Breakdown content - part of "60+ files").

---

## Phase 4: User Story 2 - Understand Hardware Requirements and Lab Options (Priority: P1)

**Goal**: Provide detailed hardware and lab setup information.

**Independent Test**: The Hardware Requirements & Lab Options chapter is accessible and provides actionable BOM and setup details.

### Implementation for User Story 2

- [x] T016 [US2] Generate content for `docs/hardware/_index.md` (Hardware Requirements & Lab Options).
- [x] T017 [US2] Generate content for `docs/hardware/economy-jetson-kit-bom.md` (Economy Jetson Kit BOM).
- [x] T018 [US2] Generate content for `docs/hardware/cloud-setup-options.md` (Cloud Setup Options).

---

## Phase 5: User Story 7 - Complete a Capstone Project (Priority: P1)

**Goal**: Outline the capstone project, its guidelines, and evaluation.

**Independent Test**: The Capstone chapter is accessible and clearly defines the project and its requirements.

### Implementation for User Story 7

- [x] T019 [US7] Generate content for `docs/capstone/_index.md` (Capstone: The Autonomous Humanoid).
- [x] T020 [US7] Generate content for `docs/capstone/project-guidelines.md` (Capstone Project Guidelines).
- [x] T021 [US7] Generate content for `docs/capstone/evaluation-rubric.md` (Capstone Evaluation Rubric).

---

## Phase 6: User Story 3 - Learn and Apply ROS 2 for Robotics (Priority: P2)

**Goal**: Provide comprehensive content for the ROS 2 module.

**Independent Test**: The ROS 2 module is accessible and explains ROS 2 fundamentals and applications.

### Implementation for User Story 3

- [x] T022 [US3] Generate content for `docs/module-1-ros2/_index.md` (Module 1: The Robotic Nervous System (ROS 2)).
- [x] T023 [US3] Generate content for `docs/module-1-ros2/ros2-fundamentals.md`.
- [x] T024 [US3] Generate content for `docs/module-1-ros2/ros2-nodes-and-topics.md`.
- [x] T025 [US3] Generate content for `docs/module-1-ros2/ros2-actions-and-services.md` (Additional content for 60+ files).
- [x] T026 [US3] Generate content for `docs/module-1-ros2/ros2-hands-on-lab.md` (Additional content for 60+ files).

---

## Phase 7: User Story 4 - Simulate Robotic Systems (Priority: P2)

**Goal**: Provide comprehensive content for the Digital Twin module.

**Independent Test**: The Digital Twin module is accessible and guides users through simulation with Gazebo and Unity.

### Implementation for User Story 4

- [x] T027 [US4] Generate content for `docs/module-2-digital-twin/_index.md` (Module 2: The Digital Twin (Gazebo & Unity)).
- [x] T028 [US4] Generate content for `docs/module-2-digital-twin/gazebo-simulation.md`.
- [x] T029 [US4] Generate content for `docs/module-2-digital-twin/unity-integration.md`.
- [x] T030 [US4] Generate content for `docs/module-2-digital-twin/simulation-best-practices.md` (Additional content for 60+ files).
- [x] T031 [US4] Generate content for `docs/module-2-digital-twin/digital-twin-lab.md` (Additional content for 60+ files).

---

## Phase 8: User Story 5 - Integrate AI with Robotics using NVIDIA Isaac (Priority: P2)

**Goal**: Provide comprehensive content for the NVIDIA Isaac module.

**Independent Test**: The NVIDIA Isaac module is accessible and demonstrates AI integration with robotic platforms.

### Implementation for User Story 5

- [x] T032 [US5] Generate content for `docs/module-3-isaac/_index.md` (Module 3: The AI-Robot Brain (NVIDIA Isaac™)).
- [x] T033 [US5] Generate content for `docs/module-3-isaac/isaac-ros-overview.md`.
- [x] T034 [US5] Generate content for `docs/module-3-isaac/isaac-perception-stack.md`.
- [x] T035 [US5] Generate content for `docs/module-3-isaac/isaac-manipulation-api.md` (Additional content for 60+ files).
- [x] T036 [US5] Generate content for `docs/module-3-isaac/isaac-lab.md` (Additional content for 60+ files).

---

## Phase 9: User Story 6 - Develop with Vision-Language-Action Models (Priority: P2)

**Goal**: Provide comprehensive content for the VLA Models module.

**Independent Test**: The VLA Models module is accessible and explains how to apply VLA models in humanoid robotics.

### Implementation for User Story 6

- [x] T037 [US6] Generate content for `docs/module-4-vla/_index.md` (Module 4: Vision-Language-Action (VLA) Models).
- [x] T038 [US6] Generate content for `docs/module-4-vla/introduction-to-vla.md`.
- [x] T039 [US6] Generate content for `docs/module-4-vla/vla-implementation-examples.md`.
- [x] T040 [US6] Generate content for `docs/module-4-vla/multimodal-perception.md` (Additional content for 60+ files).
- [x] T041 [US6] Generate content for `docs/module-4-vla/vla-hands-on-lab.md` (Additional content for 60+ files).

---

## Phase 10: Appendices

**Goal**: Provide supplementary material for the curriculum.

**Independent Test**: The Appendices chapter is accessible and contains relevant supplementary guides.

### Implementation for Appendices

- [x] T042 [US6] Generate content for `docs/appendices/_index.md` (Appendices).
- [x] T043 Generate content for `docs/appendices/economy-jetson-kit-details.md` (Detailed Jetson Kit information).
- [x] T044 Generate content for `docs/appendices/cloud-platform-setup.md` (Cloud Platform Setup Guide).
- [x] T045 Generate content for `docs/appendices/further-reading.md` (Additional content for 60+ files).
- [x] T046 Generate content for `docs/appendices/troubleshooting-guide.md` (Additional content for 60+ files).

---

## Phase 11: Polish & Cross-Cutting Concerns

**Purpose**: Review, validate, and finalize the Docusaurus book.

- [x] T047 Review all generated Markdown files for consistency, accuracy, and adherence to Docusaurus best practices.
- [x] T048 Ensure all internal links within the Docusaurus content are correct and functional.
- [x] T049 Verify that all image references (e.g., `/static/img/robot.png`) are correctly formatted.
- [x] T050 Manually add actual robot/hardware images to `static/img` as external assets (This is a manual step for the user).
- [ ] T051 Run Docusaurus build command to check for any errors.
- [ ] T052 Run Docusaurus serve command and perform a full manual review of the generated website.

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Setup (Phase 1)**: No dependencies - can start immediately.
-   **Foundational (Phase 2)**: Depends on Setup completion.
-   **User Stories (Phase 3-10)**: All depend on Foundational phase completion. User stories (phases) can then be implemented in parallel or sequentially based on priority.
-   **Polish (Phase 11)**: Depends on all content generation being complete.

### User Story Dependencies

-   User Stories 1, 2, 7 (P1 priorities) can start after Foundational.
-   User Stories 3, 4, 5, 6 (P2 priorities) and Appendices can start after Foundational.

### Within Each User Story

-   Content generation for `_index.md` files should generally precede generation of detailed sub-files.

### Parallel Opportunities

-   Tasks within "Phase 1: Setup" (T002-T010) are independent and can be parallelized.
-   Tasks T013-T015 (US1 content), T016-T018 (US2 content), T019-T021 (US7 content) can be parallelized as they are P1 and cover different content areas.
-   Tasks T022-T026 (US3 content), T027-T031 (US4 content), T032-T036 (US5 content), T037-T041 (US6 content), and T042-T046 (Appendices content) can be parallelized, especially if different content generators are used.
-   Tasks T047-T052 (Polish phase) are sequential or can be loosely parallelized (e.g., content review vs. Docusaurus build).

---

## Implementation Strategy

### MVP First (User Story 1, 2, 7)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational
3.  Complete Phase 3: User Story 1 (Introductory content)
4.  Complete Phase 4: User Story 2 (Hardware/Lab info)
5.  Complete Phase 5: User Story 7 (Capstone outline)
6.  **STOP and VALIDATE**: Review the Docusaurus site with the core curriculum structure and essential info.

### Incremental Delivery

1.  Complete Setup + Foundational → Book structure ready.
2.  Add User Story 1 → Review Intro content.
3.  Add User Story 2 → Review Hardware content.
4.  Add User Story 7 → Review Capstone content.
5.  Add User Story 3 → Review ROS 2 content.
6.  Continue adding content for P2 user stories and appendices.

---

## Notes

-   [P] tasks = different files, no dependencies
-   [Story] label maps task to specific user story for traceability
-   Each user story should be independently completable and testable
-   Verify tests fail before implementing (N/A for content generation directly)
-   Commit after each task or logical group
-   Stop at any checkpoint to validate story independently
-   Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
