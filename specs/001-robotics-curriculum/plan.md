# Implementation Plan: Physical AI & Humanoid Robotics Curriculum

**Branch**: `001-robotics-curriculum` | **Date**: 2025-12-06 | **Spec**: specs/001-robotics-curriculum/spec.md
**Input**: Feature specification from `specs/001-robotics-curriculum/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

The project aims to create a comprehensive, open-source Docusaurus book for teaching Physical AI and Humanoid Robotics, including ROS 2, NVIDIA Isaac, and Vision-Language-Action models. The curriculum will span 13 weeks, with a full hardware lab guide and a capstone project. The planning involves setting up the Docusaurus book structure, generating detailed Markdown content for each chapter, and organizing associated images.

## Technical Context

**Language/Version**: Markdown (for content), Docusaurus (framework), JavaScript/TypeScript (for Docusaurus extensions if needed)
**Primary Dependencies**: Docusaurus
**Storage**: Git repository (for Markdown files), local filesystem (for generated Docusaurus site)
**Testing**: Manual review of generated Markdown content, Docusaurus build/serve checks
**Target Platform**: Web (static site hosted by Docusaurus)
**Project Type**: Web application (static site generator)
**Performance Goals**: Fast page loads (Docusaurus defaults), efficient search within the book
**Constraints**: Open-source, Docusaurus framework, specific chapter order, detailed content (60+ Markdown files), adherence to Docusaurus default accessibility standards.
**Scale/Scope**: 13-week curriculum, full hardware lab guide, capstone project, 60+ Markdown files.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

-   **I. Code Quality**: All generated Markdown content must be well-structured, formatted, and free of typos. Docusaurus configuration files (if any are generated/modified) must adhere to best practices. (✅ PASS - Will ensure during content generation and Docusaurus setup).
-   **II. Test-Driven Development**: Not directly applicable to content generation but will ensure Docusaurus builds successfully. (✅ PASS - Docusaurus build process serves as a functional test).
-   **III. Modularity**: The book content is naturally modularized by chapters and modules as per the Docusaurus folder structure. (✅ PASS - User-defined folder structure promotes modularity).
-   **IV. Security by Design**: Not directly applicable to a static site, but ensure no sensitive information is accidentally published. (✅ PASS - Content will be public, no sensitive data involved).
-   **V. Performance Optimization**: Rely on Docusaurus's inherent performance. No specific manual optimization planned at this stage. (✅ PASS - Docusaurus defaults are sufficient per spec).
-   **VI. Open Source Contribution**: The entire project is explicitly open-source. (✅ PASS - Aligns with project goal).

## Project Structure

### Documentation (this feature)
This planning document outlines the generation of the following documentation structure (the Docusaurus content):

```text
.
├── docs/
│   ├── intro/
│   │   └── _index.md
│   ├── hardware/
│   │   └── _index.md
│   ├── module-1-ros2/
│   │   └── _index.md
│   ├── module-2-digital-twin/
│   │   └── _index.md
│   ├── module-3-isaac/
│   │   └── _index.md
│   ├── module-4-vla/
│   │   └── _index.md
│   ├── capstone/
│   │   └── _index.md
│   ├── appendices/
│   │   └── _index.md
│   └── ... (additional 60+ markdown files will be generated within these chapter directories)
├── static/
│   └── img/
│       └── ... (robot/hardware images will be placed here)
└── specs/001-robotics-curriculum/
    ├── plan.md              # This file
    ├── spec.md              # Feature specification
    └── checklists/
        └── requirements.md  # Spec quality checklist
```

### Source Code (repository root)
The primary "source code" for this feature is the Docusaurus content (Markdown files) and Docusaurus configuration. The folder structure is defined by the user for the Docusaurus content.

**Structure Decision**: The project will utilize a static site generator (Docusaurus) for curriculum delivery. The content will reside in the `docs/` and `static/img/` directories, following the specified chapter structure. This aligns with the "Web application" project type, though it's a static site and not a dynamic application.

## Complexity Tracking
*None - no constitution violations to justify.*

## Phase 0: Outline & Research

1.  **Extract unknowns from Technical Context**: All `NEEDS CLARIFICATION` are addressed in the Technical Context section.
2.  **Generate and dispatch research agents**: No specific research needed at this stage beyond confirming Docusaurus capabilities for the specified structure.
3.  **Consolidate findings** in `research.md`: Not required for this phase, as all initial requirements are clear for content generation.

**Output**: This phase is primarily about content generation and structure. No `research.md` is needed for this plan.

## Phase 1: Design & Contracts

**Prerequisites:** All initial planning is complete.

1.  **Extract entities from feature spec** → `data-model.md`: Not applicable. The entities (Chapter, Hardware Requirement, Learning Module, Autonomous Humanoid Robot) are conceptual for the curriculum content, not for a software data model.
2.  **Generate API contracts** from functional requirements: Not applicable. This project generates static content, not APIs.
3.  **Agent context update**: The agent context will be implicitly updated with the generated content.

**Specific Implementation for Book Structure and Content Generation (User's Instructions):**

1.  **Create book folder structure**:
    *   `mkdir -p docs/intro docs/hardware docs/module-1-ros2 docs/module-2-digital-twin docs/module-3-isaac docs/module-4-vla docs/capstone docs/appendices`
    *   This will ensure the base directories for the Docusaurus content are present.

2.  **Generate 60+ Markdown files directly into those folders using gemini**:
    *   Each primary chapter (intro, hardware, module-1, etc.) will have an `_index.md` or similar entry point.
    *   Additional Markdown files will be generated to provide detailed content for each section within these chapters, aiming for approximately 60+ total files to ensure comprehensive coverage.
    *   Content generation will be done iteratively using Gemini, focusing on the learning outcomes and weekly breakdown described in the spec.
    *   The `plan.md` will list these as tasks.

3.  **Drop all robot/hardware images into your existing `my-website/static/img/`**:
    *   Assumed target directory is `static/img` in the project root (as per `FR-009` in spec.md).
    *   Images are external assets and will be manually provided. This plan outlines the placeholder creation.
    *   The `static/img` directory will be created if it doesn't exist, serving as the designated location for these images.

**Output**: The Docusaurus book structure will be created, and content generation tasks will be detailed for each chapter. No `data-model.md`, `contracts/`, or `quickstart.md` will be generated, as they are not relevant to this content-focused project.