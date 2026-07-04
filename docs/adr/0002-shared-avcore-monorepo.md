# ADR-0002: One shared planning core (`avcore`) in a monorepo

**Status:** Accepted · 2026-07-05

## Context
The portfolio claim is "the live demo runs the algorithm validated in
Autoware." That is only honest if there is literally one implementation.

## Decision
All planning intelligence (map model, routing graph, filtering, A*) lives in
`packages/avcore`, a pure-Python package with **zero** dependencies on web
frameworks, ROS/Autoware, or OpenCV. Both `apps/webdemo` and the Autoware
evaluation import it. The repo is a uv workspace monorepo so contracts,
tests, and CI stay atomic across packages.

## Consequences
+ No reimplementation drift; golden-reference tests (planner vs. Autoware) are meaningful.
+ `avcore` is trivially unit-testable (mypy --strict, ≥80% coverage gate).
− Discipline required: any import of fastapi/cv2/rclpy inside avcore is a review-blocking defect.
