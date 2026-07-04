# ADR-0001: Lanelet2 OSM is the single canonical map format

**Status:** Accepted · 2026-07-05

## Context
The system needs one map consumed by two very different runtimes: the Autoware
Planning Simulator (Track A) and the lightweight web demo (Track B). Divergent
formats would mean divergent bugs and an unverifiable demo.

## Decision
The canonical artifact is a Lanelet2 OSM XML file (`lanelet2_map.osm`) plus
`map_projector_info.yaml`. Everything derives from it: Autoware loads it
directly; the web demo prebakes a routing graph from it at build time
(content-hashed for reproducibility). A CI validation gate
(`avmap_tools validate`) must pass before the map reaches either consumer.

## Consequences
+ One source of truth; the demo provably runs on the validated map.
+ Autoware-native — zero conversion risk on the research track.
− Lanelet2 tooling is Linux-centric; conversion/validation runs in WSL/CI.
