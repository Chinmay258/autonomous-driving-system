"""Map validation gate (ADR-0001): a map artifact must pass before any
consumer — Autoware or the web demo — is allowed to load it."""

from __future__ import annotations

from collections import Counter
from collections.abc import Sequence
from dataclasses import dataclass, field

from avcore import Lanelet, MapValidationError
from avcore.filtering import MAX_LANE_WIDTH_M, MIN_LANE_WIDTH_M, MIN_LANELET_LENGTH_M
from avmap_tools.graphbuild import build_graph


@dataclass(frozen=True, slots=True)
class ValidationReport:
    lanelet_count: int
    issues: tuple[str, ...] = field(default_factory=tuple)

    @property
    def ok(self) -> bool:
        return not self.issues

    def raise_if_failed(self) -> None:
        if self.issues:
            raise MapValidationError("; ".join(self.issues))


def validate_map(lanelets: Sequence[Lanelet]) -> ValidationReport:
    issues: list[str] = []

    duplicate_ids = [lid for lid, n in Counter(ll.id for ll in lanelets).items() if n > 1]
    if duplicate_ids:
        issues.append(f"duplicate lanelet ids: {sorted(duplicate_ids)}")
    if not lanelets:
        issues.append("map contains no lanelets")
        return ValidationReport(lanelet_count=0, issues=tuple(issues))

    for lanelet in lanelets:
        if not MIN_LANE_WIDTH_M <= lanelet.mean_width_m <= MAX_LANE_WIDTH_M:
            issues.append(f"lanelet {lanelet.id}: width {lanelet.mean_width_m:.2f} m out of range")
        if lanelet.length_m < MIN_LANELET_LENGTH_M:
            issues.append(f"lanelet {lanelet.id}: length {lanelet.length_m:.2f} m below minimum")

    if not duplicate_ids:
        graph = build_graph(lanelets)
        dead_ends = sum(1 for lid in graph if not graph.edges_from(lid))
        # A handful of dead ends (map border) is normal; a graph that is mostly
        # dead ends means successor derivation failed.
        if len(lanelets) > 1 and dead_ends > len(lanelets) // 2:
            issues.append(f"{dead_ends}/{len(lanelets)} lanelets have no outgoing edge")

    return ValidationReport(lanelet_count=len(lanelets), issues=tuple(issues))
