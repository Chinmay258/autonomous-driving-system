"""Directed routing graph over lanelets.

Nodes are lanelets; edges are longitudinal successors or lateral (lane-change)
adjacencies. Costs are computed by the planner's CostModel, not stored here, so
the same graph serves multiple cost functions.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
from enum import StrEnum

from avcore.errors import UnknownLaneletError
from avcore.models import Lanelet, LaneletId


class EdgeKind(StrEnum):
    SUCCESSOR = "successor"  # longitudinal continuation
    LEFT = "left"  # lane change to the left neighbour
    RIGHT = "right"  # lane change to the right neighbour


@dataclass(frozen=True, slots=True)
class Edge:
    target: LaneletId
    kind: EdgeKind


class RoutingGraph:
    """Adjacency-list digraph keyed by LaneletId."""

    def __init__(self) -> None:
        self._lanelets: dict[LaneletId, Lanelet] = {}
        self._adjacency: dict[LaneletId, list[Edge]] = {}

    def add_lanelet(self, lanelet: Lanelet) -> None:
        if lanelet.id in self._lanelets:
            raise ValueError(f"lanelet {lanelet.id} already in graph")
        self._lanelets[lanelet.id] = lanelet
        self._adjacency[lanelet.id] = []

    def add_edge(self, source: LaneletId, target: LaneletId, kind: EdgeKind) -> None:
        for node in (source, target):
            if node not in self._lanelets:
                raise UnknownLaneletError(f"lanelet {node} not in graph")
        self._adjacency[source].append(Edge(target=target, kind=kind))

    def lanelet(self, lanelet_id: LaneletId) -> Lanelet:
        try:
            return self._lanelets[lanelet_id]
        except KeyError:
            raise UnknownLaneletError(f"lanelet {lanelet_id} not in graph") from None

    def edges_from(self, lanelet_id: LaneletId) -> tuple[Edge, ...]:
        if lanelet_id not in self._lanelets:
            raise UnknownLaneletError(f"lanelet {lanelet_id} not in graph")
        return tuple(self._adjacency[lanelet_id])

    def __contains__(self, lanelet_id: LaneletId) -> bool:
        return lanelet_id in self._lanelets

    def __len__(self) -> int:
        return len(self._lanelets)

    def __iter__(self) -> Iterator[LaneletId]:
        return iter(self._lanelets)
