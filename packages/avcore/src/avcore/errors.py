"""Typed error hierarchy. Callers branch on these; no bare exceptions cross the API."""


class AvCoreError(Exception):
    """Base for all avcore errors."""


class UnknownLaneletError(AvCoreError):
    """A lanelet id was referenced that is not present in the graph/map."""


class UnreachableGoalError(AvCoreError):
    """No route exists between the given start and goal lanelets."""


class MapValidationError(AvCoreError):
    """The map artifact failed a validation invariant (see avmap_tools validate)."""
