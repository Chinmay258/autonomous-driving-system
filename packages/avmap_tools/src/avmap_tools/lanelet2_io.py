"""Lanelet2 OSM XML writer/reader.

Writes the exact primitive structure Lanelet2/Autoware expects: <node>s with
lat/lon (+ local_x/local_y tags for lossless roundtrip), <way>s per lane bound
(deduplicated, so adjacent lanes share their divider way — which is also how
lateral adjacency is rediscovered on read), and <relation type=lanelet>.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections.abc import Sequence

from avcore import Lanelet, LaneletId, LaneletSubtype, LatLng, Point2D
from avmap_tools.projection import LocalProjector

_MPS_TO_KMH = 3.6


def write_osm(lanelets: Sequence[Lanelet], origin: LatLng) -> str:
    """Serialize lanelets to Lanelet2 OSM XML (deterministic output)."""
    projector = LocalProjector(origin)
    root = ET.Element("osm", version="0.6", generator="avmap_tools")

    node_ids: dict[Point2D, int] = {}
    way_ids: dict[tuple[Point2D, ...], int] = {}
    next_node = 1
    next_way = 1

    def node_id(point: Point2D) -> int:
        nonlocal next_node
        if point not in node_ids:
            node_ids[point] = next_node
            coord = projector.to_latlng(point)
            el = ET.SubElement(
                root, "node", id=str(next_node), lat=repr(coord.lat), lon=repr(coord.lng)
            )
            ET.SubElement(el, "tag", k="local_x", v=repr(point.x))
            ET.SubElement(el, "tag", k="local_y", v=repr(point.y))
            ET.SubElement(el, "tag", k="ele", v="0.0")
            next_node += 1
        return node_ids[point]

    subtype_tags: dict[int, ET.Element] = {}
    way_roles: dict[int, set[str]] = {}

    def way_id(bound: tuple[Point2D, ...]) -> int:
        nonlocal next_way
        if bound not in way_ids:
            refs = [node_id(p) for p in bound]
            way_ids[bound] = next_way
            el = ET.SubElement(root, "way", id=str(next_way))
            for ref in refs:
                ET.SubElement(el, "nd", ref=str(ref))
            ET.SubElement(el, "tag", k="type", v="line_thin")
            subtype_tags[next_way] = ET.SubElement(el, "tag", k="subtype", v="solid")
            next_way += 1
        return way_ids[bound]

    for lanelet in lanelets:
        left = way_id(lanelet.left_bound)
        right = way_id(lanelet.right_bound)
        way_roles.setdefault(left, set()).add("left")
        way_roles.setdefault(right, set()).add("right")
        rel = ET.SubElement(root, "relation", id=str(int(lanelet.id)))
        ET.SubElement(rel, "member", type="way", role="left", ref=str(left))
        ET.SubElement(rel, "member", type="way", role="right", ref=str(right))
        ET.SubElement(rel, "tag", k="type", v="lanelet")
        ET.SubElement(rel, "tag", k="subtype", v=lanelet.subtype.value)
        ET.SubElement(rel, "tag", k="location", v="urban")
        ET.SubElement(rel, "tag", k="one_way", v="yes" if lanelet.one_way else "no")
        ET.SubElement(rel, "tag", k="speed_limit", v=repr(lanelet.speed_limit_mps * _MPS_TO_KMH))
        if lanelet.is_connector:
            ET.SubElement(rel, "tag", k="turn_connector", v="yes")

    # A way serving as the left bound of one lane and the right bound of
    # another is an internal divider between same-direction lanes: mark it
    # dashed so lanelet2 traffic rules permit lane changes across it.
    for wid, roles in way_roles.items():
        if roles == {"left", "right"}:
            subtype_tags[wid].set("v", "dashed")

    ET.indent(root)
    return ET.tostring(root, encoding="unicode", xml_declaration=True)


def _tags(el: ET.Element) -> dict[str, str]:
    return {t.attrib["k"]: t.attrib["v"] for t in el.findall("tag")}


def read_osm(xml_text: str) -> tuple[list[Lanelet], LatLng]:
    """Parse Lanelet2 OSM XML back into lanelets + the projection origin.

    Prefers exact local_x/local_y tags; falls back to inverse-projecting
    lat/lon for maps produced by other tools.
    """
    root = ET.fromstring(xml_text)

    raw_nodes: dict[int, tuple[LatLng, Point2D | None]] = {}
    for el in root.findall("node"):
        tags = _tags(el)
        coord = LatLng(lat=float(el.attrib["lat"]), lng=float(el.attrib["lon"]))
        local: Point2D | None = None
        if "local_x" in tags and "local_y" in tags:
            local = Point2D(float(tags["local_x"]), float(tags["local_y"]))
        raw_nodes[int(el.attrib["id"])] = (coord, local)

    # Recover the origin from any node carrying exact local coords.
    origin = LatLng(lat=0.0, lng=0.0)
    for coord, local in raw_nodes.values():
        if local is not None:
            lat0 = coord.lat - local.y / 111_320.0
            lng0 = coord.lng - local.x / (111_320.0 * math.cos(math.radians(lat0)))
            origin = LatLng(lat=lat0, lng=lng0)
            break

    projector = LocalProjector(origin)
    points: dict[int, Point2D] = {
        nid: (local if local is not None else projector.to_local(coord))
        for nid, (coord, local) in raw_nodes.items()
    }

    ways: dict[int, tuple[Point2D, ...]] = {}
    for el in root.findall("way"):
        refs = [int(nd.attrib["ref"]) for nd in el.findall("nd")]
        ways[int(el.attrib["id"])] = tuple(points[r] for r in refs)

    lanelets: list[Lanelet] = []
    for el in root.findall("relation"):
        tags = _tags(el)
        if tags.get("type") != "lanelet":
            continue
        members = {m.attrib["role"]: int(m.attrib["ref"]) for m in el.findall("member")}
        lanelets.append(
            Lanelet(
                id=LaneletId(int(el.attrib["id"])),
                left_bound=ways[members["left"]],
                right_bound=ways[members["right"]],
                speed_limit_mps=float(tags["speed_limit"]) / _MPS_TO_KMH,
                one_way=tags.get("one_way", "yes") == "yes",
                subtype=LaneletSubtype(tags.get("subtype", "road")),
                is_connector=tags.get("turn_connector") == "yes",
            )
        )
    return lanelets, origin


def write_projector_info_yaml(origin: LatLng) -> str:
    """map_projector_info.yaml content Autoware needs next to the map."""
    return (
        "projector_type: local_cartesian_utm\n"
        f"map_origin:\n  latitude: {origin.lat}\n  longitude: {origin.lng}\n  altitude: 0.0\n"
    )
