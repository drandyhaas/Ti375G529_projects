"""
Batch PCB Router using Rust-accelerated A* - Routes multiple nets sequentially.

Usage:
    python batch_grid_router.py input.kicad_pcb output.kicad_pcb net1 net2 net3 ...

Requires the Rust router module. Build it with:
    cd rust_router && cargo build --release
    cp target/release/grid_router.dll grid_router.pyd  # Windows
    cp target/release/libgrid_router.so grid_router.so  # Linux
"""

import sys
import os
import time
import fnmatch
import random
from typing import List, Optional, Tuple, Dict
from dataclasses import dataclass, field

from kicad_parser import (
    parse_kicad_pcb, PCBData, Segment, Via,
    auto_detect_bga_exclusion_zones, find_components_by_type, detect_package_type
)
from kicad_writer import generate_segment_sexpr, generate_via_sexpr

# Add rust_router directory to path for importing the compiled module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))

# Import Rust router
try:
    import grid_router
    from grid_router import GridObstacleMap, GridRouter
    version = getattr(grid_router, '__version__', 'unknown')
    print(f"Using Rust router v{version}")
except ImportError as e:
    print("ERROR: Rust router module not found!")
    print("Build it with:")
    print("  cd rust_router && cargo build --release")
    print("  cp target/release/grid_router.dll grid_router.pyd  # Windows")
    print("  cp target/release/libgrid_router.so grid_router.so  # Linux")
    sys.exit(1)


@dataclass
class GridRouteConfig:
    """Configuration for grid-based routing."""
    track_width: float = 0.1  # mm
    clearance: float = 0.1  # mm between tracks
    via_size: float = 0.3  # mm via outer diameter
    via_drill: float = 0.2  # mm via drill
    grid_step: float = 0.1  # mm grid resolution
    via_cost: int = 25  # grid steps equivalent penalty for via
    layers: List[str] = field(default_factory=lambda: ['F.Cu', 'B.Cu'])
    max_iterations: int = 100000
    heuristic_weight: float = 1.5
    # BGA exclusion zones (auto-detected from PCB) - vias blocked inside these areas
    bga_exclusion_zones: List[Tuple[float, float, float, float]] = field(default_factory=list)
    stub_proximity_radius: float = 1.0  # mm - radius around stubs to penalize
    stub_proximity_cost: float = 3.0  # mm equivalent cost at stub center
    # Direction search order: "forward", "backwards", or "random"
    direction_order: str = "forward"


class GridCoord:
    """Utilities for converting between float (mm) and integer grid coordinates."""
    def __init__(self, grid_step: float = 0.1):
        self.grid_step = grid_step
        self.inv_step = 1.0 / grid_step

    def to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert float mm coordinates to integer grid coordinates."""
        return (round(x * self.inv_step), round(y * self.inv_step))

    def to_float(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert integer grid coordinates to float mm coordinates."""
        return (gx * self.grid_step, gy * self.grid_step)

    def to_grid_dist(self, dist_mm: float) -> int:
        """Convert a distance in mm to grid units."""
        return round(dist_mm * self.inv_step)


def find_connected_groups(segments: List[Segment], tolerance: float = 0.01) -> List[List[Segment]]:
    """Find groups of connected segments using union-find."""
    if not segments:
        return []

    n = len(segments)
    parent = list(range(n))

    def find(x):
        if parent[x] != x:
            parent[x] = find(parent[x])
        return parent[x]

    def union(x, y):
        px, py = find(x), find(y)
        if px != py:
            parent[px] = py

    # Check all pairs for shared endpoints
    for i in range(n):
        for j in range(i + 1, n):
            si, sj = segments[i], segments[j]
            pts_i = [(si.start_x, si.start_y), (si.end_x, si.end_y)]
            pts_j = [(sj.start_x, sj.start_y), (sj.end_x, sj.end_y)]
            for pi in pts_i:
                for pj in pts_j:
                    if abs(pi[0] - pj[0]) < tolerance and abs(pi[1] - pj[1]) < tolerance:
                        union(i, j)
                        break

    # Group segments by their root
    groups: Dict[int, List[Segment]] = {}
    for i in range(n):
        root = find(i)
        if root not in groups:
            groups[root] = []
        groups[root].append(segments[i])

    return list(groups.values())


def get_net_endpoints(pcb_data: PCBData, net_id: int, config: GridRouteConfig) -> Tuple[List, List, str]:
    """
    Find source and target endpoints for a net, considering both segments and pads.

    Returns:
        (sources, targets, error_message)
        - sources: List of (gx, gy, layer_idx, orig_x, orig_y)
        - targets: List of (gx, gy, layer_idx, orig_x, orig_y)
        - error_message: None if successful, otherwise describes why routing can't proceed
    """
    coord = GridCoord(config.grid_step)
    layer_map = {name: idx for idx, name in enumerate(config.layers)}

    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    net_pads = pcb_data.pads_by_net.get(net_id, [])

    # Case 1: Multiple segment groups - use segments as before
    if len(net_segments) >= 2:
        groups = find_connected_groups(net_segments)
        if len(groups) >= 2:
            groups.sort(key=len, reverse=True)
            source_segs = groups[0]
            target_segs = groups[1]

            sources = []
            for seg in source_segs:
                layer_idx = layer_map.get(seg.layer)
                if layer_idx is None:
                    continue
                gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
                gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
                sources.append((gx1, gy1, layer_idx, seg.start_x, seg.start_y))
                if (gx1, gy1) != (gx2, gy2):
                    sources.append((gx2, gy2, layer_idx, seg.end_x, seg.end_y))

            targets = []
            for seg in target_segs:
                layer_idx = layer_map.get(seg.layer)
                if layer_idx is None:
                    continue
                gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
                gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
                targets.append((gx1, gy1, layer_idx, seg.start_x, seg.start_y))
                if (gx1, gy1) != (gx2, gy2):
                    targets.append((gx2, gy2, layer_idx, seg.end_x, seg.end_y))

            if sources and targets:
                return sources, targets, None

    # Case 2: One segment group + unconnected pads
    if len(net_segments) >= 1 and len(net_pads) >= 1:
        groups = find_connected_groups(net_segments)
        if len(groups) == 1:
            # Check if any pad is NOT connected to the segment group
            seg_group = groups[0]
            seg_points = set()
            for seg in seg_group:
                seg_points.add((round(seg.start_x, 3), round(seg.start_y, 3)))
                seg_points.add((round(seg.end_x, 3), round(seg.end_y, 3)))

            unconnected_pads = []
            for pad in net_pads:
                pad_pos = (round(pad.global_x, 3), round(pad.global_y, 3))
                # Check if pad is near any segment point
                connected = False
                for sp in seg_points:
                    if abs(pad_pos[0] - sp[0]) < 0.05 and abs(pad_pos[1] - sp[1]) < 0.05:
                        connected = True
                        break
                if not connected:
                    unconnected_pads.append(pad)

            if unconnected_pads:
                # Use segment endpoints as source, unconnected pad(s) as target
                sources = []
                for seg in seg_group:
                    layer_idx = layer_map.get(seg.layer)
                    if layer_idx is None:
                        continue
                    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
                    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
                    sources.append((gx1, gy1, layer_idx, seg.start_x, seg.start_y))
                    if (gx1, gy1) != (gx2, gy2):
                        sources.append((gx2, gy2, layer_idx, seg.end_x, seg.end_y))

                targets = []
                for pad in unconnected_pads:
                    gx, gy = coord.to_grid(pad.global_x, pad.global_y)
                    for layer in pad.layers:
                        layer_idx = layer_map.get(layer)
                        if layer_idx is not None:
                            targets.append((gx, gy, layer_idx, pad.global_x, pad.global_y))

                if sources and targets:
                    return sources, targets, None

    # Case 3: No segments, just pads - route between pads
    if len(net_segments) == 0 and len(net_pads) >= 2:
        # Use first pad as source, rest as targets
        sources = []
        pad = net_pads[0]
        gx, gy = coord.to_grid(pad.global_x, pad.global_y)
        for layer in pad.layers:
            layer_idx = layer_map.get(layer)
            if layer_idx is not None:
                sources.append((gx, gy, layer_idx, pad.global_x, pad.global_y))

        targets = []
        for pad in net_pads[1:]:
            gx, gy = coord.to_grid(pad.global_x, pad.global_y)
            for layer in pad.layers:
                layer_idx = layer_map.get(layer)
                if layer_idx is not None:
                    targets.append((gx, gy, layer_idx, pad.global_x, pad.global_y))

        if sources and targets:
            return sources, targets, None

    # Case 4: Single segment, check if it connects two pads already
    if len(net_segments) == 1 and len(net_pads) >= 2:
        # Segment already connects pads - nothing to route
        return [], [], "Net has 1 segment connecting pads - already routed"

    # Determine why we can't route
    if len(net_segments) == 0 and len(net_pads) < 2:
        return [], [], f"Net has no segments and only {len(net_pads)} pad(s) - need at least 2 endpoints"
    if len(net_segments) >= 1:
        groups = find_connected_groups(net_segments)
        if len(groups) == 1:
            return [], [], "Net segments are already connected (single group) with no unconnected pads"

    return [], [], f"Cannot determine endpoints: {len(net_segments)} segments, {len(net_pads)} pads"


def get_stub_endpoints(pcb_data: PCBData, net_ids: List[int]) -> List[Tuple[float, float]]:
    """Get centroid positions of unrouted net stubs for proximity avoidance."""
    stubs = []
    for net_id in net_ids:
        net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
        if len(net_segments) < 2:
            continue
        groups = find_connected_groups(net_segments)
        if len(groups) < 2:
            continue
        for group in groups:
            points = []
            for seg in group:
                points.append((seg.start_x, seg.start_y))
                points.append((seg.end_x, seg.end_y))
            if points:
                cx = sum(p[0] for p in points) / len(points)
                cy = sum(p[1] for p in points) / len(points)
                stubs.append((cx, cy))
    return stubs


def get_net_stub_centroids(pcb_data: PCBData, net_id: int) -> List[Tuple[float, float]]:
    """
    Get centroids of each connected stub group for a net.
    Returns list of (x, y) centroids, typically 2 for a 2-point net.
    """
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    if len(net_segments) < 2:
        return []
    groups = find_connected_groups(net_segments)
    if len(groups) < 2:
        return []

    centroids = []
    for group in groups:
        points = []
        for seg in group:
            points.append((seg.start_x, seg.start_y))
            points.append((seg.end_x, seg.end_y))
        if points:
            cx = sum(p[0] for p in points) / len(points)
            cy = sum(p[1] for p in points) / len(points)
            centroids.append((cx, cy))
    return centroids


def compute_mps_net_ordering(pcb_data: PCBData, net_ids: List[int],
                              center: Tuple[float, float] = None) -> List[int]:
    """
    Compute optimal net routing order using Maximum Planar Subset (MPS) algorithm.

    The MPS approach identifies crossing conflicts between nets and orders them
    so that non-conflicting nets are routed first. This reduces routing failures
    caused by earlier routes blocking later ones.

    Algorithm:
    1. For each net, find its two stub endpoint centroids
    2. Project all endpoints onto a circular boundary centered on the routing region
    3. Assign each endpoint an angular position on the boundary
    4. Detect crossing conflicts: nets A and B cross if their endpoints alternate
       on the boundary (A1, B1, A2, B2 or B1, A1, B2, A2 ordering)
    5. Build a conflict graph where edges connect crossing nets
    6. Use greedy algorithm: repeatedly select net with fewest active conflicts,
       add to result, and remove its neighbors from consideration for this round
    7. Continue until all nets are ordered (multiple rounds/layers)

    Args:
        pcb_data: PCB data with segments
        net_ids: List of net IDs to order
        center: Optional center point for angular projection (auto-computed if None)

    Returns:
        Ordered list of net IDs, with least-conflicting nets first
    """
    import math

    # Step 1: Get stub centroids for each net
    net_endpoints = {}  # net_id -> [(x1, y1), (x2, y2)]
    for net_id in net_ids:
        centroids = get_net_stub_centroids(pcb_data, net_id)
        if len(centroids) >= 2:
            # Take the first two centroids (source and target)
            net_endpoints[net_id] = centroids[:2]

    if not net_endpoints:
        print("MPS: No nets with valid stub endpoints found")
        return list(net_ids)

    # Step 2: Compute center if not provided (centroid of all endpoints)
    if center is None:
        all_points = []
        for endpoints in net_endpoints.values():
            all_points.extend(endpoints)
        if all_points:
            center = (
                sum(p[0] for p in all_points) / len(all_points),
                sum(p[1] for p in all_points) / len(all_points)
            )
        else:
            center = (0, 0)

    # Step 3: Compute angular position for each endpoint
    def angle_from_center(point: Tuple[float, float]) -> float:
        """Compute angle from center to point in radians [0, 2*pi)."""
        dx = point[0] - center[0]
        dy = point[1] - center[1]
        ang = math.atan2(dy, dx)
        if ang < 0:
            ang += 2 * math.pi
        return ang

    # For each net, get angles of both endpoints and normalize order
    net_angles = {}  # net_id -> (angle1, angle2) where angle1 < angle2
    for net_id, endpoints in net_endpoints.items():
        a1 = angle_from_center(endpoints[0])
        a2 = angle_from_center(endpoints[1])
        # Normalize: always store with smaller angle first
        if a1 > a2:
            a1, a2 = a2, a1
        net_angles[net_id] = (a1, a2)

    # Step 4: Detect crossing conflicts
    # Two nets cross if their intervals on the circle interleave
    # Net A with (a1, a2) and Net B with (b1, b2) cross if:
    #   a1 < b1 < a2 < b2  OR  b1 < a1 < b2 < a2
    def nets_cross(net_a: int, net_b: int) -> bool:
        """Check if two nets have crossing paths on the circular boundary."""
        a1, a2 = net_angles[net_a]
        b1, b2 = net_angles[net_b]

        # Check interleaving: one net's interval partially overlaps the other's
        # a1 < b1 < a2 < b2 means A starts, B starts, A ends, B ends = crossing
        if a1 < b1 < a2 < b2:
            return True
        if b1 < a1 < b2 < a2:
            return True
        return False

    # Build conflict graph
    net_list = list(net_angles.keys())
    conflicts = {net_id: set() for net_id in net_list}

    for i, net_a in enumerate(net_list):
        for net_b in net_list[i+1:]:
            if nets_cross(net_a, net_b):
                conflicts[net_a].add(net_b)
                conflicts[net_b].add(net_a)

    # Count total conflicts for reporting
    total_conflicts = sum(len(c) for c in conflicts.values()) // 2
    print(f"MPS: {len(net_list)} nets with {total_conflicts} crossing conflicts detected")

    # Step 5: Greedy ordering - repeatedly pick net with fewest active conflicts
    ordered = []
    remaining = set(net_list)
    round_num = 0

    while remaining:
        round_num += 1
        round_winners = []
        round_losers = set()
        active_conflicts = {net_id: len(conflicts[net_id] & remaining)
                           for net_id in remaining}

        # Process this round: pick nets with minimal conflicts
        round_remaining = set(remaining)
        while round_remaining:
            # Find net with minimum active conflicts among round_remaining
            min_conflicts = float('inf')
            best_net = None
            for net_id in round_remaining:
                # Active conflicts = conflicts with nets still in round_remaining
                active = len(conflicts[net_id] & round_remaining)
                if active < min_conflicts:
                    min_conflicts = active
                    best_net = net_id

            if best_net is None:
                break

            # This net wins this round
            round_winners.append(best_net)
            round_remaining.discard(best_net)

            # All its conflicting neighbors in round_remaining become losers
            for loser in conflicts[best_net] & round_remaining:
                round_losers.add(loser)
                round_remaining.discard(loser)

        # Add winners to ordered list, then losers go to next round
        ordered.extend(round_winners)
        remaining = round_losers

        if round_winners:
            net_names = [pcb_data.nets[nid].name for nid in round_winners[:3]]
            suffix = f"... (+{len(round_winners)-3} more)" if len(round_winners) > 3 else ""
            print(f"MPS Round {round_num}: {len(round_winners)} nets selected "
                  f"({', '.join(net_names)}{suffix})")

    # Add any nets that weren't in net_angles (no valid stubs) at the end
    nets_without_stubs = [nid for nid in net_ids if nid not in net_angles]
    if nets_without_stubs:
        print(f"MPS: {len(nets_without_stubs)} nets without valid stubs appended at end")
        ordered.extend(nets_without_stubs)

    return ordered


def build_base_obstacle_map(pcb_data: PCBData, config: GridRouteConfig,
                            nets_to_route: List[int]) -> GridObstacleMap:
    """Build base obstacle map with static obstacles (BGA zones, pads, pre-existing tracks/vias).

    Excludes all nets that will be routed (nets_to_route) - their stubs will be added
    per-net in the routing loop (excluding the current net being routed).
    """
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)
    layer_map = {name: idx for idx, name in enumerate(config.layers)}
    nets_to_route_set = set(nets_to_route)

    obstacles = GridObstacleMap(num_layers)

    # Set BGA exclusion zones - block vias AND tracks on ALL layers
    for zone in config.bga_exclusion_zones:
        min_x, min_y, max_x, max_y = zone
        gmin_x, gmin_y = coord.to_grid(min_x, min_y)
        gmax_x, gmax_y = coord.to_grid(max_x, max_y)
        obstacles.set_bga_zone(gmin_x, gmin_y, gmax_x, gmax_y)
        for layer_idx in range(num_layers):
            for gx in range(gmin_x, gmax_x + 1):
                for gy in range(gmin_y, gmax_y + 1):
                    obstacles.add_blocked_cell(gx, gy, layer_idx)

    # Precompute grid expansions
    expansion_mm = config.track_width / 2 + config.clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_block_mm = config.via_size / 2 + config.track_width / 2 + config.clearance
    via_block_grid = max(1, coord.to_grid_dist(via_block_mm))
    via_track_expansion_grid = max(1, coord.to_grid_dist(config.via_size / 2 + config.track_width / 2 + config.clearance))
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance))

    # Add segments as obstacles (excluding nets we'll route - their stubs added per-net)
    for seg in pcb_data.segments:
        if seg.net_id in nets_to_route_set:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue

        _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_grid, via_block_grid)

    # Add vias as obstacles (excluding nets we'll route)
    for via in pcb_data.vias:
        if via.net_id in nets_to_route_set:
            continue
        _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion_grid, via_via_expansion_grid)

    # Add pads as obstacles (excluding nets we'll route - their pads added per-net)
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id in nets_to_route_set:
            continue
        for pad in pads:
            _add_pad_obstacle(obstacles, pad, coord, layer_map, config)

    return obstacles


def add_net_stubs_as_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                net_id: int, config: GridRouteConfig):
    """Add a net's stub segments as obstacles to the map."""
    coord = GridCoord(config.grid_step)
    layer_map = {name: idx for idx, name in enumerate(config.layers)}

    expansion_mm = config.track_width / 2 + config.clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_block_mm = config.via_size / 2 + config.track_width / 2 + config.clearance
    via_block_grid = max(1, coord.to_grid_dist(via_block_mm))

    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_grid, via_block_grid)


def add_net_pads_as_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                               net_id: int, config: GridRouteConfig):
    """Add a net's pads as obstacles to the map."""
    coord = GridCoord(config.grid_step)
    layer_map = {name: idx for idx, name in enumerate(config.layers)}

    pads = pcb_data.pads_by_net.get(net_id, [])
    for pad in pads:
        _add_pad_obstacle(obstacles, pad, coord, layer_map, config)


def add_net_vias_as_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                               net_id: int, config: GridRouteConfig):
    """Add a net's vias as obstacles to the map."""
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)

    via_track_expansion_grid = max(1, coord.to_grid_dist(config.via_size / 2 + config.track_width / 2 + config.clearance))
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance))

    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion_grid, via_via_expansion_grid)


def _add_segment_obstacle(obstacles: GridObstacleMap, seg, coord: GridCoord,
                          layer_idx: int, expansion_grid: int, via_block_grid: int):
    """Add a segment as obstacle to the map."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    # Bresenham line with expansion
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1
    err = dx - dy

    gx, gy = gx1, gy1
    while True:
        for ex in range(-expansion_grid, expansion_grid + 1):
            for ey in range(-expansion_grid, expansion_grid + 1):
                obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
        for ex in range(-via_block_grid, via_block_grid + 1):
            for ey in range(-via_block_grid, via_block_grid + 1):
                if ex*ex + ey*ey <= via_block_grid * via_block_grid:
                    obstacles.add_blocked_via(gx + ex, gy + ey)

        if gx == gx2 and gy == gy2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            gx += sx
        if e2 < dx:
            err += dx
            gy += sy


def _add_via_obstacle(obstacles: GridObstacleMap, via, coord: GridCoord,
                      num_layers: int, via_track_expansion_grid: int, via_via_expansion_grid: int):
    """Add a via as obstacle to the map."""
    gx, gy = coord.to_grid(via.x, via.y)
    # Block cells for track routing
    for ex in range(-via_track_expansion_grid, via_track_expansion_grid + 1):
        for ey in range(-via_track_expansion_grid, via_track_expansion_grid + 1):
            if ex*ex + ey*ey <= via_track_expansion_grid * via_track_expansion_grid:
                for layer_idx in range(num_layers):
                    obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
    # Block cells for via placement
    for ex in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
        for ey in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
            if ex*ex + ey*ey <= via_via_expansion_grid * via_via_expansion_grid:
                obstacles.add_blocked_via(gx + ex, gy + ey)


def _add_pad_obstacle(obstacles: GridObstacleMap, pad, coord: GridCoord,
                      layer_map: Dict[str, int], config: GridRouteConfig):
    """Add a pad as obstacle to the map."""
    gx, gy = coord.to_grid(pad.global_x, pad.global_y)

    # Rectangular expansion for track clearance
    half_x_mm = pad.size_x / 2 + config.clearance
    half_y_mm = pad.size_y / 2 + config.clearance
    expand_x = coord.to_grid_dist(half_x_mm)
    expand_y = coord.to_grid_dist(half_y_mm)

    for ex in range(-expand_x, expand_x + 1):
        for ey in range(-expand_y, expand_y + 1):
            for layer in pad.layers:
                layer_idx = layer_map.get(layer)
                if layer_idx is not None:
                    obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)

    # Via blocking near pads
    if 'F.Cu' in pad.layers or 'B.Cu' in pad.layers:
        via_clear_mm = config.via_size / 2 + config.clearance
        via_expand_x = int((pad.size_x / 2 + via_clear_mm) / config.grid_step)
        via_expand_y = int((pad.size_y / 2 + via_clear_mm) / config.grid_step)
        for ex in range(-via_expand_x, via_expand_x + 1):
            for ey in range(-via_expand_y, via_expand_y + 1):
                obstacles.add_blocked_via(gx + ex, gy + ey)


def add_routed_path_obstacles(obstacles: GridObstacleMap, path: List[Tuple[int, int, int]],
                               config: GridRouteConfig):
    """Add a newly routed path as obstacles to the map."""
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)

    expansion_mm = config.track_width / 2 + config.clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_block_mm = config.via_size / 2 + config.track_width / 2 + config.clearance
    via_block_grid = max(1, coord.to_grid_dist(via_block_mm))
    via_track_expansion_grid = max(1, coord.to_grid_dist(config.via_size / 2 + config.track_width / 2 + config.clearance))
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance))

    for i in range(len(path) - 1):
        gx1, gy1, layer1 = path[i]
        gx2, gy2, layer2 = path[i + 1]

        if layer1 != layer2:
            # Via - add via obstacle
            for ex in range(-via_track_expansion_grid, via_track_expansion_grid + 1):
                for ey in range(-via_track_expansion_grid, via_track_expansion_grid + 1):
                    if ex*ex + ey*ey <= via_track_expansion_grid * via_track_expansion_grid:
                        for layer_idx in range(num_layers):
                            obstacles.add_blocked_cell(gx1 + ex, gy1 + ey, layer_idx)
            for ex in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
                for ey in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
                    if ex*ex + ey*ey <= via_via_expansion_grid * via_via_expansion_grid:
                        obstacles.add_blocked_via(gx1 + ex, gy1 + ey)
        else:
            # Segment on same layer - add track obstacle using Bresenham
            dx = abs(gx2 - gx1)
            dy = abs(gy2 - gy1)
            sx = 1 if gx1 < gx2 else -1
            sy = 1 if gy1 < gy2 else -1
            err = dx - dy

            gx, gy = gx1, gy1
            while True:
                for ex in range(-expansion_grid, expansion_grid + 1):
                    for ey in range(-expansion_grid, expansion_grid + 1):
                        obstacles.add_blocked_cell(gx + ex, gy + ey, layer1)
                for ex in range(-via_block_grid, via_block_grid + 1):
                    for ey in range(-via_block_grid, via_block_grid + 1):
                        if ex*ex + ey*ey <= via_block_grid * via_block_grid:
                            obstacles.add_blocked_via(gx + ex, gy + ey)

                if gx == gx2 and gy == gy2:
                    break
                e2 = 2 * err
                if e2 > -dy:
                    err -= dy
                    gx += sx
                if e2 < dx:
                    err += dx
                    gy += sy


def add_stub_proximity_costs(obstacles: GridObstacleMap, unrouted_stubs: List[Tuple[float, float]],
                              config: GridRouteConfig):
    """Add stub proximity costs to the obstacle map."""
    coord = GridCoord(config.grid_step)
    stub_radius_grid = coord.to_grid_dist(config.stub_proximity_radius)
    stub_cost_grid = int(config.stub_proximity_cost * 1000 / config.grid_step)

    for stub_x, stub_y in unrouted_stubs:
        gcx, gcy = coord.to_grid(stub_x, stub_y)
        for dx in range(-stub_radius_grid, stub_radius_grid + 1):
            for dy in range(-stub_radius_grid, stub_radius_grid + 1):
                dist_sq = dx * dx + dy * dy
                if dist_sq <= stub_radius_grid * stub_radius_grid:
                    dist = (dist_sq ** 0.5)
                    proximity = 1.0 - (dist / stub_radius_grid) if stub_radius_grid > 0 else 1.0
                    cost = int(proximity * stub_cost_grid)
                    obstacles.set_stub_proximity(gcx + dx, gcy + dy, cost)


def build_obstacle_map(pcb_data: PCBData, config: GridRouteConfig,
                       exclude_net_id: int, unrouted_stubs: Optional[List[Tuple[float, float]]] = None) -> GridObstacleMap:
    """Build Rust obstacle map from PCB data (legacy function for compatibility)."""
    # Build base map excluding just this net
    obstacles = build_base_obstacle_map(pcb_data, config, [exclude_net_id])

    # Add stub proximity costs
    if unrouted_stubs:
        add_stub_proximity_costs(obstacles, unrouted_stubs, config)

    return obstacles


def route_net(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
              unrouted_stubs: Optional[List[Tuple[float, float]]] = None) -> Optional[dict]:
    """Route a single net using the Rust router."""
    # Find endpoints (segments or pads)
    sources, targets, error = get_net_endpoints(pcb_data, net_id, config)
    if error:
        print(f"  {error}")
        return None

    if not sources or not targets:
        print(f"  No valid source/target endpoints found")
        return None

    coord = GridCoord(config.grid_step)
    layer_names = config.layers

    # Extract grid-only coords for routing
    sources_grid = [(s[0], s[1], s[2]) for s in sources]
    targets_grid = [(t[0], t[1], t[2]) for t in targets]

    # Build obstacles
    obstacles = build_obstacle_map(pcb_data, config, net_id, unrouted_stubs)

    # Add source and target positions as allowed cells to override BGA zone blocking
    # This only affects BGA zone blocking, not regular obstacle blocking (tracks, stubs, pads)
    allow_radius = 10
    for gx, gy, _ in sources_grid + targets_grid:
        for dx in range(-allow_radius, allow_radius + 1):
            for dy in range(-allow_radius, allow_radius + 1):
                obstacles.add_allowed_cell(gx + dx, gy + dy)

    # Mark exact source/target cells so routing can start/end there even if blocked by
    # adjacent track expansion (but NOT blocked by BGA zones - use allowed_cells for that)
    for gx, gy, _ in sources_grid + targets_grid:
        obstacles.add_source_target_cell(gx, gy)

    router = GridRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight)

    # Determine direction order
    if config.direction_order == "random":
        start_backwards = random.choice([True, False])
    elif config.direction_order == "backwards":
        start_backwards = True
    else:  # "forward" or default
        start_backwards = False

    # Set up first and second direction based on order
    if start_backwards:
        first_sources, first_targets = targets_grid, sources_grid
        second_sources, second_targets = sources_grid, targets_grid
        first_label, second_label = "backwards", "forward"
    else:
        first_sources, first_targets = sources_grid, targets_grid
        second_sources, second_targets = targets_grid, sources_grid
        first_label, second_label = "forward", "backwards"

    # Try first direction, then second if first fails
    reversed_path = False
    total_iterations = 0

    path, iterations = router.route_multi(obstacles, first_sources, first_targets, config.max_iterations)
    total_iterations += iterations

    if path is None:
        print(f"No route found after {iterations} iterations ({first_label}), trying {second_label}...")
        path, iterations = router.route_multi(obstacles, second_sources, second_targets, config.max_iterations)
        total_iterations += iterations
        if path is not None:
            reversed_path = not start_backwards  # True if we ended up going backwards

    if path is None:
        print(f"No route found after {total_iterations} iterations (both directions)")
        return None

    print(f"Route found in {total_iterations} iterations, path length: {len(path)}")

    # If path was found in reverse direction, swap sources/targets for connection logic
    if reversed_path:
        sources, targets = targets, sources

    # Find which source/target the path actually connects to
    path_start = path[0]
    path_end = path[-1]

    start_original = None
    for s in sources:
        if s[0] == path_start[0] and s[1] == path_start[1] and s[2] == path_start[2]:
            start_original = (s[3], s[4], layer_names[s[2]])
            break

    end_original = None
    for t in targets:
        if t[0] == path_end[0] and t[1] == path_end[1] and t[2] == path_end[2]:
            end_original = (t[3], t[4], layer_names[t[2]])
            break

    # Convert path to segments and vias
    new_segments = []
    new_vias = []

    # Add connecting segment from original start to first path point if needed
    if start_original:
        first_grid_x, first_grid_y = coord.to_float(path_start[0], path_start[1])
        orig_x, orig_y, orig_layer = start_original
        if abs(orig_x - first_grid_x) > 0.001 or abs(orig_y - first_grid_y) > 0.001:
            seg = Segment(
                start_x=orig_x, start_y=orig_y,
                end_x=first_grid_x, end_y=first_grid_y,
                width=config.track_width,
                layer=orig_layer,
                net_id=net_id
            )
            new_segments.append(seg)

    for i in range(len(path) - 1):
        gx1, gy1, layer1 = path[i]
        gx2, gy2, layer2 = path[i + 1]

        x1, y1 = coord.to_float(gx1, gy1)
        x2, y2 = coord.to_float(gx2, gy2)

        if layer1 != layer2:
            via = Via(
                x=x1, y=y1,
                size=config.via_size,
                drill=config.via_drill,
                layers=[layer_names[layer1], layer_names[layer2]],
                net_id=net_id
            )
            new_vias.append(via)
        else:
            if (x1, y1) != (x2, y2):
                seg = Segment(
                    start_x=x1, start_y=y1,
                    end_x=x2, end_y=y2,
                    width=config.track_width,
                    layer=layer_names[layer1],
                    net_id=net_id
                )
                new_segments.append(seg)

    # Add connecting segment from last path point to original end if needed
    if end_original:
        last_grid_x, last_grid_y = coord.to_float(path_end[0], path_end[1])
        orig_x, orig_y, orig_layer = end_original
        if abs(orig_x - last_grid_x) > 0.001 or abs(orig_y - last_grid_y) > 0.001:
            seg = Segment(
                start_x=last_grid_x, start_y=last_grid_y,
                end_x=orig_x, end_y=orig_y,
                width=config.track_width,
                layer=orig_layer,
                net_id=net_id
            )
            new_segments.append(seg)

    return {
        'new_segments': new_segments,
        'new_vias': new_vias,
        'iterations': total_iterations,
        'path_length': len(path),
        'path': path,  # Include raw path for incremental obstacle updates
    }


def route_net_with_obstacles(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
                              obstacles: GridObstacleMap) -> Optional[dict]:
    """Route a single net using pre-built obstacles (for incremental routing)."""
    # Find endpoints (segments or pads)
    sources, targets, error = get_net_endpoints(pcb_data, net_id, config)
    if error:
        print(f"  {error}")
        return None

    if not sources or not targets:
        print(f"  No valid source/target endpoints found")
        return None

    coord = GridCoord(config.grid_step)
    layer_names = config.layers

    sources_grid = [(s[0], s[1], s[2]) for s in sources]
    targets_grid = [(t[0], t[1], t[2]) for t in targets]

    # Add source and target positions as allowed cells to override BGA zone blocking
    # This only affects BGA zone blocking, not regular obstacle blocking (tracks, stubs, pads)
    allow_radius = 10
    for gx, gy, _ in sources_grid + targets_grid:
        for dx in range(-allow_radius, allow_radius + 1):
            for dy in range(-allow_radius, allow_radius + 1):
                obstacles.add_allowed_cell(gx + dx, gy + dy)

    # Mark exact source/target cells so routing can start/end there even if blocked by
    # adjacent track expansion (but NOT blocked by BGA zones - use allowed_cells for that)
    for gx, gy, _ in sources_grid + targets_grid:
        obstacles.add_source_target_cell(gx, gy)

    router = GridRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight)

    # Determine direction order
    if config.direction_order == "random":
        start_backwards = random.choice([True, False])
    elif config.direction_order == "backwards":
        start_backwards = True
    else:
        start_backwards = False

    if start_backwards:
        first_sources, first_targets = targets_grid, sources_grid
        second_sources, second_targets = sources_grid, targets_grid
        first_label, second_label = "backwards", "forward"
    else:
        first_sources, first_targets = sources_grid, targets_grid
        second_sources, second_targets = targets_grid, sources_grid
        first_label, second_label = "forward", "backwards"

    reversed_path = False
    total_iterations = 0

    path, iterations = router.route_multi(obstacles, first_sources, first_targets, config.max_iterations)
    total_iterations += iterations

    if path is None:
        print(f"No route found after {iterations} iterations ({first_label}), trying {second_label}...")
        path, iterations = router.route_multi(obstacles, second_sources, second_targets, config.max_iterations)
        total_iterations += iterations
        if path is not None:
            reversed_path = not start_backwards

    if path is None:
        print(f"No route found after {total_iterations} iterations (both directions)")
        return None

    print(f"Route found in {total_iterations} iterations, path length: {len(path)}")

    if reversed_path:
        sources, targets = targets, sources

    path_start = path[0]
    path_end = path[-1]

    start_original = None
    for s in sources:
        if s[0] == path_start[0] and s[1] == path_start[1] and s[2] == path_start[2]:
            start_original = (s[3], s[4], layer_names[s[2]])
            break

    end_original = None
    for t in targets:
        if t[0] == path_end[0] and t[1] == path_end[1] and t[2] == path_end[2]:
            end_original = (t[3], t[4], layer_names[t[2]])
            break

    new_segments = []
    new_vias = []

    if start_original:
        first_grid_x, first_grid_y = coord.to_float(path_start[0], path_start[1])
        orig_x, orig_y, orig_layer = start_original
        if abs(orig_x - first_grid_x) > 0.001 or abs(orig_y - first_grid_y) > 0.001:
            seg = Segment(
                start_x=orig_x, start_y=orig_y,
                end_x=first_grid_x, end_y=first_grid_y,
                width=config.track_width,
                layer=orig_layer,
                net_id=net_id
            )
            new_segments.append(seg)

    for i in range(len(path) - 1):
        gx1, gy1, layer1 = path[i]
        gx2, gy2, layer2 = path[i + 1]

        x1, y1 = coord.to_float(gx1, gy1)
        x2, y2 = coord.to_float(gx2, gy2)

        if layer1 != layer2:
            via = Via(
                x=x1, y=y1,
                size=config.via_size,
                drill=config.via_drill,
                layers=[layer_names[layer1], layer_names[layer2]],
                net_id=net_id
            )
            new_vias.append(via)
        else:
            if (x1, y1) != (x2, y2):
                seg = Segment(
                    start_x=x1, start_y=y1,
                    end_x=x2, end_y=y2,
                    width=config.track_width,
                    layer=layer_names[layer1],
                    net_id=net_id
                )
                new_segments.append(seg)

    if end_original:
        last_grid_x, last_grid_y = coord.to_float(path_end[0], path_end[1])
        orig_x, orig_y, orig_layer = end_original
        if abs(orig_x - last_grid_x) > 0.001 or abs(orig_y - last_grid_y) > 0.001:
            seg = Segment(
                start_x=last_grid_x, start_y=last_grid_y,
                end_x=orig_x, end_y=orig_y,
                width=config.track_width,
                layer=orig_layer,
                net_id=net_id
            )
            new_segments.append(seg)

    return {
        'new_segments': new_segments,
        'new_vias': new_vias,
        'iterations': total_iterations,
        'path_length': len(path),
        'path': path,
    }


def add_route_to_pcb_data(pcb_data: PCBData, result: dict) -> None:
    """Add routed segments and vias to PCB data for subsequent routes to see."""
    for seg in result['new_segments']:
        pcb_data.segments.append(seg)
    for via in result['new_vias']:
        pcb_data.vias.append(via)


def batch_route(input_file: str, output_file: str, net_names: List[str],
                layers: List[str] = None,
                bga_exclusion_zones: Optional[List[Tuple[float, float, float, float]]] = None,
                direction_order: str = None,
                ordering_strategy: str = "inside_out",
                disable_bga_zones: bool = False) -> Tuple[int, int, float]:
    """
    Route multiple nets using the Rust router.

    Args:
        input_file: Path to input KiCad PCB file
        output_file: Path to output KiCad PCB file
        net_names: List of net names to route
        layers: List of copper layers to route on (must be specified - cannot auto-detect
                which layers are ground planes vs signal layers)
        bga_exclusion_zones: Optional list of BGA exclusion zones (auto-detected if None)
        direction_order: Direction search order - "forward", "backwards", or "random"
                        (None = use GridRouteConfig default)
        ordering_strategy: Net ordering strategy:
            - "inside_out": Sort BGA nets by distance from BGA center (default)
            - "mps": Use Maximum Planar Subset algorithm to minimize crossing conflicts
            - "original": Keep nets in original order

    Returns:
        (successful_count, failed_count, total_time)
    """
    print(f"Loading {input_file}...")
    pcb_data = parse_kicad_pcb(input_file)

    # Layers must be specified - we can't auto-detect which are ground planes
    if layers is None:
        layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']  # Default 4-layer signal stack
    print(f"Using {len(layers)} routing layers: {layers}")

    # Auto-detect BGA exclusion zones if not specified
    if disable_bga_zones:
        bga_exclusion_zones = []
        print("BGA exclusion zones disabled")
    elif bga_exclusion_zones is None:
        bga_exclusion_zones = auto_detect_bga_exclusion_zones(pcb_data, margin=0.5)
        if bga_exclusion_zones:
            bga_components = find_components_by_type(pcb_data, 'BGA')
            print(f"Auto-detected {len(bga_exclusion_zones)} BGA exclusion zone(s):")
            for i, (fp, zone) in enumerate(zip(bga_components, bga_exclusion_zones)):
                print(f"  {fp.reference}: ({zone[0]:.1f}, {zone[1]:.1f}) to ({zone[2]:.1f}, {zone[3]:.1f})")
        else:
            print("No BGA components detected - no exclusion zones needed")

    config_kwargs = dict(
        track_width=0.1,
        clearance=0.1,
        via_size=0.3,
        via_drill=0.2,
        grid_step=0.1,
        via_cost=25,
        layers=layers,
        max_iterations=100000,
        heuristic_weight=1.5,
        bga_exclusion_zones=bga_exclusion_zones,
        stub_proximity_radius=1.0,
        stub_proximity_cost=3.0,
    )
    if direction_order is not None:
        config_kwargs['direction_order'] = direction_order
    config = GridRouteConfig(**config_kwargs)

    # Find net IDs - check both pcb.nets and pads_by_net
    net_ids = []
    for net_name in net_names:
        net_id = None
        # First check pcb.nets
        for nid, net in pcb_data.nets.items():
            if net.name == net_name:
                net_id = nid
                break
        # If not found, check pads_by_net (for nets not in pcb.nets)
        if net_id is None:
            for nid, pads in pcb_data.pads_by_net.items():
                for pad in pads:
                    if pad.net_name == net_name:
                        net_id = nid
                        break
                if net_id is not None:
                    break
        if net_id is None:
            print(f"Warning: Net '{net_name}' not found, skipping")
        else:
            net_ids.append((net_name, net_id))

    if not net_ids:
        print("No valid nets to route!")
        return 0, 0, 0.0

    # Apply net ordering strategy
    if ordering_strategy == "mps":
        # Use Maximum Planar Subset algorithm to minimize crossing conflicts
        print(f"\nUsing MPS ordering strategy...")
        all_net_ids = [nid for _, nid in net_ids]
        ordered_ids = compute_mps_net_ordering(pcb_data, all_net_ids)
        # Rebuild net_ids in the new order
        id_to_name = {nid: name for name, nid in net_ids}
        net_ids = [(id_to_name[nid], nid) for nid in ordered_ids if nid in id_to_name]

    elif ordering_strategy == "inside_out" and bga_exclusion_zones:
        # Sort nets inside-out from BGA center(s) for better escape routing
        # Only applies to nets that have pads inside a BGA zone
        def pad_in_bga_zone(pad):
            """Check if a pad is inside any BGA zone."""
            for zone in bga_exclusion_zones:
                if zone[0] <= pad.global_x <= zone[2] and zone[1] <= pad.global_y <= zone[3]:
                    return True
            return False

        def get_min_distance_to_bga_center(net_id):
            """Get minimum distance from any BGA pad of this net to its BGA center."""
            pads = pcb_data.pads_by_net.get(net_id, [])
            if not pads:
                return float('inf')

            min_dist = float('inf')
            for zone in bga_exclusion_zones:
                center_x = (zone[0] + zone[2]) / 2
                center_y = (zone[1] + zone[3]) / 2
                for pad in pads:
                    # Only consider pads that are inside this BGA zone
                    if zone[0] <= pad.global_x <= zone[2] and zone[1] <= pad.global_y <= zone[3]:
                        dist = ((pad.global_x - center_x) ** 2 + (pad.global_y - center_y) ** 2) ** 0.5
                        min_dist = min(min_dist, dist)
            return min_dist

        # Separate BGA nets from non-BGA nets
        bga_nets = []
        non_bga_nets = []
        for net_name, net_id in net_ids:
            pads = pcb_data.pads_by_net.get(net_id, [])
            has_bga_pad = any(pad_in_bga_zone(pad) for pad in pads)
            if has_bga_pad:
                bga_nets.append((net_name, net_id))
            else:
                non_bga_nets.append((net_name, net_id))

        # Sort BGA nets inside-out, keep non-BGA nets in original order
        bga_nets.sort(key=lambda x: get_min_distance_to_bga_center(x[1]))
        net_ids = bga_nets + non_bga_nets

        if bga_nets:
            print(f"\nSorted {len(bga_nets)} BGA nets inside-out ({len(non_bga_nets)} non-BGA nets unchanged)")

    elif ordering_strategy == "original":
        print(f"\nUsing original net order (no sorting)")

    print(f"\nRouting {len(net_ids)} nets...")
    print("=" * 60)

    results = []
    successful = 0
    failed = 0
    total_time = 0
    total_iterations = 0

    # Build base obstacle map once (excludes all nets we're routing)
    all_net_ids_to_route = [nid for _, nid in net_ids]
    print("Building base obstacle map...")
    base_start = time.time()
    base_obstacles = build_base_obstacle_map(pcb_data, config, all_net_ids_to_route)
    base_elapsed = time.time() - base_start
    print(f"Base obstacle map built in {base_elapsed:.2f}s")

    # Track which nets have been routed (their segments/vias are now in pcb_data)
    routed_net_ids = []
    remaining_net_ids = list(all_net_ids_to_route)

    for i, (net_name, net_id) in enumerate(net_ids):
        print(f"\n[{i+1}/{len(net_ids)}] Routing {net_name} (id={net_id})")
        print("-" * 40)

        start_time = time.time()

        # Clone base obstacles
        obstacles = base_obstacles.clone()

        # Add previously routed nets' segments/vias/pads as obstacles (from pcb_data)
        for routed_id in routed_net_ids:
            add_net_stubs_as_obstacles(obstacles, pcb_data, routed_id, config)
            add_net_vias_as_obstacles(obstacles, pcb_data, routed_id, config)
            add_net_pads_as_obstacles(obstacles, pcb_data, routed_id, config)

        # Add other unrouted nets' stubs, vias, and pads as obstacles (not the current net)
        other_unrouted = [nid for nid in remaining_net_ids if nid != net_id]
        for other_net_id in other_unrouted:
            add_net_stubs_as_obstacles(obstacles, pcb_data, other_net_id, config)
            add_net_vias_as_obstacles(obstacles, pcb_data, other_net_id, config)
            add_net_pads_as_obstacles(obstacles, pcb_data, other_net_id, config)

        # Add stub proximity costs for remaining unrouted nets
        unrouted_stubs = get_stub_endpoints(pcb_data, other_unrouted)
        if unrouted_stubs:
            add_stub_proximity_costs(obstacles, unrouted_stubs, config)

        # Route the net using the prepared obstacles
        result = route_net_with_obstacles(pcb_data, net_id, config, obstacles)
        elapsed = time.time() - start_time
        total_time += elapsed

        if result:
            print(f"  SUCCESS: {len(result['new_segments'])} segments, {len(result['new_vias'])} vias, {result['iterations']} iterations ({elapsed:.2f}s)")
            results.append(result)
            successful += 1
            total_iterations += result['iterations']
            add_route_to_pcb_data(pcb_data, result)
            remaining_net_ids.remove(net_id)
            routed_net_ids.append(net_id)
        else:
            print(f"  FAILED: Could not find route ({elapsed:.2f}s)")
            failed += 1

    print("\n" + "=" * 60)
    print(f"Routing complete: {successful} successful, {failed} failed")
    print(f"Total time: {total_time:.2f}s")
    print(f"Total iterations: {total_iterations}")

    if results:
        print(f"\nWriting output to {output_file}...")
        with open(input_file, 'r', encoding='utf-8') as f:
            content = f.read()

        routing_text = ""
        for result in results:
            for seg in result['new_segments']:
                routing_text += generate_segment_sexpr(
                    (seg.start_x, seg.start_y), (seg.end_x, seg.end_y),
                    seg.width, seg.layer, seg.net_id
                ) + "\n"
            for via in result['new_vias']:
                routing_text += generate_via_sexpr(
                    via.x, via.y, via.size, via.drill,
                    via.layers, via.net_id
                ) + "\n"

        last_paren = content.rfind(')')
        new_content = content[:last_paren] + '\n' + routing_text + '\n' + content[last_paren:]

        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(new_content)

        print(f"Successfully wrote {output_file}")

    return successful, failed, total_time


def expand_net_patterns(pcb_data: PCBData, patterns: List[str]) -> List[str]:
    """
    Expand wildcard patterns to matching net names.

    Patterns can include * and ? wildcards (fnmatch style).
    Example: "Net-(U2A-DATA_*)" matches Net-(U2A-DATA_0), Net-(U2A-DATA_1), etc.

    Returns list of unique net names in sorted order for patterns,
    preserving order of non-pattern names.
    """
    # Collect net names from both pcb.nets and pads_by_net
    all_net_names = set(net.name for net in pcb_data.nets.values())
    # Also include net names from pads (for nets not in pcb.nets)
    for pads in pcb_data.pads_by_net.values():
        for pad in pads:
            if pad.net_name:
                all_net_names.add(pad.net_name)
                break  # Only need one pad's net_name per net
    all_net_names = list(all_net_names)
    result = []
    seen = set()

    for pattern in patterns:
        if '*' in pattern or '?' in pattern:
            # It's a wildcard pattern - find all matching nets
            matches = sorted([name for name in all_net_names if fnmatch.fnmatch(name, pattern)])
            if not matches:
                print(f"Warning: Pattern '{pattern}' matched no nets")
            else:
                print(f"Pattern '{pattern}' matched {len(matches)} nets")
            for name in matches:
                if name not in seen:
                    result.append(name)
                    seen.add(name)
        else:
            # Literal net name
            if pattern not in seen:
                result.append(pattern)
                seen.add(pattern)

    return result


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Batch PCB Router - Routes multiple nets using Rust-accelerated A*",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Wildcard patterns supported:
  "Net-(U2A-DATA_*)"  - matches Net-(U2A-DATA_0), Net-(U2A-DATA_1), etc.
  "Net-(*CLK*)"       - matches any net containing CLK

Examples:
  python batch_grid_router.py fanout_starting_point.kicad_pcb routed.kicad_pcb "Net-(U2A-DATA_*)"
  python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_*)" --ordering mps
"""
    )
    parser.add_argument("input_file", help="Input KiCad PCB file")
    parser.add_argument("output_file", help="Output KiCad PCB file")
    parser.add_argument("net_patterns", nargs="+", help="Net names or wildcard patterns to route")
    parser.add_argument("--ordering", "-o", choices=["inside_out", "mps", "original"],
                        default="inside_out",
                        help="Net ordering strategy: inside_out (default), mps (crossing conflicts), or original")
    parser.add_argument("--direction", "-d", choices=["forward", "backwards", "random"],
                        default=None,
                        help="Direction search order for each net route")
    parser.add_argument("--no-bga-zones", action="store_true",
                        help="Disable BGA exclusion zone detection (allows routing through BGA areas)")

    args = parser.parse_args()

    # Load PCB to expand wildcards
    print(f"Loading {args.input_file} to expand net patterns...")
    pcb_data = parse_kicad_pcb(args.input_file)
    net_names = expand_net_patterns(pcb_data, args.net_patterns)

    if not net_names:
        print("No nets matched the given patterns!")
        sys.exit(1)

    print(f"Routing {len(net_names)} nets: {net_names[:5]}{'...' if len(net_names) > 5 else ''}")

    batch_route(args.input_file, args.output_file, net_names,
                direction_order=args.direction,
                ordering_strategy=args.ordering,
                disable_bga_zones=args.no_bga_zones)
