"""
A* PCB Router - Octilinear routing with multi-layer support.

Routes between two points on a PCB, avoiding obstacles and using vias
to change layers when needed.
"""

import heapq
import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional, Set
from kicad_parser import parse_kicad_pcb, PCBData, Segment, Via, Pad


@dataclass
class RouteConfig:
    """Configuration for routing."""
    track_width: float = 0.1  # mm
    clearance: float = 0.1  # mm between tracks
    via_size: float = 0.4  # mm via outer diameter
    via_drill: float = 0.2  # mm via drill
    grid_step: float = 0.1  # mm grid resolution
    via_cost: float = 0.5  # cost penalty for via (in mm equivalent)
    layers: List[str] = field(default_factory=lambda: ['F.Cu', 'B.Cu'])
    max_iterations: int = 100000  # max A* iterations before giving up
    # Escape zone parameters - for escaping congested stub areas
    escape_radius: float = 2.0  # mm - distance from start within which reduced clearance applies
    escape_clearance: float = 0.02  # mm - minimal clearance in escape zone (just avoid touching)
    # Heuristic weight: 1.0 = standard A*, <1.0 = more Dijkstra-like (explores broader paths)
    # Lower values help find paths around obstacles that require detours away from the goal
    heuristic_weight: float = 0.1
    # Jump Point Search: dramatically speeds up A* by jumping over empty space
    use_jps: bool = True
    # Coarse-to-fine routing: first find path with larger grid, then refine
    use_coarse_first: bool = True
    coarse_grid_step: float = 0.5  # mm - coarse grid for initial pathfinding
    # BGA exclusion zone - vias inside this box are ignored (they're fanout vias)
    # and routing is blocked from entering this area (hard wall at boundary)
    # Format: (min_x, min_y, max_x, max_y) or None to disable
    bga_exclusion_zone: Optional[Tuple[float, float, float, float]] = None
    # Obstacle repulsion - adds cost when routing near obstacles to leave space for future routes
    # repulsion_distance: distance (mm) within which repulsion applies (beyond clearance)
    # repulsion_cost: maximum cost penalty (in mm equivalent) when touching the clearance boundary
    # Cost falls off linearly from repulsion_cost at clearance to 0 at repulsion_distance
    repulsion_distance: float = 2.0  # mm - repulsion field extends this far beyond clearance
    repulsion_cost: float = 2.0  # mm equivalent cost at clearance boundary


class State:
    """A state in the A* search: position + layer.

    Uses pre-computed grid key for fast hashing/equality.
    Coordinates are stored as-is but compared via grid key.
    """
    __slots__ = ('x', 'y', 'layer', '_grid_key')

    def __init__(self, x: float, y: float, layer: str):
        self.x = x
        self.y = y
        self.layer = layer
        # Pre-compute grid key: convert to integer grid units (0.0001mm precision)
        # This avoids repeated round() calls in __hash__ and __eq__
        self._grid_key = (int(x * 10000), int(y * 10000), layer)

    def __hash__(self):
        return hash(self._grid_key)

    def __eq__(self, other):
        return self._grid_key == other._grid_key

    def __repr__(self):
        return f"State({self.x}, {self.y}, {self.layer})"


class ObstacleGrid:
    """
    Spatial index for collision detection.
    Uses a simple grid-based approach for efficiency.
    """

    def __init__(self, pcb_data: PCBData, config: RouteConfig,
                 exclude_net_id: Optional[int] = None):
        """
        Build obstacle grid from PCB data.

        Args:
            pcb_data: Parsed PCB data
            config: Routing configuration
            exclude_net_id: Net ID to exclude from obstacles (the net being routed)
        """
        self.config = config
        self.pcb_data = pcb_data
        self.exclude_net_id = exclude_net_id

        # Expanded clearance (half track width + clearance)
        self.expansion = config.track_width / 2 + config.clearance

        # Build spatial index per layer
        # Using simple grid cells for quick lookup
        self.cell_size = 1.0  # mm
        self.obstacles_by_layer: Dict[str, Dict[Tuple[int, int], List]] = {}

        for layer in config.layers:
            self.obstacles_by_layer[layer] = {}

        self._build_from_segments()
        self._build_from_vias()
        self._build_from_pads()

    def _grid_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Convert coordinates to grid cell."""
        return (int(x / self.cell_size), int(y / self.cell_size))

    def _add_obstacle(self, layer: str, x1: float, y1: float, x2: float, y2: float,
                      expansion: float):
        """Add a line segment obstacle with expansion."""
        if layer not in self.obstacles_by_layer:
            return

        # Find all cells this segment might touch
        min_x = min(x1, x2) - expansion
        max_x = max(x1, x2) + expansion
        min_y = min(y1, y2) - expansion
        max_y = max(y1, y2) + expansion

        cell_min = self._grid_cell(min_x, min_y)
        cell_max = self._grid_cell(max_x, max_y)

        obstacle = (x1, y1, x2, y2, expansion)

        for cx in range(cell_min[0], cell_max[0] + 1):
            for cy in range(cell_min[1], cell_max[1] + 1):
                cell = (cx, cy)
                if cell not in self.obstacles_by_layer[layer]:
                    self.obstacles_by_layer[layer][cell] = []
                self.obstacles_by_layer[layer][cell].append(('segment', obstacle))

    def _add_circle_obstacle(self, layer: str, x: float, y: float, radius: float):
        """Add a circular obstacle (via, pad)."""
        if layer not in self.obstacles_by_layer:
            return

        cell_min = self._grid_cell(x - radius, y - radius)
        cell_max = self._grid_cell(x + radius, y + radius)

        obstacle = (x, y, radius)

        for cx in range(cell_min[0], cell_max[0] + 1):
            for cy in range(cell_min[1], cell_max[1] + 1):
                cell = (cx, cy)
                if cell not in self.obstacles_by_layer[layer]:
                    self.obstacles_by_layer[layer][cell] = []
                self.obstacles_by_layer[layer][cell].append(('circle', obstacle))

    def _build_from_segments(self):
        """Add existing track segments as obstacles."""
        for seg in self.pcb_data.segments:
            if seg.net_id == self.exclude_net_id:
                continue
            expansion = seg.width / 2 + self.config.clearance
            self._add_obstacle(seg.layer, seg.start_x, seg.start_y,
                             seg.end_x, seg.end_y, expansion)

    def _in_bga_zone(self, x: float, y: float) -> bool:
        """Check if a point is inside the BGA exclusion zone."""
        zone = self.config.bga_exclusion_zone
        if zone is None:
            return False
        min_x, min_y, max_x, max_y = zone
        return min_x <= x <= max_x and min_y <= y <= max_y

    def _segment_crosses_bga_zone(self, x1: float, y1: float, x2: float, y2: float) -> bool:
        """Check if a segment crosses into the BGA exclusion zone."""
        zone = self.config.bga_exclusion_zone
        if zone is None:
            return False

        min_x, min_y, max_x, max_y = zone

        # If both endpoints are outside the zone, check if segment crosses it
        # Use line-box intersection test
        # Check intersection with each edge of the BGA zone

        # Parametric line: P = P1 + t*(P2-P1) where t in [0,1]
        dx = x2 - x1
        dy = y2 - y1

        # Check intersection with vertical edges (x = min_x and x = max_x)
        for edge_x in [min_x, max_x]:
            if abs(dx) > 0.0001:
                t = (edge_x - x1) / dx
                if 0 <= t <= 1:
                    y_at_t = y1 + t * dy
                    if min_y <= y_at_t <= max_y:
                        return True  # Segment crosses into zone

        # Check intersection with horizontal edges (y = min_y and y = max_y)
        for edge_y in [min_y, max_y]:
            if abs(dy) > 0.0001:
                t = (edge_y - y1) / dy
                if 0 <= t <= 1:
                    x_at_t = x1 + t * dx
                    if min_x <= x_at_t <= max_x:
                        return True  # Segment crosses into zone

        return False

    def _build_from_vias(self):
        """Add existing vias as obstacles on all layers."""
        for via in self.pcb_data.vias:
            if via.net_id == self.exclude_net_id:
                continue
            # Skip vias inside BGA exclusion zone - they're fanout vias, not obstacles
            if self._in_bga_zone(via.x, via.y):
                continue
            # Store just the physical radius - clearance is added during collision check
            radius = via.size / 2
            # Vias block all layers
            for layer in self.config.layers:
                self._add_circle_obstacle(layer, via.x, via.y, radius)

    def _build_from_pads(self):
        """Add pads as obstacles (except those on excluded net)."""
        for net_id, pads in self.pcb_data.pads_by_net.items():
            if net_id == self.exclude_net_id:
                continue
            for pad in pads:
                # Store just the physical radius - clearance is added during collision check
                radius = max(pad.size_x, pad.size_y) / 2
                for layer in pad.layers:
                    if layer in self.config.layers:
                        self._add_circle_obstacle(layer, pad.global_x, pad.global_y, radius)

    def _point_to_segment_distance_sq(self, px: float, py: float,
                                        x1: float, y1: float, x2: float, y2: float) -> float:
        """Calculate squared minimum distance from point to line segment.
        Returns squared distance to avoid sqrt overhead."""
        dx = x2 - x1
        dy = y2 - y1
        len_sq = dx * dx + dy * dy

        if len_sq == 0:
            dpx = px - x1
            dpy = py - y1
            return dpx * dpx + dpy * dpy

        t = ((px - x1) * dx + (py - y1) * dy) / len_sq
        if t < 0:
            t = 0
        elif t > 1:
            t = 1

        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        dpx = px - closest_x
        dpy = py - closest_y

        return dpx * dpx + dpy * dpy

    def _point_to_segment_distance(self, px: float, py: float,
                                    x1: float, y1: float, x2: float, y2: float) -> float:
        """Calculate minimum distance from point to line segment."""
        return math.sqrt(self._point_to_segment_distance_sq(px, py, x1, y1, x2, y2))

    @staticmethod
    def _ccw(ax: float, ay: float, bx: float, by: float, cx: float, cy: float) -> bool:
        """Counter-clockwise test for three points."""
        return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)

    def _segment_to_segment_distance_sq(self, ax1: float, ay1: float, ax2: float, ay2: float,
                                         bx1: float, by1: float, bx2: float, by2: float) -> float:
        """Calculate squared minimum distance between two line segments.
        Returns squared distance to avoid sqrt overhead."""
        # Check if segments intersect using inlined ccw tests
        ccw = self._ccw
        if ccw(ax1, ay1, bx1, by1, bx2, by2) != ccw(ax2, ay2, bx1, by1, bx2, by2) and \
           ccw(ax1, ay1, ax2, ay2, bx1, by1) != ccw(ax1, ay1, ax2, ay2, bx2, by2):
            return 0.0  # Segments intersect

        # Otherwise, find minimum squared distance between endpoints and segments
        d1 = self._point_to_segment_distance_sq(ax1, ay1, bx1, by1, bx2, by2)
        d2 = self._point_to_segment_distance_sq(ax2, ay2, bx1, by1, bx2, by2)
        d3 = self._point_to_segment_distance_sq(bx1, by1, ax1, ay1, ax2, ay2)
        d4 = self._point_to_segment_distance_sq(bx2, by2, ax1, ay1, ax2, ay2)

        # Use if/elif chain - often faster than min() for 4 values
        result = d1
        if d2 < result:
            result = d2
        if d3 < result:
            result = d3
        if d4 < result:
            result = d4
        return result

    def _segment_to_segment_distance(self, ax1: float, ay1: float, ax2: float, ay2: float,
                                      bx1: float, by1: float, bx2: float, by2: float) -> float:
        """Calculate minimum distance between two line segments."""
        return math.sqrt(self._segment_to_segment_distance_sq(
            ax1, ay1, ax2, ay2, bx1, by1, bx2, by2))

    def segment_collides(self, x1: float, y1: float, x2: float, y2: float,
                         layer: str, reduced_clearance: float = None) -> bool:
        """Check if a new segment would collide with obstacles.

        Args:
            x1, y1: Start point of segment
            x2, y2: End point of segment
            layer: Layer name
            reduced_clearance: If provided, use this clearance instead of config.clearance
                             (for escaping from congested stub areas)
        """
        # BGA exclusion zone is a HARD BLOCK - never route inside it
        # Check if either endpoint is inside the zone, or if segment crosses into it
        if self._in_bga_zone(x1, y1) or self._in_bga_zone(x2, y2):
            return True  # Blocked - inside BGA zone
        if self._segment_crosses_bga_zone(x1, y1, x2, y2):
            return True  # Blocked - crosses into BGA zone

        if layer not in self.obstacles_by_layer:
            return False

        # Use reduced clearance if provided (for escape from stub areas)
        if reduced_clearance is not None:
            expansion = self.config.track_width / 2 + reduced_clearance
        else:
            expansion = self.expansion

        # Find cells this segment passes through
        if x1 < x2:
            min_x = x1 - expansion
            max_x = x2 + expansion
        else:
            min_x = x2 - expansion
            max_x = x1 + expansion
        if y1 < y2:
            min_y = y1 - expansion
            max_y = y2 + expansion
        else:
            min_y = y2 - expansion
            max_y = y1 + expansion

        cell_min = self._grid_cell(min_x, min_y)
        cell_max = self._grid_cell(max_x, max_y)

        obstacles_by_layer = self.obstacles_by_layer[layer]
        half_track = self.config.track_width / 2

        for cx in range(cell_min[0], cell_max[0] + 1):
            for cy in range(cell_min[1], cell_max[1] + 1):
                cell = (cx, cy)
                if cell not in obstacles_by_layer:
                    continue

                for obs_type, obs_data in obstacles_by_layer[cell]:
                    if obs_type == 'segment':
                        ox1, oy1, ox2, oy2, obs_exp = obs_data
                        # Threshold = half-width of new track + clearance + half-width of obstacle track
                        # obs_exp already includes the obstacle's half-width + clearance
                        # So we just need: our half-width + obs_half_width + clearance (not double clearance)
                        obs_half_width = obs_exp - self.config.clearance  # Extract just the half-width
                        if reduced_clearance is not None:
                            threshold = half_track + reduced_clearance + obs_half_width
                        else:
                            threshold = half_track + self.config.clearance + obs_half_width
                        threshold_sq = threshold * threshold
                        dist_sq = self._segment_to_segment_distance_sq(
                            x1, y1, x2, y2, ox1, oy1, ox2, oy2)
                        if dist_sq < threshold_sq:
                            return True
                    elif obs_type == 'circle':
                        cx_, cy_, radius = obs_data
                        threshold = expansion + radius
                        threshold_sq = threshold * threshold
                        dist_sq = self._point_to_segment_distance_sq(cx_, cy_, x1, y1, x2, y2)
                        if dist_sq < threshold_sq:
                            return True

        return False

    def via_collides(self, x: float, y: float) -> bool:
        """Check if placing a via at (x, y) would collide."""
        via_radius = self.config.via_size / 2 + self.config.clearance

        # Check all layers
        for layer in self.config.layers:
            if layer not in self.obstacles_by_layer:
                continue

            cell_min = self._grid_cell(x - via_radius, y - via_radius)
            cell_max = self._grid_cell(x + via_radius, y + via_radius)

            for cx in range(cell_min[0], cell_max[0] + 1):
                for cy in range(cell_min[1], cell_max[1] + 1):
                    cell = (cx, cy)
                    if cell not in self.obstacles_by_layer[layer]:
                        continue

                    for obs_type, obs_data in self.obstacles_by_layer[layer][cell]:
                        if obs_type == 'segment':
                            ox1, oy1, ox2, oy2, obs_exp = obs_data
                            dist = self._point_to_segment_distance(x, y, ox1, oy1, ox2, oy2)
                            if dist < via_radius + obs_exp:
                                return True
                        elif obs_type == 'circle':
                            cx_, cy_, radius = obs_data
                            dist = math.sqrt((x - cx_)**2 + (y - cy_)**2)
                            if dist < via_radius + radius:
                                return True

        return False

    def add_segment(self, x1: float, y1: float, x2: float, y2: float, layer: str):
        """Add a new segment to obstacles (after routing it)."""
        self._add_obstacle(layer, x1, y1, x2, y2, self.expansion)

    def add_via(self, x: float, y: float):
        """Add a new via to obstacles (after routing it)."""
        radius = self.config.via_size / 2 + self.config.clearance
        for layer in self.config.layers:
            self._add_circle_obstacle(layer, x, y, radius)

    def min_distance_to_obstacle(self, x: float, y: float, layer: str) -> float:
        """
        Calculate minimum distance from point (x, y) to any obstacle on the given layer.
        Returns distance to obstacle edge (not center), accounting for obstacle expansion.
        Returns float('inf') if no obstacles nearby.
        """
        if layer not in self.obstacles_by_layer:
            return float('inf')

        # Search radius - look for obstacles within repulsion distance
        search_radius = self.config.repulsion_distance + self.expansion + 1.0

        cell_min = self._grid_cell(x - search_radius, y - search_radius)
        cell_max = self._grid_cell(x + search_radius, y + search_radius)

        min_dist = float('inf')
        half_track = self.config.track_width / 2

        for cx in range(cell_min[0], cell_max[0] + 1):
            for cy in range(cell_min[1], cell_max[1] + 1):
                cell = (cx, cy)
                if cell not in self.obstacles_by_layer[layer]:
                    continue

                for obs_type, obs_data in self.obstacles_by_layer[layer][cell]:
                    if obs_type == 'segment':
                        ox1, oy1, ox2, oy2, obs_exp = obs_data
                        # Distance from point to segment, minus the expansion (obstacle radius)
                        dist = self._point_to_segment_distance(x, y, ox1, oy1, ox2, oy2)
                        # Subtract our track half-width and obstacle expansion to get edge-to-edge
                        edge_dist = dist - half_track - obs_exp + self.config.clearance
                        if edge_dist < min_dist:
                            min_dist = edge_dist
                    elif obs_type == 'circle':
                        cx_, cy_, radius = obs_data
                        dist = math.sqrt((x - cx_)**2 + (y - cy_)**2)
                        # Subtract our track half-width and obstacle radius to get edge-to-edge
                        edge_dist = dist - half_track - radius
                        if edge_dist < min_dist:
                            min_dist = edge_dist

        return min_dist

    def repulsion_cost(self, x: float, y: float, layer: str) -> float:
        """
        Calculate repulsion cost at point (x, y) on the given layer.
        Returns 0 if far from obstacles, up to repulsion_cost if at clearance boundary.
        Linear falloff from clearance to repulsion_distance.
        """
        if self.config.repulsion_cost <= 0 or self.config.repulsion_distance <= 0:
            return 0.0

        min_dist = self.min_distance_to_obstacle(x, y, layer)

        # If beyond repulsion distance, no cost
        if min_dist >= self.config.repulsion_distance:
            return 0.0

        # If at or below clearance (shouldn't happen, but handle it), max cost
        if min_dist <= 0:
            return self.config.repulsion_cost

        # Linear falloff: max cost at dist=0, zero cost at dist=repulsion_distance
        fraction = 1.0 - (min_dist / self.config.repulsion_distance)
        return self.config.repulsion_cost * fraction


class AStarRouter:
    """A* based PCB router with octilinear constraints."""

    # 8 directions: 0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°
    DIRECTIONS = [
        (1, 0),    # East
        (1, -1),   # NE (note: Y increases downward in KiCad)
        (0, -1),   # North
        (-1, -1),  # NW
        (-1, 0),   # West
        (-1, 1),   # SW
        (0, 1),    # South
        (1, 1),    # SE
    ]

    def __init__(self, obstacles: ObstacleGrid, config: RouteConfig):
        """
        Initialize A* router.

        Args:
            obstacles: Obstacle grid for collision detection
            config: Routing configuration
        """
        self.obstacles = obstacles
        self.config = config
        self.sqrt2 = math.sqrt(2)

    def _heuristic(self, state: State, goal: State) -> float:
        """Admissible heuristic: Euclidean distance + min via cost if layer differs."""
        dx = abs(state.x - goal.x)
        dy = abs(state.y - goal.y)
        h = math.sqrt(dx * dx + dy * dy)

        # Add via cost if on different layer
        if state.layer != goal.layer:
            h += self.config.via_cost

        return h

    def _heuristic_to_segments(self, state: State, target_segments: List[Segment]) -> float:
        """Heuristic: minimum distance to any target segment + via cost if needed."""
        min_dist = float('inf')
        needs_via = True

        for seg in target_segments:
            # Distance from point to segment
            dist = self._point_to_segment_distance(state.x, state.y,
                                                    seg.start_x, seg.start_y,
                                                    seg.end_x, seg.end_y)
            if dist < min_dist:
                min_dist = dist
                needs_via = (state.layer != seg.layer)

        if needs_via:
            min_dist += self.config.via_cost

        return min_dist

    def _point_to_segment_distance(self, px: float, py: float,
                                    x1: float, y1: float, x2: float, y2: float) -> float:
        """Calculate distance from point to line segment."""
        dx, dy = x2 - x1, y2 - y1
        seg_len_sq = dx * dx + dy * dy

        if seg_len_sq < 0.0001:  # Degenerate segment
            return math.sqrt((px - x1)**2 + (py - y1)**2)

        # Project point onto line, clamped to segment
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / seg_len_sq))
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        return math.sqrt((px - closest_x)**2 + (py - closest_y)**2)

    def _can_connect_to_segment(self, state: State, seg: Segment,
                                 via_radius: float, track_half_width: float) -> Optional[Tuple[float, float]]:
        """
        Check if current position can connect to the segment via copper overlap.
        Returns the connection point on the segment, or None if no connection.

        IMPORTANT: Connection is only possible if on the SAME LAYER as the segment.
        A via connects layers at the via location, not to tracks on other layers.
        """
        # Must be on the same layer to connect!
        if state.layer != seg.layer:
            return None

        # Same layer - check if we're close enough to the track
        dist = self._point_to_segment_distance(state.x, state.y,
                                               seg.start_x, seg.start_y,
                                               seg.end_x, seg.end_y)
        # Can connect if copper overlaps (via/track radius + track half width)
        if dist <= via_radius + track_half_width + 0.01:
            # Find the closest point on segment
            dx, dy = seg.end_x - seg.start_x, seg.end_y - seg.start_y
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 0.0001:
                return (seg.start_x, seg.start_y)
            t = max(0, min(1, ((state.x - seg.start_x) * dx + (state.y - seg.start_y) * dy) / seg_len_sq))
            return (seg.start_x + t * dx, seg.start_y + t * dy)

        return None

    def _is_goal(self, state: State, goal: State, tolerance: float = 0.05) -> bool:
        """Check if state is close enough to goal."""
        dx = abs(state.x - goal.x)
        dy = abs(state.y - goal.y)
        return dx <= tolerance and dy <= tolerance and state.layer == goal.layer

    def _snap_to_grid(self, x: float, y: float) -> Tuple[float, float]:
        """Snap coordinates to grid."""
        step = self.config.grid_step
        return (round(x / step) * step, round(y / step) * step)

    # ==================== Jump Point Search (JPS) Methods ====================
    # JPS dramatically speeds up A* by "jumping" over empty space instead of
    # expanding every grid cell. It identifies "jump points" where direction
    # changes are forced due to obstacles.

    def _is_blocked(self, x: float, y: float, layer: str,
                    reduced_clearance: float = None) -> bool:
        """Check if a position is blocked (can't move there)."""
        step = self.config.grid_step
        # Check a tiny segment at this point to see if position is valid
        return self.obstacles.segment_collides(x, y, x + step * 0.01, y + step * 0.01,
                                                layer, reduced_clearance)

    def _jump_horizontal(self, x: float, y: float, dx: int, layer: str,
                         goal_x: float, goal_y: float,
                         reduced_clearance: float = None,
                         max_jump: int = 200) -> Optional[Tuple[float, float]]:
        """
        Jump horizontally until we hit an obstacle or find a jump point.
        Returns jump point coordinates or None if blocked.
        """
        step = self.config.grid_step
        for _ in range(max_jump):
            nx = x + dx * step
            # Check if we can move there
            if self.obstacles.segment_collides(x, y, nx, y, layer, reduced_clearance):
                return None  # Blocked
            x = nx

            # Check if we reached goal
            if abs(x - goal_x) < step * 0.5 and abs(y - goal_y) < step * 0.5:
                return (x, y)

            # Check for forced neighbors (obstacles that create turning points)
            # If there's an obstacle above/below and open diagonal, it's a jump point
            blocked_above = self.obstacles.segment_collides(x, y, x, y - step, layer, reduced_clearance)
            blocked_below = self.obstacles.segment_collides(x, y, x, y + step, layer, reduced_clearance)

            if blocked_above:
                # Check if diagonal (dx, -1) is open - forced neighbor
                if not self.obstacles.segment_collides(x, y, x + dx * step, y - step, layer, reduced_clearance):
                    return (x, y)
            if blocked_below:
                # Check if diagonal (dx, +1) is open - forced neighbor
                if not self.obstacles.segment_collides(x, y, x + dx * step, y + step, layer, reduced_clearance):
                    return (x, y)

        return (x, y)  # Return current position if max jump reached

    def _jump_vertical(self, x: float, y: float, dy: int, layer: str,
                       goal_x: float, goal_y: float,
                       reduced_clearance: float = None,
                       max_jump: int = 200) -> Optional[Tuple[float, float]]:
        """
        Jump vertically until we hit an obstacle or find a jump point.
        """
        step = self.config.grid_step
        for _ in range(max_jump):
            ny = y + dy * step
            if self.obstacles.segment_collides(x, y, x, ny, layer, reduced_clearance):
                return None
            y = ny

            if abs(x - goal_x) < step * 0.5 and abs(y - goal_y) < step * 0.5:
                return (x, y)

            blocked_left = self.obstacles.segment_collides(x, y, x - step, y, layer, reduced_clearance)
            blocked_right = self.obstacles.segment_collides(x, y, x + step, y, layer, reduced_clearance)

            if blocked_left:
                if not self.obstacles.segment_collides(x, y, x - step, y + dy * step, layer, reduced_clearance):
                    return (x, y)
            if blocked_right:
                if not self.obstacles.segment_collides(x, y, x + step, y + dy * step, layer, reduced_clearance):
                    return (x, y)

        return (x, y)

    def _jump_diagonal(self, x: float, y: float, dx: int, dy: int, layer: str,
                       goal_x: float, goal_y: float,
                       reduced_clearance: float = None,
                       max_jump: int = 200) -> Optional[Tuple[float, float]]:
        """
        Jump diagonally, checking horizontal and vertical jumps at each step.
        """
        step = self.config.grid_step
        for _ in range(max_jump):
            nx = x + dx * step
            ny = y + dy * step

            if self.obstacles.segment_collides(x, y, nx, ny, layer, reduced_clearance):
                return None
            x, y = nx, ny

            if abs(x - goal_x) < step * 0.5 and abs(y - goal_y) < step * 0.5:
                return (x, y)

            # Check for forced neighbors on diagonal
            # Blocked perpendicular + open diagonal = forced neighbor
            blocked_horiz = self.obstacles.segment_collides(x, y, x - dx * step, y, layer, reduced_clearance)
            blocked_vert = self.obstacles.segment_collides(x, y, x, y - dy * step, layer, reduced_clearance)

            if blocked_horiz:
                if not self.obstacles.segment_collides(x, y, x - dx * step, y + dy * step, layer, reduced_clearance):
                    return (x, y)
            if blocked_vert:
                if not self.obstacles.segment_collides(x, y, x + dx * step, y - dy * step, layer, reduced_clearance):
                    return (x, y)

            # Check if horizontal or vertical jumps find anything interesting
            h_jump = self._jump_horizontal(x, y, dx, layer, goal_x, goal_y, reduced_clearance, 50)
            if h_jump is not None:
                return (x, y)

            v_jump = self._jump_vertical(x, y, dy, layer, goal_x, goal_y, reduced_clearance, 50)
            if v_jump is not None:
                return (x, y)

        return (x, y)

    def _get_jps_successors(self, state: State, goal: State,
                            reduced_clearance: float = None) -> List[Tuple[State, float]]:
        """
        Get JPS successors for a state. Instead of returning all 8 neighbors,
        returns only jump points which are much fewer in open areas.
        """
        successors = []
        step = self.config.grid_step
        x, y, layer = state.x, state.y, state.layer
        goal_x, goal_y = goal.x, goal.y

        # Try all 8 directions with jumping
        for dx, dy in self.DIRECTIONS:
            if dx == 0:
                # Vertical movement
                result = self._jump_vertical(x, y, dy, layer, goal_x, goal_y, reduced_clearance)
            elif dy == 0:
                # Horizontal movement
                result = self._jump_horizontal(x, y, dx, layer, goal_x, goal_y, reduced_clearance)
            else:
                # Diagonal movement
                result = self._jump_diagonal(x, y, dx, dy, layer, goal_x, goal_y, reduced_clearance)

            if result is not None:
                jx, jy = result
                # Calculate actual distance traveled
                dist = math.sqrt((jx - x)**2 + (jy - y)**2)
                if dist > 0.001:  # Only add if we actually moved
                    successors.append((State(jx, jy, layer), dist))

        return successors

    # ==================== End JPS Methods ====================

    def route(self, start: State, goal: State, max_iterations: int = None,
              use_escape_zone: bool = False) -> Optional[List[State]]:
        """
        Find a path from start to goal using A*.

        Args:
            start: Starting state
            goal: Goal state
            max_iterations: Maximum A* iterations before giving up
            use_escape_zone: If True, use reduced clearance near start/goal points
                           to escape from congested stub areas

        Returns list of States forming the path, or None if no path found.
        """
        if max_iterations is None:
            max_iterations = self.config.max_iterations
        # Save original endpoints before snapping
        original_start = start
        original_goal = goal

        # Snap start and goal to grid for A* search
        start = State(*self._snap_to_grid(start.x, start.y), start.layer)
        goal = State(*self._snap_to_grid(goal.x, goal.y), goal.layer)

        # Escape zone parameters - for escaping congested stub areas
        escape_radius = self.config.escape_radius
        escape_clearance = self.config.escape_clearance
        start_x, start_y = start.x, start.y
        goal_x, goal_y = goal.x, goal.y

        def is_in_escape_zone(x: float, y: float) -> bool:
            """Check if point is within escape radius of start or goal."""
            if not use_escape_zone:
                return False
            dist_start_sq = (x - start_x)**2 + (y - start_y)**2
            dist_goal_sq = (x - goal_x)**2 + (y - goal_y)**2
            return dist_start_sq < escape_radius * escape_radius or \
                   dist_goal_sq < escape_radius * escape_radius

        # Priority queue: (f_cost, counter, g_cost, state, parent)
        # Use weighted heuristic to encourage broader exploration
        h_weight = self.config.heuristic_weight
        counter = 0
        open_set = [(self._heuristic(start, goal) * h_weight, counter, 0.0, start, None)]

        # Best g_cost to reach each state
        g_costs: Dict[State, float] = {start: 0.0}

        # Track parent for path reconstruction
        parents: Dict[State, Optional[State]] = {start: None}

        # Closed set
        closed: Set[State] = set()

        iterations = 0

        while open_set and iterations < max_iterations:
            iterations += 1

            f, _, g, current, parent = heapq.heappop(open_set)

            if current in closed:
                continue

            closed.add(current)

            # Goal check
            if self._is_goal(current, goal):
                # Reconstruct path
                path = [current]
                while parents[path[-1]] is not None:
                    path.append(parents[path[-1]])
                path.reverse()

                # Replace snapped endpoints with original exact coordinates
                path[0] = original_start
                path[-1] = original_goal

                print(f"Route found in {iterations} iterations, path length: {len(path)}")
                return path

            # Check if current position is in escape zone
            in_escape = is_in_escape_zone(current.x, current.y)

            # Expand neighbors
            step = self.config.grid_step

            # Try 8 directions on same layer
            for dx, dy in self.DIRECTIONS:
                nx = current.x + dx * step
                ny = current.y + dy * step

                # Use reduced clearance if BOTH current and neighbor are in escape zone
                if in_escape and is_in_escape_zone(nx, ny):
                    collides = self.obstacles.segment_collides(
                        current.x, current.y, nx, ny, current.layer,
                        reduced_clearance=escape_clearance
                    )
                else:
                    collides = self.obstacles.segment_collides(
                        current.x, current.y, nx, ny, current.layer
                    )

                if collides:
                    continue

                neighbor = State(nx, ny, current.layer)

                # Calculate move cost (diagonal is sqrt(2) * step)
                move_cost = step * self.sqrt2 if (dx != 0 and dy != 0) else step
                # Add repulsion cost to discourage routing near obstacles
                move_cost += self.obstacles.repulsion_cost(nx, ny, current.layer)
                new_g = g + move_cost

                if neighbor in closed:
                    continue

                if neighbor not in g_costs or new_g < g_costs[neighbor]:
                    g_costs[neighbor] = new_g
                    parents[neighbor] = current
                    f = new_g + self._heuristic(neighbor, goal) * h_weight
                    counter += 1
                    heapq.heappush(open_set, (f, counter, new_g, neighbor, current))

            # Try via to other layers
            if not self.obstacles.via_collides(current.x, current.y):
                for layer in self.config.layers:
                    if layer == current.layer:
                        continue

                    neighbor = State(current.x, current.y, layer)
                    new_g = g + self.config.via_cost

                    if neighbor in closed:
                        continue

                    if neighbor not in g_costs or new_g < g_costs[neighbor]:
                        g_costs[neighbor] = new_g
                        parents[neighbor] = current
                        f = new_g + self._heuristic(neighbor, goal) * h_weight
                        counter += 1
                        heapq.heappush(open_set, (f, counter, new_g, neighbor, current))

        print(f"No route found after {iterations} iterations")
        return None

    def route_to_segments(self, start: State, target_segments: List[Segment],
                          max_iterations: int = None) -> Optional[Tuple[List[State], Tuple[float, float]]]:
        """
        Find a path from start to any point on the target segments.
        Allows connecting via copper overlap (via pad touching track).

        Returns (path, connection_point) or None if no path found.
        The connection_point is where the route connects to the target segment.
        """
        if max_iterations is None:
            max_iterations = self.config.max_iterations
        original_start = start
        start = State(*self._snap_to_grid(start.x, start.y), start.layer)

        via_radius = self.config.via_size / 2
        track_half_width = self.config.track_width / 2
        h_weight = self.config.heuristic_weight
        step = self.config.grid_step

        # Escape zone parameters - for escaping congested stub areas
        escape_radius = self.config.escape_radius
        escape_clearance = self.config.escape_clearance
        start_x, start_y = start.x, start.y

        def is_in_escape_zone(x: float, y: float) -> bool:
            """Check if point is within escape radius of starting point."""
            dist_sq = (x - start_x)**2 + (y - start_y)**2
            return dist_sq < escape_radius * escape_radius

        # Priority queue: (f_cost, counter, g_cost, state, parent)
        counter = 0
        open_set = [(self._heuristic_to_segments(start, target_segments) * h_weight, counter, 0.0, start, None)]

        g_costs: Dict[State, float] = {start: 0.0}
        parents: Dict[State, Optional[State]] = {start: None}
        closed: Set[State] = set()

        iterations = 0

        while open_set and iterations < max_iterations:
            iterations += 1

            f, _, g, current, parent = heapq.heappop(open_set)

            if current in closed:
                continue

            closed.add(current)

            # Check if we can connect to any target segment from here
            for seg in target_segments:
                conn_point = self._can_connect_to_segment(current, seg, via_radius, track_half_width)
                if conn_point is not None:
                    # Found connection! Reconstruct path
                    path = [current]
                    while parents[path[-1]] is not None:
                        path.append(parents[path[-1]])
                    path.reverse()

                    # Replace snapped start with original
                    path[0] = original_start

                    # Add the connection point to the path to ensure explicit segment
                    # from route endpoint to the stub
                    conn_state = State(conn_point[0], conn_point[1], seg.layer)
                    path.append(conn_state)

                    print(f"Route found in {iterations} iterations, path length: {len(path)}")
                    print(f"  Connecting to segment at ({conn_point[0]:.3f}, {conn_point[1]:.3f})")
                    return (path, conn_point)

            # Check if current position is in escape zone
            in_escape = is_in_escape_zone(current.x, current.y)

            # Expand neighbors - 8 directions on same layer
            for dx, dy in self.DIRECTIONS:
                nx = current.x + dx * step
                ny = current.y + dy * step

                # Use reduced clearance if BOTH current and neighbor are in escape zone
                if in_escape and is_in_escape_zone(nx, ny):
                    collides = self.obstacles.segment_collides(
                        current.x, current.y, nx, ny, current.layer,
                        reduced_clearance=escape_clearance
                    )
                else:
                    collides = self.obstacles.segment_collides(
                        current.x, current.y, nx, ny, current.layer
                    )

                if collides:
                    continue

                neighbor = State(nx, ny, current.layer)
                move_cost = step * self.sqrt2 if (dx != 0 and dy != 0) else step
                # Add repulsion cost to discourage routing near obstacles
                move_cost += self.obstacles.repulsion_cost(nx, ny, current.layer)
                new_g = g + move_cost

                if neighbor in closed:
                    continue

                if neighbor not in g_costs or new_g < g_costs[neighbor]:
                    g_costs[neighbor] = new_g
                    parents[neighbor] = current
                    f = new_g + self._heuristic_to_segments(neighbor, target_segments) * h_weight
                    counter += 1
                    heapq.heappush(open_set, (f, counter, new_g, neighbor, current))

            # Try via to other layers
            if not self.obstacles.via_collides(current.x, current.y):
                for layer in self.config.layers:
                    if layer == current.layer:
                        continue

                    neighbor = State(current.x, current.y, layer)
                    new_g = g + self.config.via_cost

                    if neighbor in closed:
                        continue

                    if neighbor not in g_costs or new_g < g_costs[neighbor]:
                        g_costs[neighbor] = new_g
                        parents[neighbor] = current
                        f = new_g + self._heuristic_to_segments(neighbor, target_segments) * h_weight
                        counter += 1
                        heapq.heappush(open_set, (f, counter, new_g, neighbor, current))

        print(f"No route found after {iterations} iterations")
        return None

    def route_segments_to_segments(self, source_segments: List[Segment],
                                    target_segments: List[Segment],
                                    max_iterations: int = None) -> Optional[Tuple[List[State], Tuple[float, float], Tuple[float, float]]]:
        """
        Find a path from any point on source segments to any point on target segments.
        This allows escaping from congested stub areas.

        Uses reduced clearance within an "escape zone" near starting points, since
        stub segments are tightly packed but already validated in the design.

        Returns (path, source_connection_point, target_connection_point) or None.
        """
        if max_iterations is None:
            max_iterations = self.config.max_iterations
        via_radius = self.config.via_size / 2
        track_half_width = self.config.track_width / 2
        step = self.config.grid_step
        h_weight = self.config.heuristic_weight

        # Escape zone parameters from config - for escaping congested stub areas
        escape_radius = self.config.escape_radius
        escape_clearance = self.config.escape_clearance

        # Generate starting points along source segments (sample every grid step)
        start_states = []
        # Track the stub endpoint(s) for escape zone - just the segment endpoints, not all samples
        escape_zone_centers = set()
        for seg in source_segments:
            # Add segment endpoints as escape zone centers (these are the actual stub tips)
            escape_zone_centers.add((round(seg.start_x, 3), round(seg.start_y, 3)))
            escape_zone_centers.add((round(seg.end_x, 3), round(seg.end_y, 3)))

            dx = seg.end_x - seg.start_x
            dy = seg.end_y - seg.start_y
            length = math.sqrt(dx*dx + dy*dy)
            if length < 0.001:
                continue
            num_points = max(2, int(length / step) + 1)
            for i in range(num_points):
                t = i / (num_points - 1)
                x = seg.start_x + t * dx
                y = seg.start_y + t * dy
                # Snap to grid
                x, y = self._snap_to_grid(x, y)
                start_states.append((State(x, y, seg.layer), (seg.start_x + t*dx, seg.start_y + t*dy)))

        if not start_states:
            return None

        # Pre-compute escape zone centers as a list for faster iteration
        escape_centers_list = list(escape_zone_centers)
        escape_radius_sq = escape_radius * escape_radius

        def is_in_escape_zone(x: float, y: float) -> bool:
            """Check if point is within escape radius of stub endpoints (not all samples)."""
            for sx, sy in escape_centers_list:
                dx = x - sx
                dy = y - sy
                if dx*dx + dy*dy < escape_radius_sq:
                    return True
            return False

        # Initialize A* with all starting points
        counter = 0
        open_set = []
        g_costs: Dict[State, float] = {}
        parents: Dict[State, Optional[State]] = {}
        source_points: Dict[State, Tuple[float, float]] = {}  # Track which source point each state came from
        closed: Set[State] = set()

        for state, source_pt in start_states:
            h = self._heuristic_to_segments(state, target_segments) * h_weight
            heapq.heappush(open_set, (h, counter, 0.0, state, None))
            counter += 1
            g_costs[state] = 0.0
            parents[state] = None
            source_points[state] = source_pt

        iterations = 0

        while open_set and iterations < max_iterations:
            iterations += 1

            f, _, g, current, parent = heapq.heappop(open_set)

            if current in closed:
                continue

            closed.add(current)

            # Propagate source point info
            if current not in source_points and parent in source_points:
                source_points[current] = source_points[parent]

            # Check if we can connect to any target segment
            for seg in target_segments:
                conn_point = self._can_connect_to_segment(current, seg, via_radius, track_half_width)
                if conn_point is not None:
                    # Found connection! Reconstruct path
                    path = [current]
                    while parents[path[-1]] is not None:
                        path.append(parents[path[-1]])
                    path.reverse()

                    # Add connection point to path
                    conn_state = State(conn_point[0], conn_point[1], seg.layer)
                    path.append(conn_state)

                    # Get source connection point
                    src_pt = source_points.get(path[0], (path[0].x, path[0].y))

                    print(f"Route found in {iterations} iterations, path length: {len(path)}")
                    print(f"  From source at ({src_pt[0]:.3f}, {src_pt[1]:.3f})")
                    print(f"  To target at ({conn_point[0]:.3f}, {conn_point[1]:.3f})")
                    return (path, src_pt, conn_point)

            # Check if current position is in escape zone
            in_escape = is_in_escape_zone(current.x, current.y)

            # Expand neighbors - use JPS when enabled and outside escape zone
            use_jps_here = self.config.use_jps and not in_escape

            if use_jps_here:
                # Use Jump Point Search for faster exploration
                # Create a dummy goal from target segment centroid for JPS direction
                target_cx = sum((s.start_x + s.end_x) / 2 for s in target_segments) / len(target_segments)
                target_cy = sum((s.start_y + s.end_y) / 2 for s in target_segments) / len(target_segments)
                dummy_goal = State(target_cx, target_cy, current.layer)

                jps_successors = self._get_jps_successors(current, dummy_goal, reduced_clearance=None)
                for neighbor, move_cost in jps_successors:
                    new_g = g + move_cost

                    if neighbor in closed:
                        continue

                    if neighbor not in g_costs or new_g < g_costs[neighbor]:
                        g_costs[neighbor] = new_g
                        parents[neighbor] = current
                        if current in source_points:
                            source_points[neighbor] = source_points[current]
                        f = new_g + self._heuristic_to_segments(neighbor, target_segments) * h_weight
                        counter += 1
                        heapq.heappush(open_set, (f, counter, new_g, neighbor, current))
            else:
                # Standard A* neighbor expansion (used in escape zone or when JPS disabled)
                for dx, dy in self.DIRECTIONS:
                    nx = current.x + dx * step
                    ny = current.y + dy * step

                    # Use reduced clearance if BOTH current and neighbor are in escape zone
                    if in_escape and is_in_escape_zone(nx, ny):
                        collides = self.obstacles.segment_collides(
                            current.x, current.y, nx, ny, current.layer,
                            reduced_clearance=escape_clearance
                        )
                    else:
                        collides = self.obstacles.segment_collides(
                            current.x, current.y, nx, ny, current.layer
                        )

                    if collides:
                        continue

                    neighbor = State(nx, ny, current.layer)
                    move_cost = step * self.sqrt2 if (dx != 0 and dy != 0) else step
                    # Add repulsion cost to discourage routing near obstacles
                    move_cost += self.obstacles.repulsion_cost(nx, ny, current.layer)
                    new_g = g + move_cost

                    if neighbor in closed:
                        continue

                    if neighbor not in g_costs or new_g < g_costs[neighbor]:
                        g_costs[neighbor] = new_g
                        parents[neighbor] = current
                        if current in source_points:
                            source_points[neighbor] = source_points[current]
                        f = new_g + self._heuristic_to_segments(neighbor, target_segments) * h_weight
                        counter += 1
                        heapq.heappush(open_set, (f, counter, new_g, neighbor, current))

            # Try via to other layers
            # Note: Do NOT bypass via collision check in escape zone - we only relax
            # clearance for track routing, not for via placement which could collide
            # with existing vias/pads on other nets
            via_check_result = self.obstacles.via_collides(current.x, current.y)
            if not via_check_result:
                for layer in self.config.layers:
                    if layer == current.layer:
                        continue

                    neighbor = State(current.x, current.y, layer)
                    new_g = g + self.config.via_cost

                    if neighbor in closed:
                        continue

                    if neighbor not in g_costs or new_g < g_costs[neighbor]:
                        g_costs[neighbor] = new_g
                        parents[neighbor] = current
                        if current in source_points:
                            source_points[neighbor] = source_points[current]
                        f = new_g + self._heuristic_to_segments(neighbor, target_segments) * h_weight
                        counter += 1
                        heapq.heappush(open_set, (f, counter, new_g, neighbor, current))

        print(f"No route found after {iterations} iterations")
        return None

    def route_endpoint_to_segments(self, start: State, target_segments: List[Segment],
                                    max_iterations: int = None) -> Optional[Tuple[List[State], Tuple[float, float]]]:
        """
        Find a path from a single endpoint to any point on target segments.
        Used for BGA fanout (fixed endpoint) to QFN stub (flexible target).

        Uses reduced clearance in an "escape zone" near the starting point to help
        escape from congested stub areas.

        Returns (path, target_connection_point) or None.
        """
        if max_iterations is None:
            max_iterations = self.config.max_iterations
        via_radius = self.config.via_size / 2
        track_half_width = self.config.track_width / 2
        step = self.config.grid_step
        h_weight = self.config.heuristic_weight

        # Escape zone parameters from config
        escape_radius = self.config.escape_radius
        escape_clearance = self.config.escape_clearance
        escape_radius_sq = escape_radius * escape_radius
        start_x, start_y = start.x, start.y

        def is_in_escape_zone(x: float, y: float) -> bool:
            """Check if point is within escape radius of starting point."""
            dx = x - start_x
            dy = y - start_y
            return dx*dx + dy*dy < escape_radius_sq

        # Initialize A* with single starting point
        counter = 0
        open_set = []
        g_costs: Dict[State, float] = {}
        parents: Dict[State, Optional[State]] = {}
        closed: Set[State] = set()

        h = self._heuristic_to_segments(start, target_segments) * h_weight
        heapq.heappush(open_set, (h, counter, 0.0, start, None))
        counter += 1
        g_costs[start] = 0.0
        parents[start] = None

        iterations = 0

        while open_set and iterations < max_iterations:
            iterations += 1

            f, _, g, current, parent = heapq.heappop(open_set)

            if current in closed:
                continue

            closed.add(current)

            # Check if we can connect to any target segment
            for seg in target_segments:
                conn_point = self._can_connect_to_segment(current, seg, via_radius, track_half_width)
                if conn_point is not None:
                    # Found connection! Reconstruct path
                    path = [current]
                    while parents[path[-1]] is not None:
                        path.append(parents[path[-1]])
                    path.reverse()

                    # Add connection point to path
                    conn_state = State(conn_point[0], conn_point[1], seg.layer)
                    path.append(conn_state)

                    print(f"Route found in {iterations} iterations, path length: {len(path)}")
                    print(f"  Connecting to segment at ({conn_point[0]:.3f}, {conn_point[1]:.3f})")
                    return (path, conn_point)

            # Expand neighbors using JPS when enabled
            if self.config.use_jps:
                # Create a dummy goal from target segment centroid for JPS direction
                target_cx = sum((s.start_x + s.end_x) / 2 for s in target_segments) / len(target_segments)
                target_cy = sum((s.start_y + s.end_y) / 2 for s in target_segments) / len(target_segments)
                dummy_goal = State(target_cx, target_cy, current.layer)

                # Use reduced clearance in escape zone near starting point
                in_escape = is_in_escape_zone(current.x, current.y)
                jps_reduced_clearance = escape_clearance if in_escape else None
                jps_successors = self._get_jps_successors(current, dummy_goal, reduced_clearance=jps_reduced_clearance)
                for neighbor, move_cost in jps_successors:
                    new_g = g + move_cost

                    if neighbor in closed:
                        continue

                    if neighbor not in g_costs or new_g < g_costs[neighbor]:
                        g_costs[neighbor] = new_g
                        parents[neighbor] = current
                        f = new_g + self._heuristic_to_segments(neighbor, target_segments) * h_weight
                        counter += 1
                        heapq.heappush(open_set, (f, counter, new_g, neighbor, current))
            else:
                # Standard A* neighbor expansion
                for dx, dy in self.DIRECTIONS:
                    nx = current.x + dx * step
                    ny = current.y + dy * step

                    # Use reduced clearance in escape zone near starting point
                    # Use reduced clearance if EITHER endpoint is in escape zone (to allow escaping)
                    in_escape = is_in_escape_zone(current.x, current.y) or is_in_escape_zone(nx, ny)
                    if in_escape:
                        collides = self.obstacles.segment_collides(
                            current.x, current.y, nx, ny, current.layer,
                            reduced_clearance=escape_clearance
                        )
                    else:
                        collides = self.obstacles.segment_collides(current.x, current.y, nx, ny, current.layer)

                    if collides:
                        continue

                    neighbor = State(nx, ny, current.layer)
                    move_cost = step * self.sqrt2 if (dx != 0 and dy != 0) else step
                    # Add repulsion cost to discourage routing near obstacles
                    move_cost += self.obstacles.repulsion_cost(nx, ny, current.layer)
                    new_g = g + move_cost

                    if neighbor in closed:
                        continue

                    if neighbor not in g_costs or new_g < g_costs[neighbor]:
                        g_costs[neighbor] = new_g
                        parents[neighbor] = current
                        f = new_g + self._heuristic_to_segments(neighbor, target_segments) * h_weight
                        counter += 1
                        heapq.heappush(open_set, (f, counter, new_g, neighbor, current))

            # Try via to other layers
            via_check_result = self.obstacles.via_collides(current.x, current.y)
            if not via_check_result:
                for layer in self.config.layers:
                    if layer == current.layer:
                        continue

                    neighbor = State(current.x, current.y, layer)
                    new_g = g + self.config.via_cost

                    if neighbor in closed:
                        continue

                    if neighbor not in g_costs or new_g < g_costs[neighbor]:
                        g_costs[neighbor] = new_g
                        parents[neighbor] = current
                        f = new_g + self._heuristic_to_segments(neighbor, target_segments) * h_weight
                        counter += 1
                        heapq.heappush(open_set, (f, counter, new_g, neighbor, current))

        print(f"No route found after {iterations} iterations")
        return None

    def simplify_path(self, path: List[State]) -> List[State]:
        """
        Simplify path by merging collinear segments.
        Returns a list of waypoints (vertices where direction changes or layer changes).
        """
        if len(path) <= 2:
            return path

        def get_direction(p1: State, p2: State) -> Tuple[int, int]:
            """Get normalized direction as integer tuple (-1, 0, or 1)."""
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            # Normalize to -1, 0, 1
            def sign(v):
                if v > 0.001:
                    return 1
                elif v < -0.001:
                    return -1
                return 0
            return (sign(dx), sign(dy))

        simplified = [path[0]]

        for i in range(1, len(path) - 1):
            prev = simplified[-1]
            curr = path[i]
            next_ = path[i + 1]

            # Layer change is always a waypoint
            if curr.layer != prev.layer or curr.layer != next_.layer:
                simplified.append(curr)
                continue

            # Check if direction changes
            dir1 = get_direction(prev, curr)
            dir2 = get_direction(curr, next_)

            # If directions differ, keep this waypoint
            if dir1 != dir2:
                simplified.append(curr)

        simplified.append(path[-1])
        return simplified

    def smooth_path(self, path: List[State]) -> List[State]:
        """
        Smooth the path by replacing zigzag staircase patterns with clean
        horizontal + diagonal or diagonal + horizontal segments.

        The key insight is that a zigzag from (x1,y1) to (x2,y2) can be replaced
        by either:
        - horizontal first, then diagonal: (x1,y1) -> (x1+dx-dy, y1) -> (x2,y2)
        - diagonal first, then horizontal: (x1,y1) -> (x1+dy, y1+dy) -> (x2,y2)

        Where dx = x2-x1, dy = y2-y1 (for dx > dy case).
        """
        if len(path) <= 2:
            return path

        def find_best_two_segment_path(p1: State, p2: State) -> Optional[List[State]]:
            """
            Try to connect p1 to p2 with at most 2 octilinear segments.
            Returns the intermediate waypoints if successful.
            """
            if p1.layer != p2.layer:
                return None

            dx = p2.x - p1.x
            dy = p2.y - p1.y
            adx, ady = abs(dx), abs(dy)

            # Already octilinear - just check single segment
            is_horizontal = ady < 0.001
            is_vertical = adx < 0.001
            is_diagonal = abs(adx - ady) < 0.001

            if is_horizontal or is_vertical or is_diagonal:
                if not self.obstacles.segment_collides(p1.x, p1.y, p2.x, p2.y, p1.layer):
                    return []  # Direct connection, no intermediate points

            # Try horizontal-then-diagonal
            if adx > ady:
                # More horizontal than vertical
                # Horizontal portion: (x1,y1) -> (x2-|dy|*sign(dx), y1)
                horiz_end_x = p2.x - ady * (1 if dx > 0 else -1)
                mid1 = State(horiz_end_x, p1.y, p1.layer)

                if not self.obstacles.segment_collides(p1.x, p1.y, mid1.x, mid1.y, p1.layer):
                    if not self.obstacles.segment_collides(mid1.x, mid1.y, p2.x, p2.y, p1.layer):
                        return [mid1]

                # Try diagonal-then-horizontal
                diag_end_x = p1.x + ady * (1 if dx > 0 else -1)
                diag_end_y = p1.y + ady * (1 if dy > 0 else -1)
                mid2 = State(diag_end_x, diag_end_y, p1.layer)

                if not self.obstacles.segment_collides(p1.x, p1.y, mid2.x, mid2.y, p1.layer):
                    if not self.obstacles.segment_collides(mid2.x, mid2.y, p2.x, p2.y, p1.layer):
                        return [mid2]

            else:
                # More vertical than horizontal
                # Vertical portion: (x1,y1) -> (x1, y2-|dx|*sign(dy))
                vert_end_y = p2.y - adx * (1 if dy > 0 else -1)
                mid1 = State(p1.x, vert_end_y, p1.layer)

                if not self.obstacles.segment_collides(p1.x, p1.y, mid1.x, mid1.y, p1.layer):
                    if not self.obstacles.segment_collides(mid1.x, mid1.y, p2.x, p2.y, p1.layer):
                        return [mid1]

                # Try diagonal-then-vertical
                diag_end_x = p1.x + adx * (1 if dx > 0 else -1)
                diag_end_y = p1.y + adx * (1 if dy > 0 else -1)
                mid2 = State(diag_end_x, diag_end_y, p1.layer)

                if not self.obstacles.segment_collides(p1.x, p1.y, mid2.x, mid2.y, p1.layer):
                    if not self.obstacles.segment_collides(mid2.x, mid2.y, p2.x, p2.y, p1.layer):
                        return [mid2]

            return None  # Can't do it with 2 segments

        # Greedy smoothing: try to replace zigzags with clean 2-segment paths
        smoothed = [path[0]]
        i = 0

        while i < len(path) - 1:
            best_j = i + 1
            best_intermediate = None

            # Try to find the furthest point we can reach with 1-2 segments
            for j in range(len(path) - 1, i + 1, -1):  # Search from far to near
                # Stop at layer changes
                if path[j].layer != path[i].layer:
                    continue

                result = find_best_two_segment_path(path[i], path[j])
                if result is not None:
                    best_j = j
                    best_intermediate = result
                    break  # Found furthest reachable point

            # Add intermediate points and destination
            if best_intermediate:
                for mid in best_intermediate:
                    smoothed.append(mid)
            smoothed.append(path[best_j])

            i = best_j

        return smoothed


def find_stub_segments(pcb_data: PCBData, net_id: int) -> List[List[Segment]]:
    """
    Find the existing track stub chains for a net.
    Returns a list of segment chains, where each chain is a connected set of segments.
    """
    segments = [s for s in pcb_data.segments if s.net_id == net_id]
    if not segments:
        return []

    # Build adjacency - segments that share endpoints
    def point_key(x, y):
        return (round(x, 3), round(y, 3))

    # Map from point to segments touching that point
    point_to_segs = {}
    for seg in segments:
        for pt in [(seg.start_x, seg.start_y), (seg.end_x, seg.end_y)]:
            key = point_key(*pt)
            if key not in point_to_segs:
                point_to_segs[key] = []
            point_to_segs[key].append(seg)

    # Find connected chains using DFS
    visited = set()
    chains = []

    for seg in segments:
        if id(seg) in visited:
            continue

        chain = []
        stack = [seg]
        while stack:
            s = stack.pop()
            if id(s) in visited:
                continue
            visited.add(id(s))
            chain.append(s)

            # Find connected segments
            for pt in [(s.start_x, s.start_y), (s.end_x, s.end_y)]:
                key = point_key(*pt)
                for neighbor in point_to_segs.get(key, []):
                    if id(neighbor) not in visited:
                        stack.append(neighbor)

        if chain:
            chains.append(chain)

    return chains


def find_stub_endpoints(pcb_data: PCBData, net_id: int) -> List[Tuple[float, float, str]]:
    """
    Find the endpoints of existing track stubs for a net.

    An endpoint is a segment endpoint that isn't connected to another segment
    or to the original pad.
    """
    segments = [s for s in pcb_data.segments if s.net_id == net_id]
    vias = [v for v in pcb_data.vias if v.net_id == net_id]
    pads = pcb_data.pads_by_net.get(net_id, [])

    if not segments:
        # No stubs, return pad locations
        endpoints = []
        for pad in pads:
            # Determine layer from pad
            layer = pad.layers[0] if pad.layers else 'F.Cu'
            if 'F.Cu' in pad.layers:
                layer = 'F.Cu'
            elif 'B.Cu' in pad.layers:
                layer = 'B.Cu'
            endpoints.append((pad.global_x, pad.global_y, layer))
        return endpoints

    # Collect all segment endpoints
    all_points = []
    for seg in segments:
        all_points.append((seg.start_x, seg.start_y, seg.layer))
        all_points.append((seg.end_x, seg.end_y, seg.layer))

    # Count occurrences - endpoints only appear once
    from collections import Counter
    point_counts = Counter()
    for x, y, layer in all_points:
        key = (round(x, 3), round(y, 3), layer)
        point_counts[key] += 1

    # Also add pad and via locations as connection points
    connection_points = set()
    for pad in pads:
        connection_points.add((round(pad.global_x, 3), round(pad.global_y, 3)))
    for via in vias:
        connection_points.add((round(via.x, 3), round(via.y, 3)))

    # Find endpoints (appear once and not at pad/via)
    endpoints = []
    for (x, y, layer), count in point_counts.items():
        if count == 1:  # Only appears once = endpoint
            if (x, y) not in connection_points:
                endpoints.append((x, y, layer))

    return endpoints


@dataclass
class RouteResult:
    """Result of routing a net."""
    path: List[State]  # Simplified path waypoints
    new_segments: List[Segment]  # New track segments to add
    new_vias: List[Via]  # New vias to add
    segments_to_remove: List[Segment] = None  # Existing segments to remove
    segments_to_shorten: List[Tuple[Segment, float, float, float, float]] = None  # (seg, new_start_x, new_start_y, new_end_x, new_end_y)

    def __post_init__(self):
        if self.segments_to_remove is None:
            self.segments_to_remove = []
        if self.segments_to_shorten is None:
            self.segments_to_shorten = []


def point_on_segment(px: float, py: float, x1: float, y1: float, x2: float, y2: float,
                     tolerance: float = 0.05) -> bool:
    """Check if point (px, py) lies on segment (x1,y1)-(x2,y2)."""
    # Check if point is within bounding box (with tolerance)
    min_x, max_x = min(x1, x2) - tolerance, max(x1, x2) + tolerance
    min_y, max_y = min(y1, y2) - tolerance, max(y1, y2) + tolerance
    if not (min_x <= px <= max_x and min_y <= py <= max_y):
        return False

    # Check distance from point to line
    dx, dy = x2 - x1, y2 - y1
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 0.0001:  # Degenerate segment
        return math.sqrt((px - x1)**2 + (py - y1)**2) < tolerance

    # Project point onto line
    t = ((px - x1) * dx + (py - y1) * dy) / seg_len_sq
    if t < -0.01 or t > 1.01:  # Outside segment
        return False

    # Distance from point to closest point on segment
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy
    dist = math.sqrt((px - closest_x)**2 + (py - closest_y)**2)
    return dist < tolerance


def find_stub_intersection_and_trim(pcb_data: PCBData, net_id: int, path: List[State],
                                     pads: List[Pad]) -> Tuple[List[Segment], List[Tuple[Segment, float, float, float, float]]]:
    """
    Find where the new route intersects existing stub segments and determine
    which segments to remove or shorten.

    Returns:
        segments_to_remove: List of segments to completely remove
        segments_to_shorten: List of (segment, new_start_x, new_start_y, new_end_x, new_end_y)
    """
    segments_to_remove = []
    segments_to_shorten_map = {}  # seg id -> (segment, new_start_x, new_start_y, new_end_x, new_end_y, distance_from_pad_end)

    existing_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    if not existing_segments:
        return [], []

    # Get pad locations to know which end of stub connects to pad
    pad_locs = {(round(p.global_x, 3), round(p.global_y, 3)) for p in pads}

    # First, check if first/last path points are near segment endpoints
    # (handles grid-snapped paths that don't exactly lie on segments)
    near_endpoint_tolerance = 0.15  # mm - larger than grid step to catch snapped points

    for state in [path[0], path[-1]]:
        px, py, layer = state.x, state.y, state.layer

        for seg in existing_segments:
            if seg.layer != layer:
                continue

            # Check if path point is near segment start or end
            near_start = (abs(px - seg.start_x) < near_endpoint_tolerance and
                         abs(py - seg.start_y) < near_endpoint_tolerance)
            near_end = (abs(px - seg.end_x) < near_endpoint_tolerance and
                       abs(py - seg.end_y) < near_endpoint_tolerance)

            if not near_start and not near_end:
                continue

            # Found a nearby endpoint! Determine pad direction
            start_key = (round(seg.start_x, 3), round(seg.start_y, 3))
            end_key = (round(seg.end_x, 3), round(seg.end_y, 3))

            start_is_pad = start_key in pad_locs
            end_is_pad = end_key in pad_locs

            if not start_is_pad and not end_is_pad:
                start_is_pad = trace_to_pad(existing_segments, seg, 'start', pad_locs)
                end_is_pad = trace_to_pad(existing_segments, seg, 'end', pad_locs)

            # Handle the four cases based on which endpoint we're near
            if near_start and start_is_pad:
                # Connection near pad end - remove this segment and beyond
                if seg not in segments_to_remove:
                    segments_to_remove.append(seg)
                removed = find_segments_beyond_intersection(
                    existing_segments, seg, seg.end_x, seg.end_y, pad_locs, True
                )
                segments_to_remove.extend(removed)
            elif near_end and end_is_pad:
                # Connection near pad end - remove this segment and beyond
                if seg not in segments_to_remove:
                    segments_to_remove.append(seg)
                removed = find_segments_beyond_intersection(
                    existing_segments, seg, seg.start_x, seg.start_y, pad_locs, False
                )
                segments_to_remove.extend(removed)
            elif near_end and not end_is_pad:
                # Connection at far end - remove segments beyond
                removed = find_segments_beyond_intersection(
                    existing_segments, seg, seg.end_x, seg.end_y, pad_locs, True
                )
                segments_to_remove.extend(removed)
            elif near_start and not start_is_pad:
                # Connection at far end - remove this segment and beyond
                if seg not in segments_to_remove:
                    segments_to_remove.append(seg)
                removed = find_segments_beyond_intersection(
                    existing_segments, seg, seg.end_x, seg.end_y, pad_locs, True
                )
                segments_to_remove.extend(removed)

    # Check each point in the new path for intersection with existing segments
    for state in path:
        px, py, layer = state.x, state.y, state.layer

        for seg in existing_segments:
            if seg.layer != layer:
                continue

            # Check if this path point lies on this segment
            if point_on_segment(px, py, seg.start_x, seg.start_y, seg.end_x, seg.end_y):
                # Found intersection! Now determine which part to keep

                # Check which end is closer to a pad
                start_key = (round(seg.start_x, 3), round(seg.start_y, 3))
                end_key = (round(seg.end_x, 3), round(seg.end_y, 3))

                start_is_pad = start_key in pad_locs
                end_is_pad = end_key in pad_locs

                # Also check if either end connects to another segment (toward pad)
                if not start_is_pad and not end_is_pad:
                    # Need to trace which direction leads to pad
                    start_is_pad = trace_to_pad(existing_segments, seg, 'start', pad_locs)
                    end_is_pad = trace_to_pad(existing_segments, seg, 'end', pad_locs)

                # Check if intersection point is at segment endpoint (no trim needed for this segment)
                at_start = (abs(px - seg.start_x) < 0.05 and abs(py - seg.start_y) < 0.05)
                at_end = (abs(px - seg.end_x) < 0.05 and abs(py - seg.end_y) < 0.05)

                if at_start or at_end:
                    # Intersection at endpoint - segments beyond may need removal
                    # Also, this segment itself may need removal depending on which
                    # side of the intersection is toward the pad

                    if at_start and start_is_pad:
                        # Connection is at the pad end of this segment
                        # This segment extends AWAY from the pad - remove it and
                        # anything beyond its far end
                        if seg not in segments_to_remove:
                            segments_to_remove.append(seg)
                        removed = find_segments_beyond_intersection(
                            existing_segments, seg, seg.end_x, seg.end_y, pad_locs, True
                        )
                        segments_to_remove.extend(removed)
                    elif at_end and end_is_pad:
                        # Connection is at the pad end of this segment
                        # This segment extends AWAY from the pad - remove it and
                        # anything beyond its far end (the start)
                        if seg not in segments_to_remove:
                            segments_to_remove.append(seg)
                        removed = find_segments_beyond_intersection(
                            existing_segments, seg, seg.start_x, seg.start_y, pad_locs, False
                        )
                        segments_to_remove.extend(removed)
                    elif at_end and not end_is_pad:
                        # The new route connects at the end of this segment
                        # This segment stays (it connects path to the stub toward pad)
                        # Find and remove segments beyond the end
                        removed = find_segments_beyond_intersection(
                            existing_segments, seg, seg.end_x, seg.end_y, pad_locs, True
                        )
                        segments_to_remove.extend(removed)
                    elif at_start and not start_is_pad:
                        # The new route connects at the start of this segment
                        # If start is NOT toward pad, this whole segment is beyond
                        # the connection point and should be removed
                        if seg not in segments_to_remove:
                            segments_to_remove.append(seg)
                        # Also find and remove segments connected at the END of this segment
                        removed = find_segments_beyond_intersection(
                            existing_segments, seg, seg.end_x, seg.end_y, pad_locs, True
                        )
                        segments_to_remove.extend(removed)
                    continue

                # Intersection in middle of segment - need to shorten
                # Calculate distance from pad end to intersection point
                if start_is_pad or (not end_is_pad and not start_is_pad):
                    # Keep the start side (toward pad), trim the end side
                    keep_start = True
                    dist_from_pad = math.sqrt((px - seg.start_x)**2 + (py - seg.start_y)**2)
                else:
                    # Keep the end side (toward pad), trim the start side
                    keep_start = False
                    dist_from_pad = math.sqrt((px - seg.end_x)**2 + (py - seg.end_y)**2)

                # Only update if this is the first intersection or closer to pad
                seg_id = id(seg)
                if seg_id not in segments_to_shorten_map or dist_from_pad > segments_to_shorten_map[seg_id][5]:
                    if keep_start:
                        segments_to_shorten_map[seg_id] = (seg, seg.start_x, seg.start_y, px, py, dist_from_pad)
                    else:
                        segments_to_shorten_map[seg_id] = (seg, px, py, seg.end_x, seg.end_y, dist_from_pad)

                    # Find and mark segments beyond the intersection for removal
                    removed = find_segments_beyond_intersection(
                        existing_segments, seg, px, py, pad_locs, keep_start
                    )
                    for r in removed:
                        if r not in segments_to_remove:
                            segments_to_remove.append(r)

    # Convert map to list (without the distance field)
    segments_to_shorten = [(s[0], s[1], s[2], s[3], s[4]) for s in segments_to_shorten_map.values()]

    return segments_to_remove, segments_to_shorten


def trace_to_pad(segments: List[Segment], current_seg: Segment, from_end: str,
                 pad_locs: Set[Tuple[float, float]], visited: Set[int] = None) -> bool:
    """
    Trace from one end of a segment to see if it eventually reaches a pad.
    """
    if visited is None:
        visited = set()

    if id(current_seg) in visited:
        return False
    visited.add(id(current_seg))

    if from_end == 'start':
        check_x, check_y = current_seg.start_x, current_seg.start_y
    else:
        check_x, check_y = current_seg.end_x, current_seg.end_y

    check_key = (round(check_x, 3), round(check_y, 3))
    if check_key in pad_locs:
        return True

    # Find connected segments
    for seg in segments:
        if id(seg) == id(current_seg):
            continue

        # Check if this segment connects at the check point
        start_key = (round(seg.start_x, 3), round(seg.start_y, 3))
        end_key = (round(seg.end_x, 3), round(seg.end_y, 3))

        if start_key == check_key:
            if trace_to_pad(segments, seg, 'end', pad_locs, visited):
                return True
        elif end_key == check_key:
            if trace_to_pad(segments, seg, 'start', pad_locs, visited):
                return True

    return False


def find_segments_beyond_intersection(segments: List[Segment], intersected_seg: Segment,
                                       int_x: float, int_y: float, pad_locs: Set[Tuple[float, float]],
                                       keep_start_side: bool) -> List[Segment]:
    """
    Find all segments on the far side of the intersection (away from pad).
    """
    to_remove = []

    # Determine which end of the intersected segment to trace from
    if keep_start_side:
        # We're keeping start->intersection, so trace from end side
        trace_x, trace_y = intersected_seg.end_x, intersected_seg.end_y
    else:
        # We're keeping intersection->end, so trace from start side
        trace_x, trace_y = intersected_seg.start_x, intersected_seg.start_y

    trace_key = (round(trace_x, 3), round(trace_y, 3))

    # BFS to find all connected segments on this side
    visited = {id(intersected_seg)}
    queue = [trace_key]

    while queue:
        current_key = queue.pop(0)

        for seg in segments:
            if id(seg) in visited:
                continue

            start_key = (round(seg.start_x, 3), round(seg.start_y, 3))
            end_key = (round(seg.end_x, 3), round(seg.end_y, 3))

            if start_key == current_key:
                visited.add(id(seg))
                to_remove.append(seg)
                if end_key not in pad_locs:
                    queue.append(end_key)
            elif end_key == current_key:
                visited.add(id(seg))
                to_remove.append(seg)
                if start_key not in pad_locs:
                    queue.append(start_key)

    return to_remove


def get_stub_segments_for_endpoint(pcb_data: PCBData, net_id: int,
                                    endpoint: Tuple[float, float, str]) -> List[Segment]:
    """Get all segments of a stub that contains the given endpoint."""
    ex, ey, elayer = endpoint
    endpoint_key = (round(ex, 3), round(ey, 3))

    # Get all segments for this net
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]

    # Find segments connected to this endpoint and trace the stub
    stub_segments = []
    visited = set()
    queue = [endpoint_key]

    while queue:
        current_key = queue.pop(0)

        for seg in net_segments:
            if id(seg) in visited:
                continue

            start_key = (round(seg.start_x, 3), round(seg.start_y, 3))
            end_key = (round(seg.end_x, 3), round(seg.end_y, 3))

            if start_key == current_key or end_key == current_key:
                visited.add(id(seg))
                stub_segments.append(seg)
                # Add the other end to explore
                other_key = end_key if start_key == current_key else start_key
                if other_key not in [q for q in queue]:
                    queue.append(other_key)

    return stub_segments


def _try_route_direction(router: AStarRouter, pcb_data: PCBData, net_id: int,
                          start_endpoint: Tuple[float, float, str],
                          end_endpoint: Tuple[float, float, str]) -> Optional[List[State]]:
    """
    Try routing from start_endpoint to end_endpoint.
    Returns the path if successful, None otherwise.
    """
    start_x, start_y, start_layer = start_endpoint
    end_x, end_y, end_layer = end_endpoint

    print(f"Routing from ({start_x:.3f}, {start_y:.3f}) on {start_layer}")
    print(f"       to   ({end_x:.3f}, {end_y:.3f}) on {end_layer}")

    # Get target stub segments (QFN side - we can connect anywhere on these)
    target_segments = get_stub_segments_for_endpoint(pcb_data, net_id, end_endpoint)

    # Start from BGA endpoint (fixed point, outside BGA exclusion zone)
    start = State(start_x, start_y, start_layer)
    goal = State(end_x, end_y, end_layer)

    path = None

    # Use endpoint-to-segment routing: from BGA endpoint to any point on QFN stub
    if target_segments:
        print(f"  Endpoint-to-segment routing (start -> {len(target_segments)} target segments)...")
        result = router.route_endpoint_to_segments(start, target_segments)
        if result:
            path, _ = result

    # Fall back to endpoint-to-endpoint if no target stub segments available
    if path is None and not target_segments:
        print("  Trying endpoint-to-endpoint routing...")
        path = router.route(start, goal, use_escape_zone=True)

    return path


def route_net(pcb_data: PCBData, net_id: int, config: RouteConfig) -> Optional[RouteResult]:
    """
    Route a single net from its stub endpoints.
    Tries both directions if the first attempt fails.

    Returns RouteResult with path, new segments/vias.
    """
    endpoints = find_stub_endpoints(pcb_data, net_id)

    if len(endpoints) < 2:
        print(f"Net {net_id}: Not enough endpoints to route ({len(endpoints)} found)")
        return None

    if len(endpoints) > 2:
        print(f"Net {net_id}: Multiple endpoints ({len(endpoints)}), routing first two")

    # Randomly choose starting direction to distribute routes more evenly
    # Use net_id as seed for reproducibility
    import random
    rng = random.Random(net_id)
    if rng.random() < 0.5:
        endpoints = endpoints[::-1]  # Reverse order

    # Build obstacles excluding this net
    obstacles = ObstacleGrid(pcb_data, config, exclude_net_id=net_id)

    path = None

    # Coarse-to-fine routing: first try with larger grid step for speed
    if config.use_coarse_first and config.coarse_grid_step > config.grid_step:
        from dataclasses import replace
        coarse_config = replace(config,
                                grid_step=config.coarse_grid_step,
                                max_iterations=config.max_iterations // 5,  # Fewer iterations for coarse
                                heuristic_weight=0.5)  # More directed search for coarse
        coarse_obstacles = ObstacleGrid(pcb_data, coarse_config, exclude_net_id=net_id)
        coarse_router = AStarRouter(coarse_obstacles, coarse_config)

        print(f"  Trying coarse routing (grid={config.coarse_grid_step}mm)...")
        coarse_path = _try_route_direction(coarse_router, pcb_data, net_id, endpoints[0], endpoints[1])
        if coarse_path is None:
            coarse_path = _try_route_direction(coarse_router, pcb_data, net_id, endpoints[1], endpoints[0])

        if coarse_path:
            print(f"  Coarse path found with {len(coarse_path)} points, refining...")
            # Use the coarse path - the smoothing step later will clean it up
            # and collision checks during segment generation will catch issues
            path = coarse_path

    # If coarse routing didn't find a path (or isn't enabled), use fine routing
    if path is None:
        print(f"  Fine routing (grid={config.grid_step}mm)...")
        # Create router with fine grid
        router = AStarRouter(obstacles, config)

        # Try routing in the first direction
        path = _try_route_direction(router, pcb_data, net_id, endpoints[0], endpoints[1])

        # If first direction failed, try the reverse direction
        if path is None:
            print("  First direction failed, trying reverse direction...")
            path = _try_route_direction(router, pcb_data, net_id, endpoints[1], endpoints[0])

    if path is None:
        return None

    # Create fine router for smoothing (even if coarse path was used)
    router = AStarRouter(obstacles, config)

    # Smooth path first (removes zigzag staircase patterns)
    smoothed = router.smooth_path(path)
    print(f"Smoothed from {len(path)} to {len(smoothed)} points")

    # Then simplify (merges remaining collinear segments)
    simplified = router.simplify_path(smoothed)
    print(f"Simplified to {len(simplified)} waypoints")

    # Snap first/last path points to nearby stub endpoints or pads
    # This ensures new route segments connect properly to stubs
    existing_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    pads = pcb_data.pads_by_net.get(net_id, [])

    # Collect all connection points (segment endpoints and pads on same layer)
    snap_tolerance = 0.15  # mm - same as near_endpoint_tolerance

    for idx in [0, -1]:  # First and last path point
        state = simplified[idx]

        # Check all segment endpoints on same layer
        best_dist = snap_tolerance
        best_point = None

        for seg in existing_segments:
            if seg.layer != state.layer:
                continue

            for sx, sy in [(seg.start_x, seg.start_y), (seg.end_x, seg.end_y)]:
                dist = math.sqrt((state.x - sx)**2 + (state.y - sy)**2)
                if dist < best_dist:
                    best_dist = dist
                    best_point = (sx, sy)

        # Also check pad locations
        for pad in pads:
            dist = math.sqrt((state.x - pad.global_x)**2 + (state.y - pad.global_y)**2)
            if dist < best_dist:
                best_dist = dist
                best_point = (pad.global_x, pad.global_y)

        if best_point is not None:
            simplified[idx] = State(best_point[0], best_point[1], state.layer)

    # Debug: print first few path points
    if len(simplified) >= 1:
        s = simplified[0]
        print(f"  First point: ({s.x}, {s.y}) on {s.layer}")

    # Convert path to segments and vias
    new_segments = path_to_segments(simplified, net_id, config.track_width)
    new_vias = path_to_vias(simplified, net_id, config.via_size, config.via_drill)

    # Check if new route intersects existing stubs and trim if needed
    pads = pcb_data.pads_by_net.get(net_id, [])
    segments_to_remove, segments_to_shorten = find_stub_intersection_and_trim(
        pcb_data, net_id, simplified, pads
    )

    # If we shortened a stub, also trim the new route segments that go beyond
    # the intersection point (toward the old stub endpoint)
    # IMPORTANT: Only do this for DESTINATION-side intersections (path end), not
    # source-side intersections (path start). The source intersection is where
    # the route BEGINS, so segments starting there are part of the route and must be kept.
    if segments_to_shorten:
        # Get the path endpoint (destination) location
        path_end = simplified[-1] if simplified else None
        path_start = simplified[0] if simplified else None

        # Get intersection points only from DESTINATION-side shortened segments
        destination_intersection_points = set()
        for seg, new_sx, new_sy, new_ex, new_ey in segments_to_shorten:
            # The intersection point is the NEW endpoint (not the original)
            if (abs(new_sx - seg.start_x) < 0.01 and abs(new_sy - seg.start_y) < 0.01):
                # Kept start, so intersection is at new end
                intersection_pt = (round(new_ex, 3), round(new_ey, 3))
            else:
                # Kept end, so intersection is at new start
                intersection_pt = (round(new_sx, 3), round(new_sy, 3))

            # Only add if this intersection is near the path END (destination), not the start (source)
            if path_end is not None:
                dist_to_end = math.sqrt((intersection_pt[0] - path_end.x)**2 + (intersection_pt[1] - path_end.y)**2)
                dist_to_start = math.sqrt((intersection_pt[0] - path_start.x)**2 + (intersection_pt[1] - path_start.y)**2) if path_start else float('inf')

                # Only consider this a destination intersection if it's closer to path end than path start
                if dist_to_end < dist_to_start and dist_to_end < 0.2:  # Within tolerance of path end
                    destination_intersection_points.add(intersection_pt)

        # Remove new segments that go beyond DESTINATION intersection points
        filtered_new_segments = []
        for seg in new_segments:
            start_key = (round(seg.start_x, 3), round(seg.start_y, 3))
            end_key = (round(seg.end_x, 3), round(seg.end_y, 3))

            # Check if this segment starts at a DESTINATION intersection and goes toward
            # what was the original stub endpoint (goal)
            if start_key in destination_intersection_points:
                # This segment starts at destination intersection - remove it
                print(f"Removing new segment beyond destination intersection: ({seg.start_x:.3f}, {seg.start_y:.3f}) -> ({seg.end_x:.3f}, {seg.end_y:.3f})")
                continue

            filtered_new_segments.append(seg)

        new_segments = filtered_new_segments

    if segments_to_remove:
        print(f"Removing {len(segments_to_remove)} segments beyond intersection")
    if segments_to_shorten:
        print(f"Shortening {len(segments_to_shorten)} segments at intersection")

    return RouteResult(
        path=simplified,
        new_segments=new_segments,
        new_vias=new_vias,
        segments_to_remove=segments_to_remove,
        segments_to_shorten=segments_to_shorten
    )


def path_to_segments(path: List[State], net_id: int, track_width: float) -> List[Segment]:
    """Convert a path to KiCad segment objects."""
    segments = []
    import uuid

    for i in range(len(path) - 1):
        curr = path[i]
        next_ = path[i + 1]

        # Skip if layer change (via, not segment)
        if curr.layer != next_.layer:
            continue

        seg = Segment(
            start_x=curr.x,
            start_y=curr.y,
            end_x=next_.x,
            end_y=next_.y,
            width=track_width,
            layer=curr.layer,
            net_id=net_id,
            uuid=str(uuid.uuid4())
        )
        segments.append(seg)

    return segments


def path_to_vias(path: List[State], net_id: int, via_size: float, via_drill: float) -> List[Via]:
    """Extract vias from path (layer transition points)."""
    vias = []
    import uuid

    for i in range(len(path) - 1):
        curr = path[i]
        next_ = path[i + 1]

        if curr.layer != next_.layer:
            via = Via(
                x=curr.x,
                y=curr.y,
                size=via_size,
                drill=via_drill,
                layers=['F.Cu', 'B.Cu'],  # Through-hole via
                net_id=net_id,
                uuid=str(uuid.uuid4())
            )
            vias.append(via)

    return vias


def apply_route_result_to_pcb(input_path: str, output_path: str, result: RouteResult) -> bool:
    """
    Apply routing result to PCB file by adding new segments and vias,
    and removing/shortening existing segments as needed.
    """
    import re

    with open(input_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # First, handle segment removals
    for seg in result.segments_to_remove:
        # Use original string coordinates if available, otherwise format floats
        sx_str = seg.start_x_str if seg.start_x_str else f"{seg.start_x:.6f}"
        sy_str = seg.start_y_str if seg.start_y_str else f"{seg.start_y:.6f}"
        ex_str = seg.end_x_str if seg.end_x_str else f"{seg.end_x:.6f}"
        ey_str = seg.end_y_str if seg.end_y_str else f"{seg.end_y:.6f}"
        pattern = (
            rf'\(segment\s+'
            rf'\(start\s+{re.escape(sx_str)}\s+{re.escape(sy_str)}\)\s+'
            rf'\(end\s+{re.escape(ex_str)}\s+{re.escape(ey_str)}\)\s+'
            rf'\(width\s+[\d.]+\)\s+'
            rf'\(layer\s+"{re.escape(seg.layer)}"\)\s+'
            rf'\(net\s+{seg.net_id}\)\s+'
            rf'\(uuid\s+"[^"]+"\)\s*\)\s*'
        )
        content = re.sub(pattern, '', content)

    # Handle segment shortenings (remove old, add new shortened version)
    from kicad_writer import generate_segment_sexpr

    shortened_segments = []
    for seg, new_start_x, new_start_y, new_end_x, new_end_y in result.segments_to_shorten:
        # Use original string coordinates if available
        sx_str = seg.start_x_str if seg.start_x_str else f"{seg.start_x:.6f}"
        sy_str = seg.start_y_str if seg.start_y_str else f"{seg.start_y:.6f}"
        ex_str = seg.end_x_str if seg.end_x_str else f"{seg.end_x:.6f}"
        ey_str = seg.end_y_str if seg.end_y_str else f"{seg.end_y:.6f}"
        pattern = (
            rf'\(segment\s+'
            rf'\(start\s+{re.escape(sx_str)}\s+{re.escape(sy_str)}\)\s+'
            rf'\(end\s+{re.escape(ex_str)}\s+{re.escape(ey_str)}\)\s+'
            rf'\(width\s+[\d.]+\)\s+'
            rf'\(layer\s+"{re.escape(seg.layer)}"\)\s+'
            rf'\(net\s+{seg.net_id}\)\s+'
            rf'\(uuid\s+"[^"]+"\)\s*\)\s*'
        )
        content = re.sub(pattern, '', content)

        # Create the shortened segment
        shortened_segments.append(generate_segment_sexpr(
            (new_start_x, new_start_y),
            (new_end_x, new_end_y),
            seg.width,
            seg.layer,
            seg.net_id
        ))

    # Generate new segment and via S-expressions
    from kicad_writer import generate_via_sexpr

    elements = []

    # Add shortened segments
    elements.extend(shortened_segments)

    # Add new routed segments
    for seg in result.new_segments:
        elements.append(generate_segment_sexpr(
            (seg.start_x, seg.start_y),
            (seg.end_x, seg.end_y),
            seg.width,
            seg.layer,
            seg.net_id
        ))

    # Add new vias
    for via in result.new_vias:
        elements.append(generate_via_sexpr(
            via.x,
            via.y,
            via.size,
            via.drill,
            via.layers,
            via.net_id
        ))

    routing_text = '\n'.join(elements)

    # Find the last closing parenthesis
    last_paren = content.rfind(')')

    if last_paren == -1:
        print("Error: Could not find closing parenthesis in PCB file")
        return False

    # Insert routing before the final closing paren
    if routing_text.strip():
        new_content = content[:last_paren] + '\n' + routing_text + '\n' + content[last_paren:]
    else:
        new_content = content

    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(new_content)

    return True


if __name__ == "__main__":
    import sys

    # Default test: route DATA_0 on fanout_starting_point.kicad_pcb
    input_file = "fanout_starting_point.kicad_pcb"
    output_file = "routed_output.kicad_pcb"
    net_name = "Net-(U2A-DATA_0)"

    if len(sys.argv) > 1:
        input_file = sys.argv[1]
    if len(sys.argv) > 2:
        net_name = sys.argv[2]
    if len(sys.argv) > 3:
        output_file = sys.argv[3]

    print(f"Loading {input_file}...")
    pcb_data = parse_kicad_pcb(input_file)

    # Find the net
    net_id = None
    for nid, net in pcb_data.nets.items():
        if net.name == net_name:
            net_id = nid
            break

    if net_id is None:
        print(f"Net '{net_name}' not found!")
        sys.exit(1)

    print(f"Found net '{net_name}' with id {net_id}")

    # Configure routing
    config = RouteConfig(
        track_width=0.1,
        clearance=0.1,
        via_size=0.4,
        via_drill=0.2,
        grid_step=0.1,
        via_cost=0.5,
        layers=['F.Cu', 'In2.Cu', 'B.Cu']  # Available layers
    )

    # Route
    result = route_net(pcb_data, net_id, config)

    if result:
        print(f"\nRoute found with {len(result.path)} waypoints:")
        for state in result.path:
            print(f"  ({state.x:.3f}, {state.y:.3f}) on {state.layer}")

        print(f"\nGenerated {len(result.new_segments)} track segments and {len(result.new_vias)} vias")

        # Write to output file
        print(f"\nWriting output to {output_file}...")
        success = apply_route_result_to_pcb(input_file, output_file, result)
        if success:
            print(f"Successfully wrote {output_file}")
        else:
            print("Failed to write output file!")
    else:
        print("\nRouting failed!")
