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


@dataclass(frozen=True)
class State:
    """A state in the A* search: position + layer."""
    x: float
    y: float
    layer: str

    def __hash__(self):
        # Round to grid for hashing
        return hash((round(self.x, 4), round(self.y, 4), self.layer))

    def __eq__(self, other):
        return (round(self.x, 4) == round(other.x, 4) and
                round(self.y, 4) == round(other.y, 4) and
                self.layer == other.layer)


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

    def _build_from_vias(self):
        """Add existing vias as obstacles on all layers."""
        for via in self.pcb_data.vias:
            if via.net_id == self.exclude_net_id:
                continue
            radius = via.size / 2 + self.config.clearance
            # Vias block all layers
            for layer in self.config.layers:
                self._add_circle_obstacle(layer, via.x, via.y, radius)

    def _build_from_pads(self):
        """Add pads as obstacles (except those on excluded net)."""
        for net_id, pads in self.pcb_data.pads_by_net.items():
            if net_id == self.exclude_net_id:
                continue
            for pad in pads:
                # Use average of size as radius approximation
                radius = max(pad.size_x, pad.size_y) / 2 + self.config.clearance
                for layer in pad.layers:
                    if layer in self.config.layers:
                        self._add_circle_obstacle(layer, pad.global_x, pad.global_y, radius)

    def _point_to_segment_distance(self, px: float, py: float,
                                    x1: float, y1: float, x2: float, y2: float) -> float:
        """Calculate minimum distance from point to line segment."""
        dx = x2 - x1
        dy = y2 - y1

        if dx == 0 and dy == 0:
            return math.sqrt((px - x1)**2 + (py - y1)**2)

        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))

        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        return math.sqrt((px - closest_x)**2 + (py - closest_y)**2)

    def _segment_to_segment_distance(self, ax1: float, ay1: float, ax2: float, ay2: float,
                                      bx1: float, by1: float, bx2: float, by2: float) -> float:
        """Calculate minimum distance between two line segments."""
        # Check if segments intersect
        def ccw(A, B, C):
            return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

        A, B = (ax1, ay1), (ax2, ay2)
        C, D = (bx1, by1), (bx2, by2)

        if ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D):
            return 0.0  # Segments intersect

        # Otherwise, find minimum distance between endpoints and segments
        d1 = self._point_to_segment_distance(ax1, ay1, bx1, by1, bx2, by2)
        d2 = self._point_to_segment_distance(ax2, ay2, bx1, by1, bx2, by2)
        d3 = self._point_to_segment_distance(bx1, by1, ax1, ay1, ax2, ay2)
        d4 = self._point_to_segment_distance(bx2, by2, ax1, ay1, ax2, ay2)

        return min(d1, d2, d3, d4)

    def segment_collides(self, x1: float, y1: float, x2: float, y2: float,
                         layer: str) -> bool:
        """Check if a new segment would collide with obstacles."""
        if layer not in self.obstacles_by_layer:
            return False

        # Find cells this segment passes through
        min_x = min(x1, x2) - self.expansion
        max_x = max(x1, x2) + self.expansion
        min_y = min(y1, y2) - self.expansion
        max_y = max(y1, y2) + self.expansion

        cell_min = self._grid_cell(min_x, min_y)
        cell_max = self._grid_cell(max_x, max_y)

        for cx in range(cell_min[0], cell_max[0] + 1):
            for cy in range(cell_min[1], cell_max[1] + 1):
                cell = (cx, cy)
                if cell not in self.obstacles_by_layer[layer]:
                    continue

                for obs_type, obs_data in self.obstacles_by_layer[layer][cell]:
                    if obs_type == 'segment':
                        ox1, oy1, ox2, oy2, obs_exp = obs_data
                        dist = self._segment_to_segment_distance(
                            x1, y1, x2, y2, ox1, oy1, ox2, oy2)
                        if dist < self.expansion + obs_exp:
                            return True
                    elif obs_type == 'circle':
                        cx, cy, radius = obs_data
                        dist = self._point_to_segment_distance(cx, cy, x1, y1, x2, y2)
                        if dist < self.expansion + radius:
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

    def _is_goal(self, state: State, goal: State, tolerance: float = 0.05) -> bool:
        """Check if state is close enough to goal."""
        dx = abs(state.x - goal.x)
        dy = abs(state.y - goal.y)
        return dx <= tolerance and dy <= tolerance and state.layer == goal.layer

    def _snap_to_grid(self, x: float, y: float) -> Tuple[float, float]:
        """Snap coordinates to grid."""
        step = self.config.grid_step
        return (round(x / step) * step, round(y / step) * step)

    def route(self, start: State, goal: State, max_iterations: int = 100000) -> Optional[List[State]]:
        """
        Find a path from start to goal using A*.

        Returns list of States forming the path, or None if no path found.
        """
        # Save original endpoints before snapping
        original_start = start
        original_goal = goal

        # Snap start and goal to grid for A* search
        start = State(*self._snap_to_grid(start.x, start.y), start.layer)
        goal = State(*self._snap_to_grid(goal.x, goal.y), goal.layer)

        # Priority queue: (f_cost, counter, g_cost, state, parent)
        counter = 0
        open_set = [(self._heuristic(start, goal), counter, 0.0, start, None)]

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

            # Expand neighbors
            step = self.config.grid_step

            # Try 8 directions on same layer
            for dx, dy in self.DIRECTIONS:
                nx = current.x + dx * step
                ny = current.y + dy * step

                # Check collision
                if self.obstacles.segment_collides(current.x, current.y, nx, ny, current.layer):
                    continue

                neighbor = State(nx, ny, current.layer)

                # Calculate move cost (diagonal is sqrt(2) * step)
                move_cost = step * self.sqrt2 if (dx != 0 and dy != 0) else step
                new_g = g + move_cost

                if neighbor in closed:
                    continue

                if neighbor not in g_costs or new_g < g_costs[neighbor]:
                    g_costs[neighbor] = new_g
                    parents[neighbor] = current
                    f = new_g + self._heuristic(neighbor, goal)
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
                        f = new_g + self._heuristic(neighbor, goal)
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
                    # But this specific segment doesn't need shortening
                    if at_end and not end_is_pad:
                        # The new route connects at the far end of stub
                        # Find and remove segments beyond
                        removed = find_segments_beyond_intersection(
                            existing_segments, seg, seg.end_x, seg.end_y, pad_locs, True
                        )
                        segments_to_remove.extend(removed)
                    elif at_start and not start_is_pad:
                        removed = find_segments_beyond_intersection(
                            existing_segments, seg, seg.start_x, seg.start_y, pad_locs, False
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


def route_net(pcb_data: PCBData, net_id: int, config: RouteConfig) -> Optional[RouteResult]:
    """
    Route a single net from its stub endpoints.

    Returns RouteResult with path, new segments/vias.
    """
    endpoints = find_stub_endpoints(pcb_data, net_id)

    if len(endpoints) < 2:
        print(f"Net {net_id}: Not enough endpoints to route ({len(endpoints)} found)")
        return None

    if len(endpoints) > 2:
        print(f"Net {net_id}: Multiple endpoints ({len(endpoints)}), routing first two")

    start_x, start_y, start_layer = endpoints[0]
    end_x, end_y, end_layer = endpoints[1]

    print(f"Routing from ({start_x:.3f}, {start_y:.3f}) on {start_layer}")
    print(f"       to   ({end_x:.3f}, {end_y:.3f}) on {end_layer}")

    # Build obstacles excluding this net
    obstacles = ObstacleGrid(pcb_data, config, exclude_net_id=net_id)

    # Create router
    router = AStarRouter(obstacles, config)

    # Route
    start = State(start_x, start_y, start_layer)
    goal = State(end_x, end_y, end_layer)

    path = router.route(start, goal)

    if path is None:
        return None

    # Smooth path first (removes zigzag staircase patterns)
    smoothed = router.smooth_path(path)
    print(f"Smoothed from {len(path)} to {len(smoothed)} points")

    # Then simplify (merges remaining collinear segments)
    simplified = router.simplify_path(smoothed)
    print(f"Simplified to {len(simplified)} waypoints")

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
    if segments_to_shorten:
        # Get all intersection points from shortened segments
        intersection_points = set()
        for seg, new_sx, new_sy, new_ex, new_ey in segments_to_shorten:
            # The intersection point is the NEW endpoint (not the original)
            if (abs(new_sx - seg.start_x) < 0.01 and abs(new_sy - seg.start_y) < 0.01):
                # Kept start, so intersection is at new end
                intersection_points.add((round(new_ex, 3), round(new_ey, 3)))
            else:
                # Kept end, so intersection is at new start
                intersection_points.add((round(new_sx, 3), round(new_sy, 3)))

        # Remove new segments that go beyond intersection points
        filtered_new_segments = []
        for seg in new_segments:
            start_key = (round(seg.start_x, 3), round(seg.start_y, 3))
            end_key = (round(seg.end_x, 3), round(seg.end_y, 3))

            # Check if this segment starts at an intersection and goes toward
            # what was the original stub endpoint (goal)
            if start_key in intersection_points:
                # This segment starts at intersection - check if it goes away from pad
                # We want to REMOVE this segment since the stub is now shortened
                print(f"Removing new segment beyond intersection: ({seg.start_x:.3f}, {seg.start_y:.3f}) -> ({seg.end_x:.3f}, {seg.end_y:.3f})")
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
        # Build a regex pattern to match this specific segment
        # We need to match the segment by its coordinates
        pattern = (
            rf'\(segment\s+'
            rf'\(start\s+{re.escape(f"{seg.start_x:.6f}")}\s+{re.escape(f"{seg.start_y:.6f}")}\)\s+'
            rf'\(end\s+{re.escape(f"{seg.end_x:.6f}")}\s+{re.escape(f"{seg.end_y:.6f}")}\)\s+'
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
        # Remove the original segment
        pattern = (
            rf'\(segment\s+'
            rf'\(start\s+{re.escape(f"{seg.start_x:.6f}")}\s+{re.escape(f"{seg.start_y:.6f}")}\)\s+'
            rf'\(end\s+{re.escape(f"{seg.end_x:.6f}")}\s+{re.escape(f"{seg.end_y:.6f}")}\)\s+'
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
