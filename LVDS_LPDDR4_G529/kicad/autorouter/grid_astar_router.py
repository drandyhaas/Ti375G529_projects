"""
Grid-based A* PCB Router - Integer grid routing for speed.

Works on a discrete grid (default 0.1mm) instead of continuous coordinates.
This provides faster routing since:
1. Integer operations are faster than floating-point
2. Collision detection can be pre-computed and cached
3. State hashing is trivial with integer tuples

The final path is converted back to floating-point coordinates for KiCad output.
"""

import heapq
import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional, Set, FrozenSet
from kicad_parser import parse_kicad_pcb, PCBData, Segment, Via, Pad


@dataclass
class GridRouteConfig:
    """Configuration for grid-based routing."""
    track_width: float = 0.1  # mm
    clearance: float = 0.1  # mm between tracks
    via_size: float = 0.4  # mm via outer diameter
    via_drill: float = 0.2  # mm via drill
    grid_step: float = 0.1  # mm grid resolution (parameterizable)
    via_cost: int = 5  # grid steps equivalent penalty for via
    layers: List[str] = field(default_factory=lambda: ['F.Cu', 'B.Cu'])
    max_iterations: int = 100000
    # Escape zone for congested areas
    escape_radius: float = 2.0  # mm
    escape_clearance: float = 0.02  # mm
    # Heuristic weight (1.0 = A*, lower = more Dijkstra-like)
    heuristic_weight: float = 0.1
    # BGA exclusion zone
    bga_exclusion_zone: Optional[Tuple[float, float, float, float]] = None
    # Stub proximity cost - discourages blocking unrouted nets
    stub_proximity_radius: float = 1.0  # mm - radius around stubs to penalize
    stub_proximity_cost: float = 3.0  # mm equivalent cost at stub center (decays with distance)


class GridCoord:
    """
    Utilities for converting between float (mm) and integer grid coordinates.
    """
    def __init__(self, grid_step: float = 0.1):
        self.grid_step = grid_step
        # Inverse for faster multiplication
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

    def to_float_dist(self, grid_dist: int) -> float:
        """Convert grid distance to mm."""
        return grid_dist * self.grid_step


# Default layer index mapping - will be rebuilt based on config
# Supports all standard KiCad copper layers
ALL_COPPER_LAYERS = [
    'F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu', 'In5.Cu',
    'In6.Cu', 'In7.Cu', 'In8.Cu', 'In9.Cu', 'In10.Cu', 'In11.Cu',
    'In12.Cu', 'In13.Cu', 'In14.Cu', 'In15.Cu', 'In16.Cu', 'In17.Cu',
    'In18.Cu', 'In19.Cu', 'In20.Cu', 'In21.Cu', 'In22.Cu', 'In23.Cu',
    'In24.Cu', 'In25.Cu', 'In26.Cu', 'In27.Cu', 'In28.Cu', 'In29.Cu',
    'In30.Cu', 'B.Cu'
]


class LayerMapper:
    """
    Maps layer names to indices and vice versa.
    Built dynamically from config.layers list.
    """
    def __init__(self, layers: List[str]):
        self.layers = layers
        self.layer_to_idx: Dict[str, int] = {layer: i for i, layer in enumerate(layers)}
        self.idx_to_layer: List[str] = layers

    def get_idx(self, layer_name: str) -> Optional[int]:
        """Get layer index, or None if layer not in config."""
        return self.layer_to_idx.get(layer_name)

    def get_layer(self, idx: int) -> str:
        """Get layer name from index."""
        if 0 <= idx < len(self.idx_to_layer):
            return self.idx_to_layer[idx]
        return 'F.Cu'  # fallback

    def num_layers(self) -> int:
        return len(self.layers)


# Legacy compatibility - will be replaced by LayerMapper instances
LAYER_INDEX = {'F.Cu': 0, 'B.Cu': 1}
INDEX_LAYER = ['F.Cu', 'B.Cu']


class GridState:
    """
    A state in the grid-based A* search.
    Uses integers for fast hashing and comparison.
    """
    __slots__ = ('gx', 'gy', 'layer_idx')

    def __init__(self, gx: int, gy: int, layer_idx: int):
        self.gx = gx
        self.gy = gy
        self.layer_idx = layer_idx

    def __hash__(self):
        # Fast hash for integer tuple
        return hash((self.gx, self.gy, self.layer_idx))

    def __eq__(self, other):
        return (self.gx == other.gx and
                self.gy == other.gy and
                self.layer_idx == other.layer_idx)

    def __repr__(self):
        layer = INDEX_LAYER[self.layer_idx] if self.layer_idx < len(INDEX_LAYER) else f"L{self.layer_idx}"
        return f"GridState({self.gx}, {self.gy}, {layer})"

    def as_tuple(self) -> Tuple[int, int, int]:
        return (self.gx, self.gy, self.layer_idx)


class GridObstacleMap:
    """
    Pre-computed obstacle map on the integer grid.

    Stores:
    - blocked_cells: set of (gx, gy, layer_idx) that are blocked
    - blocked_edges: set of ((gx1,gy1), (gx2,gy2), layer_idx) blocked moves

    This allows O(1) collision checking during A* instead of
    iterating through obstacles.
    """

    def __init__(self, pcb_data: PCBData, config: GridRouteConfig,
                 exclude_net_id: Optional[int] = None,
                 unrouted_stubs: Optional[List[Tuple[float, float]]] = None):
        self.config = config
        self.pcb_data = pcb_data
        self.exclude_net_id = exclude_net_id
        self.coord = GridCoord(config.grid_step)

        # Create layer mapper from config
        self.layer_mapper = LayerMapper(config.layers)

        # Expansion in grid units (half track width + clearance)
        expansion_mm = config.track_width / 2 + config.clearance
        self.expansion_grid = max(1, self.coord.to_grid_dist(expansion_mm))

        # Via expansion in grid units
        via_expansion_mm = config.via_size / 2 + config.clearance
        self.via_expansion_grid = max(1, self.coord.to_grid_dist(via_expansion_mm))

        # Blocked cells per layer: layer_idx -> set of (gx, gy)
        self.blocked_cells: Dict[int, Set[Tuple[int, int]]] = {
            i: set() for i in range(self.layer_mapper.num_layers())
        }

        # Blocked via locations: set of (gx, gy)
        self.blocked_vias: Set[Tuple[int, int]] = set()

        # Stub proximity cost map: (gx, gy) -> cost multiplier (0.0 to 1.0)
        # 1.0 at stub center, decaying to 0.0 at radius edge
        self.stub_proximity: Dict[Tuple[int, int], float] = {}
        self._stub_radius_grid = self.coord.to_grid_dist(config.stub_proximity_radius)
        self._stub_cost_grid = int(config.stub_proximity_cost * 1000 / config.grid_step)

        # BGA zone in grid coordinates
        self.bga_zone_grid: Optional[Tuple[int, int, int, int]] = None
        if config.bga_exclusion_zone:
            min_x, min_y, max_x, max_y = config.bga_exclusion_zone
            gmin_x, gmin_y = self.coord.to_grid(min_x, min_y)
            gmax_x, gmax_y = self.coord.to_grid(max_x, max_y)
            self.bga_zone_grid = (gmin_x, gmin_y, gmax_x, gmax_y)

        # Build the obstacle map
        self._build_from_segments()
        self._build_from_vias()
        self._build_from_pads()

        # Build stub proximity map
        if unrouted_stubs:
            self._build_stub_proximity(unrouted_stubs)

    def _in_bga_zone(self, gx: int, gy: int) -> bool:
        """Check if grid point is in BGA exclusion zone."""
        if self.bga_zone_grid is None:
            return False
        gmin_x, gmin_y, gmax_x, gmax_y = self.bga_zone_grid
        return gmin_x <= gx <= gmax_x and gmin_y <= gy <= gmax_y

    def _mark_circle_blocked(self, cx: float, cy: float, radius_mm: float,
                              layer_idx: int, for_via: bool = False):
        """Mark all grid cells within radius of (cx, cy) as blocked."""
        gcx, gcy = self.coord.to_grid(cx, cy)
        radius_grid = self.coord.to_grid_dist(radius_mm)
        expansion = self.via_expansion_grid if for_via else self.expansion_grid
        total_radius = radius_grid + expansion

        # Mark all cells within the radius
        for dx in range(-total_radius, total_radius + 1):
            for dy in range(-total_radius, total_radius + 1):
                if dx*dx + dy*dy <= total_radius * total_radius:
                    gx, gy = gcx + dx, gcy + dy
                    if for_via:
                        self.blocked_vias.add((gx, gy))
                    else:
                        self.blocked_cells[layer_idx].add((gx, gy))

    def _mark_segment_blocked(self, x1: float, y1: float, x2: float, y2: float,
                               half_width_mm: float, layer_idx: int):
        """Mark all grid cells along a segment as blocked.

        Also marks cells in blocked_vias to prevent vias from being placed
        too close to segments (vias span all layers and need clearance).
        """
        gx1, gy1 = self.coord.to_grid(x1, y1)
        gx2, gy2 = self.coord.to_grid(x2, y2)

        # Track expansion for blocking (half track width + clearance)
        expansion_mm = half_width_mm + self.config.clearance
        expansion_grid = max(1, self.coord.to_grid_dist(expansion_mm))

        # Via expansion: via needs clearance from segment
        # Distance required = via_radius + track_half_width + clearance
        via_block_mm = self.config.via_size / 2 + half_width_mm + self.config.clearance
        via_block_grid = max(1, self.coord.to_grid_dist(via_block_mm))

        # Use Bresenham-like approach to mark cells along segment
        dx = abs(gx2 - gx1)
        dy = abs(gy2 - gy1)
        sx = 1 if gx1 < gx2 else -1
        sy = 1 if gy1 < gy2 else -1

        def mark_point(gx, gy):
            # Mark for track clearance
            for ex in range(-expansion_grid, expansion_grid + 1):
                for ey in range(-expansion_grid, expansion_grid + 1):
                    self.blocked_cells[layer_idx].add((gx + ex, gy + ey))
            # Mark for via clearance (vias span all layers)
            for ex in range(-via_block_grid, via_block_grid + 1):
                for ey in range(-via_block_grid, via_block_grid + 1):
                    if ex*ex + ey*ey <= via_block_grid * via_block_grid:
                        self.blocked_vias.add((gx + ex, gy + ey))

        if dx == 0 and dy == 0:
            # Single point
            mark_point(gx1, gy1)
            return

        # Walk along the segment
        gx, gy = gx1, gy1
        if dx > dy:
            err = dx // 2
            while gx != gx2:
                mark_point(gx, gy)
                err -= dy
                if err < 0:
                    gy += sy
                    err += dx
                gx += sx
        else:
            err = dy // 2
            while gy != gy2:
                mark_point(gx, gy)
                err -= dx
                if err < 0:
                    gx += sx
                    err += dy
                gy += sy

        # Mark endpoint
        mark_point(gx2, gy2)

    def _build_from_segments(self):
        """Add existing track segments as obstacles."""
        for seg in self.pcb_data.segments:
            if seg.net_id == self.exclude_net_id:
                continue
            layer_idx = self.layer_mapper.get_idx(seg.layer)
            if layer_idx is None:
                continue
            self._mark_segment_blocked(seg.start_x, seg.start_y,
                                        seg.end_x, seg.end_y,
                                        seg.width / 2, layer_idx)

    def _build_from_vias(self):
        """Add existing vias as obstacles."""
        for via in self.pcb_data.vias:
            if via.net_id == self.exclude_net_id:
                continue
            # Skip vias in BGA zone
            gx, gy = self.coord.to_grid(via.x, via.y)
            if self._in_bga_zone(gx, gy):
                continue
            # Vias block all layers
            for layer_idx in range(self.layer_mapper.num_layers()):
                self._mark_circle_blocked(via.x, via.y, via.size / 2, layer_idx)
            # Also mark as blocked for via placement
            self._mark_circle_blocked(via.x, via.y, via.size / 2, 0, for_via=True)

    def _build_from_pads(self):
        """Add pads as obstacles."""
        for net_id, pads in self.pcb_data.pads_by_net.items():
            if net_id == self.exclude_net_id:
                continue
            for pad in pads:
                radius = max(pad.size_x, pad.size_y) / 2
                for layer in pad.layers:
                    layer_idx = self.layer_mapper.get_idx(layer)
                    if layer_idx is not None:
                        self._mark_circle_blocked(pad.global_x, pad.global_y,
                                                   radius, layer_idx)

    def _build_stub_proximity(self, stubs: List[Tuple[float, float]]):
        """Build proximity cost map around unrouted stub endpoints.

        Creates a cost field that decays linearly from stub center to radius edge.
        This discourages routes from passing too close to unrouted stubs.
        """
        radius = self._stub_radius_grid
        for stub_x, stub_y in stubs:
            gcx, gcy = self.coord.to_grid(stub_x, stub_y)
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    dist_sq = dx * dx + dy * dy
                    if dist_sq <= radius * radius:
                        gx, gy = gcx + dx, gcy + dy
                        # Linear decay: 1.0 at center, 0.0 at edge
                        dist = math.sqrt(dist_sq)
                        proximity = 1.0 - (dist / radius) if radius > 0 else 1.0
                        # Keep max proximity if multiple stubs overlap
                        if (gx, gy) in self.stub_proximity:
                            self.stub_proximity[(gx, gy)] = max(
                                self.stub_proximity[(gx, gy)], proximity)
                        else:
                            self.stub_proximity[(gx, gy)] = proximity

    def get_stub_proximity_cost(self, gx: int, gy: int) -> int:
        """Get the stub proximity cost for a grid cell (integer cost units)."""
        proximity = self.stub_proximity.get((gx, gy), 0.0)
        return int(proximity * self._stub_cost_grid)

    def is_blocked(self, gx: int, gy: int, layer_idx: int) -> bool:
        """Check if a grid cell is blocked. O(1) lookup."""
        # BGA zone is always blocked
        if self._in_bga_zone(gx, gy):
            return True
        return (gx, gy) in self.blocked_cells.get(layer_idx, set())

    def is_move_blocked(self, gx1: int, gy1: int, gx2: int, gy2: int,
                        layer_idx: int) -> bool:
        """
        Check if moving from (gx1, gy1) to (gx2, gy2) is blocked.
        Uses interpolation to check intermediate cells.
        """
        # Check endpoints
        if self.is_blocked(gx1, gy1, layer_idx) or self.is_blocked(gx2, gy2, layer_idx):
            return True

        # For orthogonal moves, just check endpoints
        dx = gx2 - gx1
        dy = gy2 - gy1

        if abs(dx) <= 1 and abs(dy) <= 1:
            # Single step move - endpoints already checked
            return False

        # For longer moves (JPS jumps), check intermediate points
        steps = max(abs(dx), abs(dy))
        for i in range(1, steps):
            t = i / steps
            gx = gx1 + round(dx * t)
            gy = gy1 + round(dy * t)
            if self.is_blocked(gx, gy, layer_idx):
                return True

        return False

    def is_via_blocked(self, gx: int, gy: int) -> bool:
        """Check if placing a via at (gx, gy) would be blocked."""
        return (gx, gy) in self.blocked_vias

    def add_segment(self, gx1: int, gy1: int, gx2: int, gy2: int, layer_idx: int):
        """Add a new segment to the obstacle map (for incremental routing)."""
        x1, y1 = self.coord.to_float(gx1, gy1)
        x2, y2 = self.coord.to_float(gx2, gy2)
        self._mark_segment_blocked(x1, y1, x2, y2,
                                    self.config.track_width / 2, layer_idx)

    def add_via(self, gx: int, gy: int):
        """Add a new via to the obstacle map."""
        x, y = self.coord.to_float(gx, gy)
        for layer_idx in range(self.layer_mapper.num_layers()):
            self._mark_circle_blocked(x, y, self.config.via_size / 2, layer_idx)
        self._mark_circle_blocked(x, y, self.config.via_size / 2, 0, for_via=True)


class GridAStarRouter:
    """
    A* router working on integer grid coordinates.
    """

    # 8 directions as grid offsets (octilinear)
    DIRECTIONS = [
        (1, 0),    # East
        (1, -1),   # NE
        (0, -1),   # North
        (-1, -1),  # NW
        (-1, 0),   # West
        (-1, 1),   # SW
        (0, 1),    # South
        (1, 1),    # SE
    ]

    # Cost multiplier for diagonal moves (sqrt(2) * 1000 for integer math)
    DIAG_COST = 1414  # ~sqrt(2) * 1000
    ORTHO_COST = 1000

    def __init__(self, obstacles: GridObstacleMap, config: GridRouteConfig):
        self.obstacles = obstacles
        self.config = config
        self.coord = GridCoord(config.grid_step)

        # Get layer mapper from obstacles
        self.layer_mapper = obstacles.layer_mapper

        # Via cost in integer units
        self.via_cost = config.via_cost * self.ORTHO_COST

        # Heuristic weight
        self.h_weight = config.heuristic_weight

    def _heuristic(self, state: GridState, goal: GridState) -> int:
        """
        Admissible heuristic using octile distance (integer).
        Accounts for diagonal movement.
        """
        dx = abs(state.gx - goal.gx)
        dy = abs(state.gy - goal.gy)

        # Octile distance: diagonal steps + remaining orthogonal
        diag = min(dx, dy)
        orth = abs(dx - dy)
        h = diag * self.DIAG_COST + orth * self.ORTHO_COST

        # Add via cost if on different layer
        if state.layer_idx != goal.layer_idx:
            h += self.via_cost

        return h

    def _heuristic_to_points(self, state: GridState,
                             targets: List[Tuple[int, int, int]]) -> int:
        """Heuristic: minimum distance to any target point."""
        min_h = float('inf')
        for gx, gy, layer_idx in targets:
            dx = abs(state.gx - gx)
            dy = abs(state.gy - gy)
            diag = min(dx, dy)
            orth = abs(dx - dy)
            h = diag * self.DIAG_COST + orth * self.ORTHO_COST
            if state.layer_idx != layer_idx:
                h += self.via_cost
            if h < min_h:
                min_h = h
        return int(min_h)

    def route(self, start: GridState, goal: GridState,
              max_iterations: int = None) -> Optional[List[GridState]]:
        """
        Find path from start to goal using A*.

        Returns list of GridState forming the path, or None if not found.
        """
        if max_iterations is None:
            max_iterations = self.config.max_iterations

        # Priority queue: (f_cost, counter, g_cost, state)
        counter = 0
        h = int(self._heuristic(start, goal) * self.h_weight)
        open_set = [(h, counter, 0, start)]

        # Best g_cost to reach each state
        g_costs: Dict[Tuple[int, int, int], int] = {start.as_tuple(): 0}

        # Parent tracking for path reconstruction
        parents: Dict[Tuple[int, int, int], Optional[Tuple[int, int, int]]] = {
            start.as_tuple(): None
        }

        # Closed set
        closed: Set[Tuple[int, int, int]] = set()

        iterations = 0

        while open_set and iterations < max_iterations:
            iterations += 1

            f, _, g, current = heapq.heappop(open_set)
            current_key = current.as_tuple()

            if current_key in closed:
                continue

            closed.add(current_key)

            # Goal check
            if (current.gx == goal.gx and current.gy == goal.gy and
                current.layer_idx == goal.layer_idx):
                # Reconstruct path
                path = []
                key = current_key
                while key is not None:
                    path.append(GridState(*key))
                    key = parents[key]
                path.reverse()
                print(f"Grid route found in {iterations} iterations, path length: {len(path)}")
                return path

            # Expand neighbors - 8 directions
            for dx, dy in self.DIRECTIONS:
                ngx = current.gx + dx
                ngy = current.gy + dy

                if self.obstacles.is_move_blocked(current.gx, current.gy,
                                                   ngx, ngy, current.layer_idx):
                    continue

                neighbor_key = (ngx, ngy, current.layer_idx)

                if neighbor_key in closed:
                    continue

                # Calculate move cost
                if dx != 0 and dy != 0:
                    move_cost = self.DIAG_COST
                else:
                    move_cost = self.ORTHO_COST

                new_g = g + move_cost

                if neighbor_key not in g_costs or new_g < g_costs[neighbor_key]:
                    g_costs[neighbor_key] = new_g
                    parents[neighbor_key] = current_key
                    neighbor = GridState(*neighbor_key)
                    h = int(self._heuristic(neighbor, goal) * self.h_weight)
                    f = new_g + h
                    counter += 1
                    heapq.heappush(open_set, (f, counter, new_g, neighbor))

            # Try via to other layers
            if not self.obstacles.is_via_blocked(current.gx, current.gy):
                for layer_idx in range(self.layer_mapper.num_layers()):
                    if layer_idx == current.layer_idx:
                        continue

                    neighbor_key = (current.gx, current.gy, layer_idx)

                    if neighbor_key in closed:
                        continue

                    new_g = g + self.via_cost

                    if neighbor_key not in g_costs or new_g < g_costs[neighbor_key]:
                        g_costs[neighbor_key] = new_g
                        parents[neighbor_key] = current_key
                        neighbor = GridState(*neighbor_key)
                        h = int(self._heuristic(neighbor, goal) * self.h_weight)
                        f = new_g + h
                        counter += 1
                        heapq.heappush(open_set, (f, counter, new_g, neighbor))

        print(f"No grid route found after {iterations} iterations")
        return None

    def route_to_targets(self, start: GridState,
                         targets: List[Tuple[int, int, int]],
                         max_iterations: int = None) -> Optional[Tuple[List[GridState], Tuple[int, int, int]]]:
        """
        Find path from start to any of the target points.

        Returns (path, reached_target) or None.
        """
        if max_iterations is None:
            max_iterations = self.config.max_iterations

        # Convert targets to a set for O(1) lookup
        target_set = set(targets)

        counter = 0
        h = int(self._heuristic_to_points(start, targets) * self.h_weight)
        open_set = [(h, counter, 0, start)]

        g_costs: Dict[Tuple[int, int, int], int] = {start.as_tuple(): 0}
        parents: Dict[Tuple[int, int, int], Optional[Tuple[int, int, int]]] = {
            start.as_tuple(): None
        }
        closed: Set[Tuple[int, int, int]] = set()

        iterations = 0

        while open_set and iterations < max_iterations:
            iterations += 1

            f, _, g, current = heapq.heappop(open_set)
            current_key = current.as_tuple()

            if current_key in closed:
                continue

            closed.add(current_key)

            # Check if we reached a target
            if current_key in target_set:
                path = []
                key = current_key
                while key is not None:
                    path.append(GridState(*key))
                    key = parents[key]
                path.reverse()
                print(f"Grid route to target found in {iterations} iterations")
                return (path, current_key)

            # Expand neighbors
            for dx, dy in self.DIRECTIONS:
                ngx = current.gx + dx
                ngy = current.gy + dy

                if self.obstacles.is_move_blocked(current.gx, current.gy,
                                                   ngx, ngy, current.layer_idx):
                    continue

                neighbor_key = (ngx, ngy, current.layer_idx)

                if neighbor_key in closed:
                    continue

                move_cost = self.DIAG_COST if (dx != 0 and dy != 0) else self.ORTHO_COST
                new_g = g + move_cost

                if neighbor_key not in g_costs or new_g < g_costs[neighbor_key]:
                    g_costs[neighbor_key] = new_g
                    parents[neighbor_key] = current_key
                    neighbor = GridState(*neighbor_key)
                    h = int(self._heuristic_to_points(neighbor, targets) * self.h_weight)
                    f = new_g + h
                    counter += 1
                    heapq.heappush(open_set, (f, counter, new_g, neighbor))

            # Try via
            if not self.obstacles.is_via_blocked(current.gx, current.gy):
                for layer_idx in range(self.layer_mapper.num_layers()):
                    if layer_idx == current.layer_idx:
                        continue

                    neighbor_key = (current.gx, current.gy, layer_idx)

                    if neighbor_key in closed:
                        continue

                    new_g = g + self.via_cost

                    if neighbor_key not in g_costs or new_g < g_costs[neighbor_key]:
                        g_costs[neighbor_key] = new_g
                        parents[neighbor_key] = current_key
                        neighbor = GridState(*neighbor_key)
                        h = int(self._heuristic_to_points(neighbor, targets) * self.h_weight)
                        f = new_g + h
                        counter += 1
                        heapq.heappush(open_set, (f, counter, new_g, neighbor))

        print(f"No grid route to targets found after {iterations} iterations")
        return None

    def route_segments_to_segments(self, source_segments: List[Segment],
                                    target_segments: List[Segment],
                                    max_iterations: int = None) -> Optional[Tuple[List[GridState], Tuple[float, float], Tuple[float, float]]]:
        """
        Route from source segments to target segments.

        Samples points along source segments as starting points,
        samples points along target segments as goals.

        Returns (path, source_connection_mm, target_connection_mm) or None.
        """
        if max_iterations is None:
            max_iterations = self.config.max_iterations

        # Sample source segments to get starting points
        start_points = []  # List of (GridState, original_float_coords)
        for seg in source_segments:
            layer_idx = self.layer_mapper.get_idx(seg.layer)
            if layer_idx is None:
                continue

            dx = seg.end_x - seg.start_x
            dy = seg.end_y - seg.start_y
            length = math.sqrt(dx*dx + dy*dy)
            if length < 0.001:
                gx, gy = self.coord.to_grid(seg.start_x, seg.start_y)
                start_points.append((GridState(gx, gy, layer_idx),
                                     (seg.start_x, seg.start_y)))
                continue

            # Sample every grid step
            num_points = max(2, int(length / self.config.grid_step) + 1)
            for i in range(num_points):
                t = i / (num_points - 1)
                x = seg.start_x + t * dx
                y = seg.start_y + t * dy
                gx, gy = self.coord.to_grid(x, y)
                start_points.append((GridState(gx, gy, layer_idx), (x, y)))

        if not start_points:
            return None

        # Sample target segments to get goal points
        target_points = []  # List of (gx, gy, layer_idx, original_float_coords)
        for seg in target_segments:
            layer_idx = self.layer_mapper.get_idx(seg.layer)
            if layer_idx is None:
                continue

            dx = seg.end_x - seg.start_x
            dy = seg.end_y - seg.start_y
            length = math.sqrt(dx*dx + dy*dy)
            if length < 0.001:
                gx, gy = self.coord.to_grid(seg.start_x, seg.start_y)
                target_points.append(((gx, gy, layer_idx), (seg.start_x, seg.start_y)))
                continue

            num_points = max(2, int(length / self.config.grid_step) + 1)
            for i in range(num_points):
                t = i / (num_points - 1)
                x = seg.start_x + t * dx
                y = seg.start_y + t * dy
                gx, gy = self.coord.to_grid(x, y)
                target_points.append(((gx, gy, layer_idx), (x, y)))

        if not target_points:
            return None

        # Create lookup for target float coords
        target_float_coords = {pt[0]: pt[1] for pt in target_points}
        target_keys = [pt[0] for pt in target_points]
        target_set = set(target_keys)

        # Multi-source A* - initialize with all start points
        counter = 0
        open_set = []
        g_costs: Dict[Tuple[int, int, int], int] = {}
        parents: Dict[Tuple[int, int, int], Optional[Tuple[int, int, int]]] = {}
        source_origins: Dict[Tuple[int, int, int], Tuple[float, float]] = {}
        closed: Set[Tuple[int, int, int]] = set()

        for state, float_coords in start_points:
            key = state.as_tuple()
            h = int(self._heuristic_to_points(state, target_keys) * self.h_weight)
            heapq.heappush(open_set, (h, counter, 0, state))
            counter += 1
            g_costs[key] = 0
            parents[key] = None
            source_origins[key] = float_coords

        iterations = 0

        while open_set and iterations < max_iterations:
            iterations += 1

            f, _, g, current = heapq.heappop(open_set)
            current_key = current.as_tuple()

            if current_key in closed:
                continue

            closed.add(current_key)

            # Propagate source origin
            if current_key not in source_origins:
                parent_key = parents.get(current_key)
                if parent_key and parent_key in source_origins:
                    source_origins[current_key] = source_origins[parent_key]

            # Check if reached target
            if current_key in target_set:
                path = []
                key = current_key
                while key is not None:
                    path.append(GridState(*key))
                    key = parents[key]
                path.reverse()

                src_coords = source_origins.get(path[0].as_tuple(),
                                                 self.coord.to_float(path[0].gx, path[0].gy))
                tgt_coords = target_float_coords[current_key]

                print(f"Grid route found in {iterations} iterations, path length: {len(path)}")
                return (path, src_coords, tgt_coords)

            # Expand neighbors
            for dx, dy in self.DIRECTIONS:
                ngx = current.gx + dx
                ngy = current.gy + dy

                if self.obstacles.is_move_blocked(current.gx, current.gy,
                                                   ngx, ngy, current.layer_idx):
                    continue

                neighbor_key = (ngx, ngy, current.layer_idx)

                if neighbor_key in closed:
                    continue

                move_cost = self.DIAG_COST if (dx != 0 and dy != 0) else self.ORTHO_COST
                # Add stub proximity cost to discourage blocking unrouted nets
                move_cost += self.obstacles.get_stub_proximity_cost(ngx, ngy)
                new_g = g + move_cost

                if neighbor_key not in g_costs or new_g < g_costs[neighbor_key]:
                    g_costs[neighbor_key] = new_g
                    parents[neighbor_key] = current_key
                    if current_key in source_origins:
                        source_origins[neighbor_key] = source_origins[current_key]
                    neighbor = GridState(*neighbor_key)
                    h = int(self._heuristic_to_points(neighbor, target_keys) * self.h_weight)
                    f = new_g + h
                    counter += 1
                    heapq.heappush(open_set, (f, counter, new_g, neighbor))

            # Try via
            if not self.obstacles.is_via_blocked(current.gx, current.gy):
                for layer_idx in range(self.layer_mapper.num_layers()):
                    if layer_idx == current.layer_idx:
                        continue

                    neighbor_key = (current.gx, current.gy, layer_idx)

                    if neighbor_key in closed:
                        continue

                    # Via cost plus stub proximity cost (vias near stubs are extra bad)
                    via_proximity_cost = self.obstacles.get_stub_proximity_cost(current.gx, current.gy) * 2
                    new_g = g + self.via_cost + via_proximity_cost

                    if neighbor_key not in g_costs or new_g < g_costs[neighbor_key]:
                        g_costs[neighbor_key] = new_g
                        parents[neighbor_key] = current_key
                        if current_key in source_origins:
                            source_origins[neighbor_key] = source_origins[current_key]
                        neighbor = GridState(*neighbor_key)
                        h = int(self._heuristic_to_points(neighbor, target_keys) * self.h_weight)
                        f = new_g + h
                        counter += 1
                        heapq.heappush(open_set, (f, counter, new_g, neighbor))

        print(f"No grid route found after {iterations} iterations")
        return None

    def simplify_path(self, path: List[GridState]) -> List[GridState]:
        """
        Simplify path by removing intermediate points on straight lines.
        Keeps only waypoints where direction or layer changes.
        """
        if len(path) < 3:
            return path

        simplified = [path[0]]

        for i in range(1, len(path) - 1):
            prev = path[i - 1]
            curr = path[i]
            next_pt = path[i + 1]

            # Keep if layer changes
            if curr.layer_idx != prev.layer_idx or curr.layer_idx != next_pt.layer_idx:
                simplified.append(curr)
                continue

            # Keep if direction changes
            dx1 = curr.gx - prev.gx
            dy1 = curr.gy - prev.gy
            dx2 = next_pt.gx - curr.gx
            dy2 = next_pt.gy - curr.gy

            # Normalize direction
            if dx1 != 0:
                dx1 = dx1 // abs(dx1)
            if dy1 != 0:
                dy1 = dy1 // abs(dy1)
            if dx2 != 0:
                dx2 = dx2 // abs(dx2)
            if dy2 != 0:
                dy2 = dy2 // abs(dy2)

            if dx1 != dx2 or dy1 != dy2:
                simplified.append(curr)

        simplified.append(path[-1])
        return simplified

    def path_to_float(self, path: List[GridState]) -> List[Tuple[float, float, str]]:
        """
        Convert grid path to float coordinates for KiCad output.

        Returns list of (x_mm, y_mm, layer_name).
        """
        result = []
        for state in path:
            x, y = self.coord.to_float(state.gx, state.gy)
            layer = self.layer_mapper.get_layer(state.layer_idx)
            result.append((x, y, layer))
        return result


def create_grid_router(pcb_data: PCBData, config: GridRouteConfig,
                       exclude_net_id: Optional[int] = None) -> GridAStarRouter:
    """
    Factory function to create a grid-based router.

    Args:
        pcb_data: Parsed PCB data
        config: Routing configuration
        exclude_net_id: Net ID to exclude from obstacles (the net being routed)

    Returns:
        Configured GridAStarRouter instance
    """
    obstacles = GridObstacleMap(pcb_data, config, exclude_net_id)
    return GridAStarRouter(obstacles, config)
