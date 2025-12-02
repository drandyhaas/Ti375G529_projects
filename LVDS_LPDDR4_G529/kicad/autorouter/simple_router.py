#!/usr/bin/env python3
"""
Simple obstacle-avoiding router - FAST version.

Algorithm:
1. Start at source, heading toward sink
2. Take steps in the preferred direction
3. When blocked, turn to go around the obstacle
4. Resume heading toward sink once clear

Optimized with spatial indexing for speed.
"""

import math
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
import time


@dataclass
class Obstacle:
    """A rectangular obstacle with optional rotation and corner radius."""
    x: float  # Center X
    y: float  # Center Y
    width: float  # Full width (X dimension)
    height: float  # Full height (Y dimension)
    rotation: float = 0.0  # Rotation in degrees
    corner_radius: float = 0.0  # Corner radius for rounded rectangles

    def point_inside(self, px: float, py: float, clearance: float = 0.0) -> bool:
        """Check if a point is inside this obstacle (with clearance)."""
        # Translate point to obstacle's local coordinates
        dx = px - self.x
        dy = py - self.y

        # Rotate point to align with obstacle axes (if rotated)
        if self.rotation != 0:
            angle_rad = -math.radians(self.rotation)
            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)
            local_x = dx * cos_a - dy * sin_a
            local_y = dx * sin_a + dy * cos_a
        else:
            local_x = dx
            local_y = dy

        # Half dimensions with clearance
        half_w = self.width / 2 + clearance
        half_h = self.height / 2 + clearance

        # For rounded rectangles, check distance to rounded corners
        if self.corner_radius > 0:
            r = self.corner_radius
            # Inner rectangle dimensions (without corner regions)
            inner_half_w = half_w - r
            inner_half_h = half_h - r

            abs_x = abs(local_x)
            abs_y = abs(local_y)

            # Check if in corner region
            if abs_x > inner_half_w and abs_y > inner_half_h:
                # Distance to corner center
                corner_dx = abs_x - inner_half_w
                corner_dy = abs_y - inner_half_h
                dist_sq = corner_dx * corner_dx + corner_dy * corner_dy
                return dist_sq < r * r
            else:
                # In rectangular region
                return abs_x < half_w and abs_y < half_h
        else:
            # Simple rectangle check
            return abs(local_x) < half_w and abs(local_y) < half_h

    @property
    def radius(self) -> float:
        """Bounding radius for spatial indexing."""
        return math.sqrt((self.width/2)**2 + (self.height/2)**2)


class SpatialIndex:
    """Simple grid-based spatial index for fast obstacle lookup."""

    def __init__(self, obstacles: List[Obstacle], cell_size: float = 1.0):
        self.cell_size = cell_size
        self.grid: Dict[Tuple[int, int], List[Obstacle]] = {}

        for obs in obstacles:
            # Add obstacle to all cells it could affect
            min_cx = int((obs.x - obs.radius) / cell_size) - 1
            max_cx = int((obs.x + obs.radius) / cell_size) + 1
            min_cy = int((obs.y - obs.radius) / cell_size) - 1
            max_cy = int((obs.y + obs.radius) / cell_size) + 1

            for cx in range(min_cx, max_cx + 1):
                for cy in range(min_cy, max_cy + 1):
                    key = (cx, cy)
                    if key not in self.grid:
                        self.grid[key] = []
                    self.grid[key].append(obs)

    def get_nearby(self, x: float, y: float, radius: float) -> List[Obstacle]:
        """Get obstacles near a point."""
        cx = int(x / self.cell_size)
        cy = int(y / self.cell_size)

        nearby = []
        # Check 3x3 grid of cells around point
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                key = (cx + dx, cy + dy)
                if key in self.grid:
                    nearby.extend(self.grid[key])

        return nearby


def route_single_net(
    source: Tuple[float, float],
    sink: Tuple[float, float],
    obstacles: List[Obstacle],
    step_size: float = 0.2,
    clearance: float = 0.15,
    max_time: float = 5.0
) -> Optional[List[Tuple[float, float]]]:
    """
    Route from source to sink, avoiding obstacles.

    Returns list of (x, y) points forming the path, or None if failed.
    Must complete within max_time seconds.
    """
    start_time = time.time()

    # Build spatial index for fast obstacle lookup
    index = SpatialIndex(obstacles, cell_size=1.0)

    path = [source]
    x, y = source
    goal_x, goal_y = sink

    # Track visited positions to detect loops
    visited = set()

    step = 0
    while True:
        step += 1

        # Check timeout every 100 steps
        if step % 100 == 0:
            if time.time() - start_time > max_time:
                print(f"  Timeout after {step} steps at ({x:.2f}, {y:.2f})")
                return None

        # Check if we reached the goal (within 0.5mm or 2*step_size)
        dist_to_goal = math.sqrt((x - goal_x)**2 + (y - goal_y)**2)
        goal_threshold = max(0.5, step_size * 2)
        if dist_to_goal < goal_threshold:
            # Verify the final segment to sink is clear before appending
            if segment_is_clear_fast(x, y, goal_x, goal_y, index, clearance):
                path.append(sink)
                elapsed = time.time() - start_time
                print(f"  Reached goal in {step} steps, {elapsed:.2f}s")
                return path
            # Final segment blocked - continue routing to find clear approach

        # Direction toward goal - use only 45-degree angles
        dx = goal_x - x
        dy = goal_y - y
        best_dir = get_best_45_direction(dx, dy)

        # Try to step toward goal using 45-degree direction
        next_x = x + best_dir[0] * step_size
        next_y = y + best_dir[1] * step_size

        if segment_is_clear_fast(x, y, next_x, next_y, index, clearance):
            # Clear path - take the step
            x, y = next_x, next_y
            path.append((x, y))
            continue

        # Blocked! Try all 8 directions at 45-degree increments
        # Sort by dot product with goal direction (prefer directions closer to goal)
        length = math.sqrt(dx*dx + dy*dy)
        if length > 0.001:
            norm_dx, norm_dy = dx / length, dy / length
        else:
            norm_dx, norm_dy = 1.0, 0.0
        sorted_dirs = sorted(
            DIRECTIONS_45,
            key=lambda d: -(d[0] * norm_dx + d[1] * norm_dy)  # Descending by dot product
        )
        found = False
        for dir_x, dir_y in sorted_dirs:
            if (dir_x, dir_y) == best_dir:
                continue  # Already tried

            try_x = x + dir_x * step_size
            try_y = y + dir_y * step_size

            if segment_is_clear_fast(x, y, try_x, try_y, index, clearance):
                x, y = try_x, try_y
                path.append((x, y))
                found = True
                break

        if not found:
            print(f"  Stuck at ({x:.2f}, {y:.2f})")
            return None

        # Check for loops
        grid_pos = (round(x / step_size), round(y / step_size))
        if grid_pos in visited:
            pass  # Continue anyway
        visited.add(grid_pos)


def segment_is_clear_fast(
    x1: float, y1: float,
    x2: float, y2: float,
    index: SpatialIndex,
    clearance: float
) -> bool:
    """Segment clearance check - checks points along the segment at small intervals."""
    dx = x2 - x1
    dy = y2 - y1
    length = math.sqrt(dx*dx + dy*dy)

    # Check points at most every 0.1mm to catch small obstacles
    # Minimum of 5 points, scale up for longer segments
    num_checks = max(5, int(length / 0.1) + 1)

    for i in range(num_checks):
        t = i / (num_checks - 1) if num_checks > 1 else 0
        x = x1 + t * dx
        y = y1 + t * dy
        if is_blocked_indexed(x, y, index, clearance):
            return False
    return True


# 8 directions at 45-degree increments (0, 45, 90, 135, 180, 225, 270, 315)
DIRECTIONS_45 = [
    (1.0, 0.0),           # 0 deg
    (0.7071, 0.7071),     # 45 deg
    (0.0, 1.0),           # 90 deg
    (-0.7071, 0.7071),    # 135 deg
    (-1.0, 0.0),          # 180 deg
    (-0.7071, -0.7071),   # 225 deg
    (0.0, -1.0),          # 270 deg
    (0.7071, -0.7071),    # 315 deg
]


def get_best_45_direction(dx: float, dy: float) -> Tuple[float, float]:
    """Get the closest 45-degree direction to the given vector."""
    best_dir = DIRECTIONS_45[0]
    best_dot = -float('inf')

    # Normalize input
    length = math.sqrt(dx*dx + dy*dy)
    if length < 0.001:
        return (1.0, 0.0)
    dx /= length
    dy /= length

    for dir_x, dir_y in DIRECTIONS_45:
        dot = dx * dir_x + dy * dir_y
        if dot > best_dot:
            best_dot = dot
            best_dir = (dir_x, dir_y)

    return best_dir


def escape_from_pad(
    start: Tuple[float, float],
    target: Tuple[float, float],
    obstacles: List[Obstacle],
    index: SpatialIndex,
    step_size: float,
    clearance: float,
    max_steps: int = 50
) -> List[Tuple[float, float]]:
    """
    Escape from a dense area (like a BGA pad or QFP pad) by moving toward target.
    Returns a short path that gets out of the dense region.
    Only uses 45-degree angle increments.
    """
    path = [start]
    x, y = start
    target_x, target_y = target

    for step in range(max_steps):
        # Check obstacle density around current position
        nearby = index.get_nearby(x, y, 1.0)
        density = len(nearby)

        # If we've escaped the dense region (fewer than 5 nearby obstacles), stop
        if density < 5 and step > 0:  # Must have moved at least once
            break

        # Direction toward target - snap to 45 degrees
        dx = target_x - x
        dy = target_y - y
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 0.001:
            break

        # Get best 45-degree direction toward target
        best_dir = get_best_45_direction(dx, dy)

        # Try to step in that direction
        next_x = x + best_dir[0] * step_size
        next_y = y + best_dir[1] * step_size

        if segment_is_clear_fast(x, y, next_x, next_y, index, clearance):
            x, y = next_x, next_y
            path.append((x, y))
            continue

        # Blocked - try other 45-degree directions, ordered by preference
        # Sort by dot product with goal direction (prefer directions closer to target)
        length = math.sqrt(dx*dx + dy*dy)
        if length > 0.001:
            norm_dx, norm_dy = dx / length, dy / length
        else:
            norm_dx, norm_dy = 1.0, 0.0
        sorted_dirs = sorted(
            DIRECTIONS_45,
            key=lambda d: -(d[0] * norm_dx + d[1] * norm_dy)  # Descending by dot product
        )
        found = False
        for dir_x, dir_y in sorted_dirs:
            if (dir_x, dir_y) == best_dir:
                continue  # Already tried

            try_x = x + dir_x * step_size
            try_y = y + dir_y * step_size

            if segment_is_clear_fast(x, y, try_x, try_y, index, clearance):
                x, y = try_x, try_y
                path.append((x, y))
                found = True
                break

        if not found:
            # Last resort: find the 45-degree direction with best clearance
            best_clear_dir = None
            best_clearance = 0
            for dir_x, dir_y in DIRECTIONS_45:
                try_x = x + dir_x * step_size
                try_y = y + dir_y * step_size

                # Calculate minimum clearance at the new point
                min_clear = float('inf')
                for obs in index.get_nearby(try_x, try_y, clearance + step_size):
                    d = math.sqrt((try_x - obs.x)**2 + (try_y - obs.y)**2) - obs.radius
                    min_clear = min(min_clear, d)

                if min_clear > best_clearance:
                    best_clearance = min_clear
                    best_clear_dir = (dir_x, dir_y)

            if best_clear_dir and best_clearance > 0:
                x = x + best_clear_dir[0] * step_size
                y = y + best_clear_dir[1] * step_size
                path.append((x, y))
            else:
                break  # Truly stuck

    return path


def route_bidirectional(
    source: Tuple[float, float],
    sink: Tuple[float, float],
    obstacles: List[Obstacle],
    step_size: float = 0.2,
    clearance: float = 0.15,
    max_time: float = 5.0
) -> Optional[List[Tuple[float, float]]]:
    """
    Route from source to sink using bidirectional approach:
    1. Escape from source (dense area like BGA)
    2. Escape from sink (dense area like QFP)
    3. Connect the escape points in the middle

    Returns list of (x, y) points forming the path, or None if failed.
    """
    start_time = time.time()

    # Build spatial index
    index = SpatialIndex(obstacles, cell_size=1.0)

    # Escape from source toward sink
    print(f"  Escaping from source...")
    source_escape = escape_from_pad(source, sink, obstacles, index, step_size, clearance)
    source_end = source_escape[-1]
    print(f"    Escaped {len(source_escape)} steps to ({source_end[0]:.2f}, {source_end[1]:.2f})")

    # Escape from sink toward source
    print(f"  Escaping from sink...")
    sink_escape = escape_from_pad(sink, source, obstacles, index, step_size, clearance)
    sink_end = sink_escape[-1]
    print(f"    Escaped {len(sink_escape)} steps to ({sink_end[0]:.2f}, {sink_end[1]:.2f})")

    # Check if escapes already meet
    dist_between = math.sqrt((source_end[0] - sink_end[0])**2 + (source_end[1] - sink_end[1])**2)
    if dist_between < step_size * 2:
        # Direct connection
        print(f"  Escapes meet directly!")
        full_path = source_escape + list(reversed(sink_escape))
        return full_path

    # Route between the two escape points
    remaining_time = max_time - (time.time() - start_time)
    if remaining_time < 0.5:
        remaining_time = 0.5

    print(f"  Connecting escape points (dist={dist_between:.2f}mm)...")
    middle_path = route_single_net(
        source=source_end,
        sink=sink_end,
        obstacles=obstacles,
        step_size=step_size,
        clearance=clearance,
        max_time=remaining_time
    )

    if middle_path is None:
        print(f"  Failed to connect escape points, trying direct route...")
        # Fall back to direct routing
        return route_single_net(source, sink, obstacles, step_size, clearance, max_time)

    # Combine paths: source_escape + middle (excluding duplicate endpoints) + reversed sink_escape
    full_path = source_escape[:-1] + middle_path[:-1] + list(reversed(sink_escape))

    print(f"  Combined path has {len(full_path)} points")

    # Validate combined path has no collisions
    has_collision = False
    for i in range(1, len(full_path)):
        x1, y1 = full_path[i-1]
        x2, y2 = full_path[i]
        if not segment_is_clear_fast(x1, y1, x2, y2, index, clearance):
            print(f"  WARNING: Collision in combined path at segment {i-1}->{i}")
            has_collision = True
            break

    if has_collision:
        print(f"  Falling back to direct route due to collision...")
        return route_single_net(source, sink, obstacles, step_size, clearance, max_time)

    return full_path


def is_blocked_indexed(x: float, y: float, index: SpatialIndex, clearance: float) -> bool:
    """Check if position (x, y) is blocked by any obstacle using spatial index."""
    nearby = index.get_nearby(x, y, clearance)
    for obs in nearby:
        if obs.point_inside(x, y, clearance):
            return True
    return False


def find_blocking_obstacle_indexed(
    x: float, y: float,
    next_x: float, next_y: float,
    index: SpatialIndex,
    clearance: float
) -> Optional[Obstacle]:
    """Find the obstacle blocking the path using spatial index."""
    nearby = index.get_nearby(next_x, next_y, clearance)
    closest = None
    closest_dist = float('inf')

    for obs in nearby:
        if obs.point_inside(next_x, next_y, clearance):
            dist_sq = (next_x - obs.x)**2 + (next_y - obs.y)**2
            if dist_sq < closest_dist:
                closest_dist = dist_sq
                closest = obs

    return closest


def segment_is_clear(
    x1: float, y1: float,
    x2: float, y2: float,
    index: SpatialIndex,
    clearance: float,
    num_checks: int = None
) -> bool:
    """Check if a line segment is clear of obstacles.

    If num_checks is not specified, automatically calculates based on segment length
    to ensure checking every 0.1mm.
    """
    dx = x2 - x1
    dy = y2 - y1
    length = math.sqrt(dx*dx + dy*dy)

    if num_checks is None:
        # Check points at most every 0.1mm to catch small obstacles
        num_checks = max(5, int(length / 0.1) + 1)

    for i in range(num_checks + 1):
        t = i / num_checks if num_checks > 0 else 0
        x = x1 + t * dx
        y = y1 + t * dy
        if is_blocked_indexed(x, y, index, clearance):
            return False
    return True


def path_cost(
    path: List[Tuple[float, float]],
    index: SpatialIndex,
    obstacles: List[Obstacle],
    clearance: float,
    length_weight: float = 1.0,
    clearance_weight: float = 5.0,
    proximity_radius: float = 2.0
) -> float:
    """
    Calculate cost of a path. Lower is better.
    Returns infinity if path has collisions (except near endpoints).

    Cost considers:
    - Path length (minimize)
    - Sum of proximity penalties for ALL nearby obstacles (minimize)
      Penalty = 1/distance for each nearby obstacle
    """
    if len(path) < 2:
        return float('inf')

    # Check for collisions along ALL segments - no exceptions
    for i in range(1, len(path)):
        x1, y1 = path[i-1]
        x2, y2 = path[i]
        seg_len = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        num_checks = max(20, int(seg_len / 0.1))
        if not segment_is_clear(x1, y1, x2, y2, index, clearance, num_checks=num_checks):
            return float('inf')

    # Calculate total length
    total_length = 0.0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i-1][0]
        dy = path[i][1] - path[i-1][1]
        total_length += math.sqrt(dx*dx + dy*dy)

    # Calculate proximity penalty for all nearby obstacles
    # Higher penalty when closer to ANY obstacle
    total_proximity_penalty = 0.0

    # Check at path points (skip endpoints)
    for i, pt in enumerate(path):
        if i == 0 or i == len(path) - 1:
            continue  # Skip endpoints
        nearby = index.get_nearby(pt[0], pt[1], proximity_radius)
        for obs in nearby:
            dist = math.sqrt((pt[0] - obs.x)**2 + (pt[1] - obs.y)**2)
            effective_dist = dist - obs.radius
            if effective_dist > 0.01:
                # Inverse distance penalty - closer = higher penalty
                total_proximity_penalty += 1.0 / effective_dist

    # Also check along segments (sample points)
    for i in range(1, len(path) - 1):  # Skip first and last segments
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        for t in [0.25, 0.5, 0.75]:
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            nearby = index.get_nearby(x, y, proximity_radius)
            for obs in nearby:
                dist = math.sqrt((x - obs.x)**2 + (y - obs.y)**2)
                effective_dist = dist - obs.radius
                if effective_dist > 0.01:
                    total_proximity_penalty += 1.0 / effective_dist

    # Cost: length + proximity penalty (weighted)
    # Lower is better, so we ADD the proximity penalty
    cost = length_weight * total_length + clearance_weight * total_proximity_penalty
    return cost


def get_segment_direction(x1: float, y1: float, x2: float, y2: float) -> Tuple[float, float]:
    """Get normalized direction vector for a segment."""
    dx = x2 - x1
    dy = y2 - y1
    length = math.sqrt(dx*dx + dy*dy)
    if length < 0.0001:
        return (0, 0)
    return (dx / length, dy / length)


def fast_path_cost(
    path: List[Tuple[float, float]],
    index: SpatialIndex,
    clearance: float,
    length_weight: float = 1.0,
    clearance_weight: float = 5.0
) -> float:
    """
    Fast path cost calculation for optimization.
    Checks collisions and calculates a simple length + proximity penalty.
    """
    if len(path) < 2:
        return float('inf')

    # Quick collision check - just check segment endpoints and midpoints
    for i in range(1, len(path)):
        x1, y1 = path[i-1]
        x2, y2 = path[i]

        # Check endpoints
        if is_blocked_indexed(x1, y1, index, clearance):
            # Allow endpoints to be in obstacles (they're on pads)
            if i > 1:
                return float('inf')
        if is_blocked_indexed(x2, y2, index, clearance):
            if i < len(path) - 1:
                return float('inf')

        # Check midpoint
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        if is_blocked_indexed(mx, my, index, clearance):
            return float('inf')

        # Check points along segment - more points for longer segments
        dx = x2 - x1
        dy = y2 - y1
        seg_len = math.sqrt(dx*dx + dy*dy)
        # Check every 0.5mm for longer segments to catch all obstacles
        num_checks = max(2, int(seg_len / 0.5))
        for j in range(1, num_checks):
            t = j / num_checks
            px = x1 + t * dx
            py = y1 + t * dy
            if is_blocked_indexed(px, py, index, clearance):
                return float('inf')

    # Calculate total length
    total_length = 0.0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i-1][0]
        dy = path[i][1] - path[i-1][1]
        total_length += math.sqrt(dx*dx + dy*dy)

    # Simple proximity penalty - check only corner points
    proximity_penalty = 0.0
    for i in range(1, len(path) - 1):
        x, y = path[i]
        nearby = index.get_nearby(x, y, 1.0)
        for obs in nearby:
            dist = math.sqrt((x - obs.x)**2 + (y - obs.y)**2)
            effective_dist = dist - obs.radius
            if effective_dist > 0.01:
                proximity_penalty += 1.0 / effective_dist

    return length_weight * total_length + clearance_weight * proximity_penalty


def get_45_direction(dx: float, dy: float) -> Tuple[float, float]:
    """Snap a direction vector to the nearest 45-degree direction."""
    if abs(dx) < 0.0001 and abs(dy) < 0.0001:
        return (0, 0)

    # Find closest 45-degree direction
    best_dir = DIRECTIONS_45[0]
    best_dot = -2
    length = math.sqrt(dx*dx + dy*dy)
    ndx, ndy = dx/length, dy/length

    for dir_x, dir_y in DIRECTIONS_45:
        dot = ndx * dir_x + ndy * dir_y
        if dot > best_dot:
            best_dot = dot
            best_dir = (dir_x, dir_y)

    return best_dir


def is_exact_45_segment(x1: float, y1: float, x2: float, y2: float, tolerance: float = 0.002) -> bool:
    """Check if segment is at an exact 45-degree multiple angle.

    Args:
        tolerance: Maximum allowed deviation from exact 45-degree. Default 0.002mm (2 microns).
    """
    dx = x2 - x1
    dy = y2 - y1
    if abs(dx) < tolerance and abs(dy) < tolerance:
        return True  # Zero length is OK

    # Check horizontal, vertical, or diagonal
    is_horizontal = abs(dy) < tolerance
    is_vertical = abs(dx) < tolerance
    is_diagonal = abs(abs(dx) - abs(dy)) < tolerance

    return is_horizontal or is_vertical or is_diagonal


def optimize_path_45deg(
    path: List[Tuple[float, float]],
    obstacles: List[Obstacle],
    clearance: float,
    iterations: int = 20,
    step_size: float = 0.1,
    length_weight: float = 1.0,
    clearance_weight: float = 5.0
) -> List[Tuple[float, float]]:
    """
    Optimize a path while strictly maintaining 45-degree angle constraints.

    The optimization works in two ways:
    1. Sliding individual corner points along segment directions
    2. Sliding entire segments perpendicular to their direction (moves two corners together)
    """
    if len(path) < 3:
        return path

    # Build spatial index
    index = SpatialIndex(obstacles, cell_size=1.0)

    best_path = [pt for pt in path]
    best_cost = fast_path_cost(best_path, index, clearance, length_weight, clearance_weight)

    if best_cost == float('inf'):
        return path

    initial_cost = best_cost
    current_step = step_size

    for iteration in range(iterations):
        improved = False

        # === Strategy 1: Slide individual corner points along segment directions ===
        for i in range(1, len(best_path) - 1):
            prev = best_path[i - 1]
            curr = best_path[i]
            next_pt = best_path[i + 1]

            # Get incoming and outgoing segment directions (must be at 45-degree already)
            d1 = get_45_direction(curr[0] - prev[0], curr[1] - prev[1])
            d2 = get_45_direction(next_pt[0] - curr[0], next_pt[1] - curr[1])

            if d1 == (0, 0) or d2 == (0, 0):
                continue

            # Only move along the segment directions to maintain angles
            move_directions = [d1, (-d1[0], -d1[1]), d2, (-d2[0], -d2[1])]

            for move_dx, move_dy in move_directions:
                new_x = curr[0] + move_dx * current_step
                new_y = curr[1] + move_dy * current_step

                candidate = list(best_path)
                candidate[i] = (new_x, new_y)

                if not is_exact_45_segment(prev[0], prev[1], new_x, new_y):
                    continue
                if not is_exact_45_segment(new_x, new_y, next_pt[0], next_pt[1]):
                    continue

                candidate_cost = fast_path_cost(candidate, index, clearance,
                                               length_weight, clearance_weight)

                if candidate_cost < best_cost:
                    best_path = candidate
                    best_cost = candidate_cost
                    improved = True

        # === Strategy 2: Slide horizontal/vertical segments perpendicular to their direction ===
        # When sliding a H/V segment, we adjust its endpoints to maintain 45-deg with neighbors
        for i in range(1, len(best_path) - 1):
            if i + 1 >= len(best_path) - 1:
                continue

            p0 = best_path[i - 1]
            p1 = best_path[i]
            p2 = best_path[i + 1]
            if i + 2 < len(best_path):
                p3 = best_path[i + 2]
            else:
                continue

            seg_dx = p2[0] - p1[0]
            seg_dy = p2[1] - p1[1]

            # Only apply to horizontal or vertical segments
            is_horizontal = abs(seg_dy) < 0.001 and abs(seg_dx) > 0.001
            is_vertical = abs(seg_dx) < 0.001 and abs(seg_dy) > 0.001

            if not (is_horizontal or is_vertical):
                continue

            # Get neighbor segment directions
            d_in = get_45_direction(p1[0] - p0[0], p1[1] - p0[1])
            d_out = get_45_direction(p3[0] - p2[0], p3[1] - p2[1])

            if d_in == (0, 0) or d_out == (0, 0):
                continue

            # For a horizontal segment:
            #   - Slide up: move p1 by (+step if d_in going right-up, -step if going left-up, etc)
            #   - The key is: to maintain 45-deg with p0, if we move p1 up by delta_y,
            #     we need to move p1's x by delta_y * (d_in[0]/d_in[1]) IF d_in[1] != 0
            # For vertical: similar logic with x/y swapped

            if is_horizontal:
                # Check incoming/outgoing can slide
                if abs(d_in[1]) <= 0.5 or abs(d_out[1]) <= 0.5:
                    continue  # Can't slide with vertical neighbors

                # Slide up (+y) or down (-y)
                for slide_dir in [1, -1]:
                    delta_y = slide_dir * current_step

                    # For 45-degree diagonals, delta_x = ±delta_y
                    # d_in is normalized, so d_in[0]/d_in[1] is either +1 or -1 for diagonals
                    delta_x1 = delta_y * (1 if d_in[0] * d_in[1] > 0 else -1)
                    delta_x2 = delta_y * (1 if d_out[0] * d_out[1] > 0 else -1)

                    new_p1 = (p1[0] + delta_x1, p1[1] + delta_y)
                    new_p2 = (p2[0] + delta_x2, p2[1] + delta_y)

                    # Round to avoid floating point drift (3 decimal places = 0.001mm precision)
                    new_p1 = (round(new_p1[0], 3), round(new_p1[1], 3))
                    new_p2 = (round(new_p2[0], 3), round(new_p2[1], 3))

                    # Verify 45-degree constraints
                    if not is_exact_45_segment(p0[0], p0[1], new_p1[0], new_p1[1]):
                        continue
                    if not is_exact_45_segment(new_p1[0], new_p1[1], new_p2[0], new_p2[1]):
                        continue
                    if not is_exact_45_segment(new_p2[0], new_p2[1], p3[0], p3[1]):
                        continue

                    candidate = list(best_path)
                    candidate[i] = new_p1
                    candidate[i + 1] = new_p2

                    candidate_cost = fast_path_cost(candidate, index, clearance,
                                                   length_weight, clearance_weight)

                    if candidate_cost < best_cost:
                        best_path = candidate
                        best_cost = candidate_cost
                        improved = True

            elif is_vertical:
                # Check incoming/outgoing can slide
                if abs(d_in[0]) <= 0.5 or abs(d_out[0]) <= 0.5:
                    continue  # Can't slide with horizontal neighbors

                # Slide left (-x) or right (+x)
                for slide_dir in [1, -1]:
                    delta_x = slide_dir * current_step

                    # For 45-degree diagonals, delta_y = ±delta_x
                    delta_y1 = delta_x * (1 if d_in[0] * d_in[1] > 0 else -1)
                    delta_y2 = delta_x * (1 if d_out[0] * d_out[1] > 0 else -1)

                    new_p1 = (p1[0] + delta_x, p1[1] + delta_y1)
                    new_p2 = (p2[0] + delta_x, p2[1] + delta_y2)

                    # Round to avoid floating point drift
                    new_p1 = (round(new_p1[0], 3), round(new_p1[1], 3))
                    new_p2 = (round(new_p2[0], 3), round(new_p2[1], 3))

                    if not is_exact_45_segment(p0[0], p0[1], new_p1[0], new_p1[1]):
                        continue
                    if not is_exact_45_segment(new_p1[0], new_p1[1], new_p2[0], new_p2[1]):
                        continue
                    if not is_exact_45_segment(new_p2[0], new_p2[1], p3[0], p3[1]):
                        continue

                    candidate = list(best_path)
                    candidate[i] = new_p1
                    candidate[i + 1] = new_p2

                    candidate_cost = fast_path_cost(candidate, index, clearance,
                                                   length_weight, clearance_weight)

                    if candidate_cost < best_cost:
                        best_path = candidate
                        best_cost = candidate_cost
                        improved = True

        if not improved:
            current_step *= 0.7
            if current_step < 0.02:
                break

    # Final pass: Try to center horizontal segments between nearby obstacles
    for i in range(1, len(best_path) - 1):
        if i + 1 >= len(best_path) - 1:
            continue

        p0 = best_path[i - 1]
        p1 = best_path[i]
        p2 = best_path[i + 1]
        if i + 2 < len(best_path):
            p3 = best_path[i + 2]
        else:
            continue

        seg_dx = p2[0] - p1[0]
        seg_dy = p2[1] - p1[1]

        is_horizontal = abs(seg_dy) < 0.001 and abs(seg_dx) > 0.001

        if not is_horizontal:
            continue

        d_in = get_45_direction(p1[0] - p0[0], p1[1] - p0[1])
        d_out = get_45_direction(p3[0] - p2[0], p3[1] - p2[1])

        if abs(d_in[1]) <= 0.5 or abs(d_out[1]) <= 0.5:
            continue

        # Find obstacles above and below the segment
        seg_y = p1[1]
        seg_x_min = min(p1[0], p2[0])
        seg_x_max = max(p1[0], p2[0])

        obs_above_y = None
        obs_below_y = None

        for obs in index.get_nearby((p1[0] + p2[0])/2, seg_y, 2.0):
            # Determine obstacle dimensions accounting for rotation
            # For 90-degree rotation, width and height swap
            rot_mod = obs.rotation % 180
            if abs(rot_mod - 90) < 1:  # 90 or 270 degrees
                eff_width = obs.height
                eff_height = obs.width
            else:
                eff_width = obs.width
                eff_height = obs.height

            # Check if obstacle is in the x range of the segment
            if obs.x < seg_x_min - eff_width/2 - 0.1 or obs.x > seg_x_max + eff_width/2 + 0.1:
                continue

            # Determine obstacle edge y positions
            half_h = eff_height / 2
            obs_top = obs.y - half_h  # smaller y = "above" in value
            obs_bottom = obs.y + half_h  # larger y = "below" in value

            if obs_bottom < seg_y and (obs_above_y is None or obs_bottom > obs_above_y):
                obs_above_y = obs_bottom
            if obs_top > seg_y and (obs_below_y is None or obs_top < obs_below_y):
                obs_below_y = obs_top

        if obs_above_y is not None and obs_below_y is not None:
            # Calculate optimal centered y position
            optimal_y = (obs_above_y + obs_below_y) / 2
            delta_y = optimal_y - seg_y

            if abs(delta_y) > 0.001:  # Only if not already centered
                delta_x1 = delta_y * (1 if d_in[0] * d_in[1] > 0 else -1)
                delta_x2 = delta_y * (1 if d_out[0] * d_out[1] > 0 else -1)

                new_p1 = (round(p1[0] + delta_x1, 3), round(p1[1] + delta_y, 3))
                new_p2 = (round(p2[0] + delta_x2, 3), round(p2[1] + delta_y, 3))

                # Verify constraints
                if (is_exact_45_segment(p0[0], p0[1], new_p1[0], new_p1[1]) and
                    is_exact_45_segment(new_p1[0], new_p1[1], new_p2[0], new_p2[1]) and
                    is_exact_45_segment(new_p2[0], new_p2[1], p3[0], p3[1])):

                    # Check collision
                    candidate = list(best_path)
                    candidate[i] = new_p1
                    candidate[i + 1] = new_p2

                    candidate_cost = fast_path_cost(candidate, index, clearance,
                                                   length_weight, clearance_weight)

                    if candidate_cost < float('inf'):
                        best_path = candidate
                        print(f"    Centered segment at y={optimal_y:.3f}")

    if best_cost < initial_cost:
        improvement = (initial_cost - best_cost) / initial_cost * 100
        print(f"    Optimized: cost {initial_cost:.2f} -> {best_cost:.2f} ({improvement:.1f}% improvement)")

    return best_path


def remove_kinks(
    path: List[Tuple[float, float]],
    obstacles: List[Obstacle],
    clearance: float
) -> List[Tuple[float, float]]:
    """
    Remove unnecessary kinks/zigzags from the path.
    Tries multiple strategies:
    1. Simple point skipping if result is still 45-degree
    2. Replacing multiple segments with a diagonal+H/V or H/V+diagonal pair
    """
    if len(path) < 3:
        return path

    index = SpatialIndex(obstacles, cell_size=1.0)
    result = list(path)
    changed = True

    while changed:
        changed = False
        i = 1
        while i < len(result) - 1:
            p_prev = result[i - 1]
            p_curr = result[i]
            p_next = result[i + 1]

            # Strategy 1: Simple point skip if result is 45-degree
            if is_exact_45_segment(p_prev[0], p_prev[1], p_next[0], p_next[1]):
                if segment_is_clear(p_prev[0], p_prev[1], p_next[0], p_next[1], index, clearance):
                    old_len = (math.sqrt((p_curr[0]-p_prev[0])**2 + (p_curr[1]-p_prev[1])**2) +
                              math.sqrt((p_next[0]-p_curr[0])**2 + (p_next[1]-p_curr[1])**2))
                    new_len = math.sqrt((p_next[0]-p_prev[0])**2 + (p_next[1]-p_prev[1])**2)
                    if new_len <= old_len * 1.01:
                        result.pop(i)
                        changed = True
                        continue

            # Strategy 2: Try to replace 3 segments with 2 (diagonal + H/V or H/V + diagonal)
            if i < len(result) - 2:
                p_far = result[i + 2]

                # Try diagonal-first: p_prev -> corner -> p_far
                # Corner at 45-degree from p_prev, aligned with p_far
                dx = p_far[0] - p_prev[0]
                dy = p_far[1] - p_prev[1]

                # Option A: diagonal then horizontal
                # Go diagonally until same y as p_far, then horizontal
                diag_dy = dy
                diag_dx = diag_dy if diag_dy != 0 else 0  # 45-degree diagonal has |dx| = |dy|
                if diag_dx != 0:
                    diag_dx = diag_dx if (dx > 0) == (diag_dx > 0) else -diag_dx
                corner_a = (p_prev[0] + diag_dx, p_prev[1] + diag_dy)
                if (is_exact_45_segment(p_prev[0], p_prev[1], corner_a[0], corner_a[1]) and
                    is_exact_45_segment(corner_a[0], corner_a[1], p_far[0], p_far[1])):
                    if (segment_is_clear(p_prev[0], p_prev[1], corner_a[0], corner_a[1], index, clearance) and
                        segment_is_clear(corner_a[0], corner_a[1], p_far[0], p_far[1], index, clearance)):
                        # Check if shorter
                        old_len = sum(math.sqrt((result[j+1][0]-result[j][0])**2 + (result[j+1][1]-result[j][1])**2)
                                     for j in range(i-1, i+2))
                        new_len = (math.sqrt((corner_a[0]-p_prev[0])**2 + (corner_a[1]-p_prev[1])**2) +
                                  math.sqrt((p_far[0]-corner_a[0])**2 + (p_far[1]-corner_a[1])**2))
                        if new_len <= old_len * 1.01:
                            result[i] = corner_a
                            result.pop(i + 1)
                            changed = True
                            continue

                # Option B: horizontal then diagonal
                # Go horizontally until diagonal path to p_far
                horiz_dx = dx - dy if abs(dx) > abs(dy) else dx + dy if dx * dy < 0 else dx - dy
                corner_b = (p_prev[0] + horiz_dx, p_prev[1])
                if (is_exact_45_segment(p_prev[0], p_prev[1], corner_b[0], corner_b[1]) and
                    is_exact_45_segment(corner_b[0], corner_b[1], p_far[0], p_far[1])):
                    if (segment_is_clear(p_prev[0], p_prev[1], corner_b[0], corner_b[1], index, clearance) and
                        segment_is_clear(corner_b[0], corner_b[1], p_far[0], p_far[1], index, clearance)):
                        old_len = sum(math.sqrt((result[j+1][0]-result[j][0])**2 + (result[j+1][1]-result[j][1])**2)
                                     for j in range(i-1, i+2))
                        new_len = (math.sqrt((corner_b[0]-p_prev[0])**2 + (corner_b[1]-p_prev[1])**2) +
                                  math.sqrt((p_far[0]-corner_b[0])**2 + (p_far[1]-corner_b[1])**2))
                        if new_len <= old_len * 1.01:
                            result[i] = corner_b
                            result.pop(i + 1)
                            changed = True
                            continue

            i += 1

    return result


def optimize_simplified_path(
    path: List[Tuple[float, float]],
    obstacles: List[Obstacle],
    clearance: float,
    iterations: int = 50,
    initial_step: float = 0.1,
    min_step: float = 0.01,
    length_weight: float = 1.0,
    clearance_weight: float = 10.0
) -> List[Tuple[float, float]]:
    """
    Optimize path by adjusting corner positions while maintaining 45-degree constraints.
    Wrapper that calls optimize_path_45deg.
    """
    return optimize_path_45deg(path, obstacles, clearance, iterations, initial_step,
                               length_weight, clearance_weight)


def snap_to_45_degrees(
    path: List[Tuple[float, float]],
    obstacles: List[Obstacle],
    clearance: float,
    tolerance: float = 0.3
) -> List[Tuple[float, float]]:
    """
    Snap path segments to 45-degree angles while preserving connectivity.
    Only snaps if the resulting segments don't collide with obstacles.
    """
    if len(path) < 2:
        return path

    # Build spatial index for collision checking
    index = SpatialIndex(obstacles, cell_size=1.0)

    result = [path[0]]

    for i in range(1, len(path)):
        prev = result[-1]
        curr = path[i]

        dx = curr[0] - prev[0]
        dy = curr[1] - prev[1]

        if abs(dx) < 0.01 and abs(dy) < 0.01:
            continue

        # Try to create 45-degree segments
        # Strategy: horizontal/vertical first, then 45° diagonal

        if abs(dx) > abs(dy):
            # More horizontal - try horizontal then diagonal
            h_len = abs(dx) - abs(dy)
            if h_len > tolerance:
                mid_x = prev[0] + math.copysign(h_len, dx)
                mid_y = prev[1]
                # Check if this path is clear
                if (segment_is_clear(prev[0], prev[1], mid_x, mid_y, index, clearance) and
                    segment_is_clear(mid_x, mid_y, curr[0], curr[1], index, clearance)):
                    result.append((mid_x, mid_y))
                    result.append(curr)
                    continue

        else:
            # More vertical - try vertical then diagonal
            v_len = abs(dy) - abs(dx)
            if v_len > tolerance:
                mid_x = prev[0]
                mid_y = prev[1] + math.copysign(v_len, dy)
                # Check if this path is clear
                if (segment_is_clear(prev[0], prev[1], mid_x, mid_y, index, clearance) and
                    segment_is_clear(mid_x, mid_y, curr[0], curr[1], index, clearance)):
                    result.append((mid_x, mid_y))
                    result.append(curr)
                    continue

        # If snapping failed or wasn't needed, just use original point
        # But first verify the direct segment is clear
        if segment_is_clear(prev[0], prev[1], curr[0], curr[1], index, clearance):
            result.append(curr)
        else:
            # Segment is blocked - this shouldn't happen if the original path was valid
            # Fall back to original point
            result.append(curr)

    return result


def verify_path(
    path: List[Tuple[float, float]],
    obstacles: List[Obstacle],
    clearance: float,
    skip_endpoints: bool = True
) -> bool:
    """
    Verify that a path is collision-free.

    Args:
        path: List of (x, y) points
        obstacles: List of obstacles
        clearance: Minimum clearance from obstacles
        skip_endpoints: If True, don't check the very first and last segments
                       (they connect to pads which are excluded from obstacles)
    """
    if len(path) < 2:
        return True

    index = SpatialIndex(obstacles, cell_size=1.0)

    start_idx = 1 if skip_endpoints and len(path) > 2 else 0
    end_idx = len(path) - 1 if skip_endpoints and len(path) > 2 else len(path)

    for i in range(start_idx + 1, end_idx):
        if not segment_is_clear(path[i-1][0], path[i-1][1],
                                path[i][0], path[i][1],
                                index, clearance, num_checks=20):
            return False
    return True


def simplify_path(path: List[Tuple[float, float]], tolerance: float = 0.05) -> List[Tuple[float, float]]:
    """
    Simplify path by removing redundant points.
    Uses Douglas-Peucker-like simplification.
    """
    if len(path) <= 2:
        return path

    first = path[0]
    last = path[-1]

    max_dist = 0
    max_idx = 0

    for i in range(1, len(path) - 1):
        dist = point_to_line_distance(path[i], first, last)
        if dist > max_dist:
            max_dist = dist
            max_idx = i

    if max_dist > tolerance:
        left = simplify_path(path[:max_idx + 1], tolerance)
        right = simplify_path(path[max_idx:], tolerance)
        return left[:-1] + right
    else:
        return [first, last]


def is_45_degree_segment(x1: float, y1: float, x2: float, y2: float, tolerance: float = 0.01) -> bool:
    """Check if a segment is at a 45-degree angle (0, 45, 90, 135, etc.)."""
    dx = x2 - x1
    dy = y2 - y1
    length = math.sqrt(dx*dx + dy*dy)
    if length < 0.001:
        return True  # Zero-length segment is fine

    # Normalize
    dx /= length
    dy /= length

    # Check against all 8 valid directions
    for dir_x, dir_y in DIRECTIONS_45:
        # Check if direction matches (dot product close to 1 or -1)
        dot = abs(dx * dir_x + dy * dir_y)
        if dot > 1 - tolerance:
            return True

    return False


def can_convert_to_45_degrees(
    prev: Tuple[float, float],
    target: Tuple[float, float],
    index: SpatialIndex,
    clearance: float
) -> bool:
    """
    Check if a segment from prev to target can be converted to 45-degree
    segments without collision. Returns True if either:
    1. The segment is already exactly at a 45-degree angle, OR
    2. We can insert a corner point to make two collision-free 45-degree segments
    """
    dx = target[0] - prev[0]
    dy = target[1] - prev[1]
    abs_dx = abs(dx)
    abs_dy = abs(dy)

    # Already at exact 45-degree angle?
    is_horizontal = abs_dy < 0.001
    is_vertical = abs_dx < 0.001
    is_diagonal = abs(abs_dx - abs_dy) < 0.001

    if is_horizontal or is_vertical or is_diagonal:
        return segment_is_clear_fast(prev[0], prev[1], target[0], target[1], index, clearance)

    # Need to check if we can insert a corner point
    sign_x = 1 if dx >= 0 else -1
    sign_y = 1 if dy >= 0 else -1

    corners_to_try = []
    if abs_dx > abs_dy:
        # Option A: Diagonal first, then horizontal
        diag_corner = (prev[0] + sign_x * abs_dy, prev[1] + sign_y * abs_dy)
        corners_to_try.append(diag_corner)
        # Option B: Horizontal first, then diagonal
        horiz_corner = (prev[0] + sign_x * (abs_dx - abs_dy), prev[1])
        corners_to_try.append(horiz_corner)
    else:
        # Option A: Diagonal first, then vertical
        diag_corner = (prev[0] + sign_x * abs_dx, prev[1] + sign_y * abs_dx)
        corners_to_try.append(diag_corner)
        # Option B: Vertical first, then diagonal
        vert_corner = (prev[0], prev[1] + sign_y * (abs_dy - abs_dx))
        corners_to_try.append(vert_corner)

    for corner in corners_to_try:
        seg1_clear = segment_is_clear_fast(prev[0], prev[1], corner[0], corner[1], index, clearance)
        seg2_clear = segment_is_clear_fast(corner[0], corner[1], target[0], target[1], index, clearance)
        if seg1_clear and seg2_clear:
            return True

    return False


def simplify_path_safe(
    path: List[Tuple[float, float]],
    obstacles: List[Obstacle],
    clearance: float,
    tolerance: float = 0.1
) -> List[Tuple[float, float]]:
    """
    Simplify path while ensuring:
    1. All segments remain collision-free
    2. All segments can be converted to 45-degree angles without collision
    Iteratively removes points that can be skipped without collision.
    """
    if len(path) <= 2:
        return path

    index = SpatialIndex(obstacles, cell_size=1.0)
    result = list(path)

    changed = True
    while changed:
        changed = False
        i = 1
        while i < len(result) - 1:
            # Try to remove point i by connecting i-1 directly to i+1
            prev = result[i - 1]
            next_pt = result[i + 1]

            # Check if the resulting segment can be converted to 45-degree without collision
            if not can_convert_to_45_degrees(prev, next_pt, index, clearance):
                i += 1
                continue

            # Also check that we're not deviating too much
            dist = point_to_line_distance(result[i], prev, next_pt)
            if dist < tolerance:
                # Safe to remove this point
                result.pop(i)
                changed = True
                continue
            i += 1

    return result


def point_to_line_distance(point: Tuple[float, float], line_start: Tuple[float, float], line_end: Tuple[float, float]) -> float:
    """Calculate perpendicular distance from point to line."""
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end

    dx = x2 - x1
    dy = y2 - y1

    if dx == 0 and dy == 0:
        return math.sqrt((px - x1)**2 + (py - y1)**2)

    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx*dx + dy*dy)))

    proj_x = x1 + t * dx
    proj_y = y1 + t * dy

    return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)


def snap_path_to_45_degrees(
    path: List[Tuple[float, float]],
    obstacles: List[Obstacle],
    clearance: float
) -> List[Tuple[float, float]]:
    """
    Snap all path points to ensure every segment is exactly at a 45-degree angle.
    Keeps endpoints fixed, adjusts intermediate points.
    """
    if len(path) <= 2:
        return path

    index = SpatialIndex(obstacles, cell_size=1.0)
    result = [path[0]]  # Keep first point fixed

    for i in range(1, len(path) - 1):
        prev = result[-1]
        target = path[i]
        next_target = path[i + 1]

        # Find the closest point to target that:
        # 1. Is reachable from prev via a 45-degree segment
        # 2. Can reach next_target via a 45-degree segment

        # Try snapping to exact 45-degree position from prev
        dx = target[0] - prev[0]
        dy = target[1] - prev[1]

        # Get the best 45-degree direction
        best_dir = get_best_45_direction(dx, dy)
        seg_len = math.sqrt(dx*dx + dy*dy)

        # Snap the point to be exactly on this direction
        snapped_x = prev[0] + best_dir[0] * seg_len
        snapped_y = prev[1] + best_dir[1] * seg_len

        # Check if snapped segment is clear
        if segment_is_clear_fast(prev[0], prev[1], snapped_x, snapped_y, index, clearance):
            result.append((snapped_x, snapped_y))
        else:
            # Keep original point
            result.append(target)

    # Add the last point
    result.append(path[-1])

    return result


def enforce_45_degree_path(
    path: List[Tuple[float, float]],
    obstacles: List[Obstacle],
    clearance: float
) -> List[Tuple[float, float]]:
    """
    Convert a path to use only 45-degree angles by inserting corner points.
    Each segment becomes at most two segments: horizontal/vertical + diagonal or vice versa.

    The key insight: to go from (x1,y1) to (x2,y2) using only 45-degree segments:
    - If |dx| > |dy|: we can go diagonally for |dy| distance, then horizontally for |dx|-|dy|
    - If |dy| > |dx|: we can go diagonally for |dx| distance, then vertically for |dy|-|dx|
    - If |dx| == |dy|: already a 45-degree diagonal
    """
    if len(path) <= 1:
        return path

    index = SpatialIndex(obstacles, cell_size=1.0)
    result = [path[0]]

    for i in range(1, len(path)):
        prev = result[-1]
        target = path[i]

        dx = target[0] - prev[0]
        dy = target[1] - prev[1]
        abs_dx = abs(dx)
        abs_dy = abs(dy)

        # Skip tiny movements
        if abs_dx < 0.001 and abs_dy < 0.001:
            continue

        sign_x = 1 if dx >= 0 else -1
        sign_y = 1 if dy >= 0 else -1

        # Check if already at exact 45-degree angle (horizontal, vertical, or diagonal)
        is_horizontal = abs_dy < 0.001
        is_vertical = abs_dx < 0.001
        is_diagonal = abs(abs_dx - abs_dy) < 0.001

        if is_horizontal or is_vertical or is_diagonal:
            result.append(target)
            continue

        # Need to insert a corner point to make two 45-degree segments
        # Strategy: one diagonal + one axis-aligned segment

        corners_to_try = []

        if abs_dx > abs_dy:
            # Option A: Diagonal first, then horizontal
            diag_corner = (prev[0] + sign_x * abs_dy, prev[1] + sign_y * abs_dy)
            corners_to_try.append(diag_corner)
            # Option B: Horizontal first, then diagonal
            horiz_corner = (prev[0] + sign_x * (abs_dx - abs_dy), prev[1])
            corners_to_try.append(horiz_corner)
        else:
            # Option A: Diagonal first, then vertical
            diag_corner = (prev[0] + sign_x * abs_dx, prev[1] + sign_y * abs_dx)
            corners_to_try.append(diag_corner)
            # Option B: Vertical first, then diagonal
            vert_corner = (prev[0], prev[1] + sign_y * (abs_dy - abs_dx))
            corners_to_try.append(vert_corner)

        # Try each corner option
        path_found = False
        for corner in corners_to_try:
            seg1_clear = segment_is_clear_fast(prev[0], prev[1], corner[0], corner[1], index, clearance)
            seg2_clear = segment_is_clear_fast(corner[0], corner[1], target[0], target[1], index, clearance)

            if seg1_clear and seg2_clear:
                result.append(corner)
                result.append(target)
                path_found = True
                break

        if not path_found:
            # No collision-free 45-degree path found
            # Keep the original target point - the segment may not be exactly 45 degrees
            # but at least it won't create new collisions
            # The original path was routed collision-free, so keeping target is safe
            result.append(target)

    return result


def create_svg_visualization(
    path: List[Tuple[float, float]],
    obstacles: List[Obstacle],
    source: Tuple[float, float],
    sink: Tuple[float, float],
    bounds: Tuple[float, float, float, float],
    clearance: float
) -> str:
    """Create SVG visualization of path and obstacles."""
    min_x, min_y, max_x, max_y = bounds
    scale = 20  # pixels per mm
    margin = 20

    width = int((max_x - min_x) * scale) + 2 * margin
    height = int((max_y - min_y) * scale) + 2 * margin

    def to_svg(x, y):
        sx = margin + (x - min_x) * scale
        sy = margin + (max_y - y) * scale  # Flip Y
        return sx, sy

    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">']
    svg.append(f'<rect width="100%" height="100%" fill="#1a1a2e"/>')

    # Draw obstacles
    for obs in obstacles:
        sx, sy = to_svg(obs.x, obs.y)
        svg.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{obs.radius * scale:.1f}" fill="#444" stroke="#666" stroke-width="1"/>')
        # Draw clearance zone
        svg.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{(obs.radius + clearance) * scale:.1f}" fill="none" stroke="#333" stroke-width="1" stroke-dasharray="2,2"/>')

    # Draw path
    if path and len(path) > 1:
        points = " ".join(f"{to_svg(x, y)[0]:.1f},{to_svg(x, y)[1]:.1f}" for x, y in path)
        svg.append(f'<polyline points="{points}" fill="none" stroke="#00ff00" stroke-width="2"/>')

    # Draw source and sink
    sx, sy = to_svg(*source)
    svg.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="5" fill="#00ffff"/>')
    svg.append(f'<text x="{sx + 8:.1f}" y="{sy + 4:.1f}" fill="#00ffff" font-size="12">Source</text>')

    sx, sy = to_svg(*sink)
    svg.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="5" fill="#ff00ff"/>')
    svg.append(f'<text x="{sx + 8:.1f}" y="{sy + 4:.1f}" fill="#ff00ff" font-size="12">Sink</text>')

    svg.append('</svg>')
    return '\n'.join(svg)


# Test with sample data
if __name__ == '__main__':
    # Create some test obstacles
    obstacles = [
        Obstacle(5, 5, 0.5),
        Obstacle(7, 4, 0.5),
        Obstacle(8, 6, 0.5),
        Obstacle(10, 5, 0.5),
    ]

    source = (2, 5)
    sink = (15, 5)
    clearance = 0.2

    print(f"Routing from {source} to {sink}...")
    path = route_single_net(source, sink, obstacles, step_size=0.2, clearance=clearance, max_time=5.0)

    if path:
        print(f"Success! Path has {len(path)} points")
        simplified = simplify_path(path, tolerance=0.1)
        print(f"Simplified to {len(simplified)} points")

        # Create visualization
        bounds = (0, 0, 20, 10)
        svg = create_svg_visualization(simplified, obstacles, source, sink, bounds, clearance)

        with open('simple_route_test.svg', 'w') as f:
            f.write(svg)
        print("Saved visualization to simple_route_test.svg")
    else:
        print("Failed to find path")
