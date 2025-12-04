"""
Visualize PCB routing obstacles and A* search progress.

Creates images showing:
- Obstacles (pads, vias, existing tracks)
- Stub endpoints to connect
- A* search exploration over time
- Final path found
"""

import sys
import math
from typing import List, Tuple, Optional, Set, Dict
from dataclasses import dataclass

# Try to import visualization libraries
try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    from matplotlib.collections import LineCollection, PatchCollection
    import numpy as np
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("matplotlib not available - install with: pip install matplotlib")

from kicad_parser import parse_kicad_pcb, PCBData, Segment, Via, Pad
from astar_router import (
    RouteConfig, ObstacleGrid, State, AStarRouter,
    find_stub_endpoints, find_stub_segments
)


# Layer colors
LAYER_COLORS = {
    'F.Cu': '#FF0000',      # Red
    'In1.Cu': '#00AA00',    # Green
    'In2.Cu': '#0000FF',    # Blue
    'B.Cu': '#AA00AA',      # Magenta
}

LAYER_ALPHA = {
    'F.Cu': 0.8,
    'In1.Cu': 0.5,
    'In2.Cu': 0.5,
    'B.Cu': 0.6,
}


def get_bounds(pcb_data: PCBData, net_id: int, padding: float = 5.0) -> Tuple[float, float, float, float]:
    """Get bounding box around the net's components with padding."""
    pads = pcb_data.pads_by_net.get(net_id, [])
    segments = [s for s in pcb_data.segments if s.net_id == net_id]

    if not pads and not segments:
        return (0, 0, 100, 100)

    xs = []
    ys = []

    for pad in pads:
        xs.append(pad.global_x)
        ys.append(pad.global_y)

    for seg in segments:
        xs.extend([seg.start_x, seg.end_x])
        ys.extend([seg.start_y, seg.end_y])

    min_x = min(xs) - padding
    max_x = max(xs) + padding
    min_y = min(ys) - padding
    max_y = max(ys) + padding

    return (min_x, max_x, min_y, max_y)


def draw_obstacles(ax, pcb_data: PCBData, config: RouteConfig,
                   bounds: Tuple[float, float, float, float],
                   exclude_net_id: int = None):
    """Draw all obstacles within bounds."""
    min_x, max_x, min_y, max_y = bounds

    # Draw pads as circles
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id == exclude_net_id:
            continue
        for pad in pads:
            if min_x <= pad.global_x <= max_x and min_y <= pad.global_y <= max_y:
                radius = max(pad.size_x, pad.size_y) / 2
                circle = plt.Circle((pad.global_x, pad.global_y), radius,
                                    color='gray', alpha=0.5, zorder=1)
                ax.add_patch(circle)

    # Draw vias as small circles
    for via in pcb_data.vias:
        if via.net_id == exclude_net_id:
            continue
        if min_x <= via.x <= max_x and min_y <= via.y <= max_y:
            circle = plt.Circle((via.x, via.y), via.size / 2,
                                color='purple', alpha=0.5, zorder=1)
            ax.add_patch(circle)

    # Draw segments as lines
    for seg in pcb_data.segments:
        if seg.net_id == exclude_net_id:
            continue
        # Check if segment is in bounds
        if not (min_x <= seg.start_x <= max_x or min_x <= seg.end_x <= max_x):
            continue
        if not (min_y <= seg.start_y <= max_y or min_y <= seg.end_y <= max_y):
            continue

        color = 'lightgray'
        ax.plot([seg.start_x, seg.end_x], [seg.start_y, seg.end_y],
                color=color, linewidth=seg.width * 10, alpha=0.3, zorder=1,
                solid_capstyle='round')


def draw_net_stubs(ax, pcb_data: PCBData, net_id: int):
    """Draw the stubs for the net being routed."""
    segments = [s for s in pcb_data.segments if s.net_id == net_id]
    pads = pcb_data.pads_by_net.get(net_id, [])

    # Draw pads
    for pad in pads:
        color = LAYER_COLORS.get(pad.layers[0] if pad.layers else 'F.Cu', 'orange')
        radius = max(pad.size_x, pad.size_y) / 2
        circle = plt.Circle((pad.global_x, pad.global_y), radius,
                            color=color, alpha=0.8, zorder=5)
        ax.add_patch(circle)

    # Draw stub segments
    for seg in segments:
        color = LAYER_COLORS.get(seg.layer, 'orange')
        ax.plot([seg.start_x, seg.end_x], [seg.start_y, seg.end_y],
                color=color, linewidth=2, alpha=0.9, zorder=5,
                solid_capstyle='round')


def draw_endpoints(ax, endpoints: List[Tuple[float, float, str]]):
    """Draw stub endpoints that need to be connected."""
    for i, (x, y, layer) in enumerate(endpoints):
        color = LAYER_COLORS.get(layer, 'orange')
        # Draw a star marker
        ax.plot(x, y, marker='*', markersize=15, color=color,
                markeredgecolor='black', markeredgewidth=1, zorder=10)
        ax.annotate(f'EP{i}', (x, y), textcoords="offset points",
                   xytext=(5, 5), fontsize=8, zorder=11)


class VisualizingRouter(AStarRouter):
    """A* Router that records exploration for visualization."""

    def __init__(self, obstacles: ObstacleGrid, config: RouteConfig):
        super().__init__(obstacles, config)
        self.exploration_history = []  # List of (iteration, x, y, layer, f_score)
        self.record_interval = 1  # Record EVERY iteration for detailed visualization
        self.unique_positions = set()  # Track unique (x, y, layer) positions explored

    def route(self, start: State, goal: State,
              escape_start: Tuple[float, float] = None) -> Optional[List[State]]:
        """Route with exploration recording."""
        import heapq

        self.exploration_history = []
        self.unique_positions = set()

        start_x, start_y = start.x, start.y
        if escape_start:
            start_x, start_y = escape_start

        # A* setup
        g_scores = {start: 0}
        f_scores = {start: self._heuristic(start, goal)}
        came_from = {}

        open_set = [(f_scores[start], id(start), start)]
        open_set_lookup = {start}
        closed_set = set()

        iteration = 0

        while open_set and iteration < self.config.max_iterations:
            iteration += 1

            _, _, current = heapq.heappop(open_set)
            open_set_lookup.discard(current)

            # Track unique position
            pos_key = (round(current.x, 2), round(current.y, 2), current.layer)
            self.unique_positions.add(pos_key)

            # Record exploration - every iteration for detailed viz
            self.exploration_history.append((
                iteration,
                current.x,
                current.y,
                current.layer,
                f_scores.get(current, 0)
            ))

            # Check if reached goal
            if self._reached_goal(current, goal):
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                print(f"  Found path in {iteration} iterations")
                return path

            closed_set.add(current)

            # Get neighbors
            neighbors = self._get_neighbors(current, start_x, start_y)

            for neighbor, cost in neighbors:
                if neighbor in closed_set:
                    continue

                tentative_g = g_scores[current] + cost

                if neighbor not in g_scores or tentative_g < g_scores[neighbor]:
                    came_from[neighbor] = current
                    g_scores[neighbor] = tentative_g
                    f = tentative_g + self.config.heuristic_weight * self._heuristic(neighbor, goal)
                    f_scores[neighbor] = f

                    if neighbor not in open_set_lookup:
                        heapq.heappush(open_set, (f, id(neighbor), neighbor))
                        open_set_lookup.add(neighbor)

        print(f"  Failed after {iteration} iterations")
        return None

    def _reached_goal(self, current: State, goal: State, tolerance: float = 0.15) -> bool:
        """Check if current state has reached the goal."""
        if current.layer != goal.layer:
            return False
        dist = math.sqrt((current.x - goal.x)**2 + (current.y - goal.y)**2)
        return dist < tolerance

    def _get_neighbors(self, state: State,
                       escape_x: float, escape_y: float) -> List[Tuple[State, float]]:
        """Get valid neighboring states."""
        neighbors = []
        step = self.config.grid_step

        # Check if in escape zone
        dist_from_start = math.sqrt((state.x - escape_x)**2 + (state.y - escape_y)**2)
        in_escape = dist_from_start < self.config.escape_radius
        reduced_clearance = self.config.escape_clearance if in_escape else None

        # 8 directions on same layer
        for dx, dy in self.DIRECTIONS:
            nx = state.x + dx * step
            ny = state.y + dy * step

            if not self.obstacles.segment_collides(state.x, state.y, nx, ny,
                                                   state.layer, reduced_clearance):
                cost = step * self.sqrt2 if (dx != 0 and dy != 0) else step
                # Add repulsion cost to discourage routing near obstacles
                cost += self.obstacles.repulsion_cost(nx, ny, state.layer)
                neighbors.append((State(nx, ny, state.layer), cost))

        # Layer changes (vias)
        if not self.obstacles.via_collides(state.x, state.y):
            for layer in self.config.layers:
                if layer != state.layer:
                    neighbors.append((State(state.x, state.y, layer),
                                     self.config.via_cost))

        return neighbors


def visualize_search(pcb_data: PCBData, net_id: int, config: RouteConfig,
                     output_prefix: str = "route_viz"):
    """Create visualization of A* search for a net."""
    if not HAS_MATPLOTLIB:
        print("Cannot create visualization - matplotlib not installed")
        return

    # Get endpoints
    endpoints = find_stub_endpoints(pcb_data, net_id)
    if len(endpoints) < 2:
        print(f"Net {net_id} has fewer than 2 endpoints")
        return

    print(f"Visualizing route for net {net_id}")
    print(f"  Endpoints: {endpoints}")

    # Get bounds
    bounds = get_bounds(pcb_data, net_id, padding=3.0)

    # Build obstacles
    obstacles = ObstacleGrid(pcb_data, config, exclude_net_id=net_id)
    router = VisualizingRouter(obstacles, config)

    # Create figure
    fig, axes = plt.subplots(2, 2, figsize=(16, 16))

    # Plot 1: Overview with obstacles
    ax1 = axes[0, 0]
    ax1.set_title(f"Net {net_id} - Overview")
    draw_obstacles(ax1, pcb_data, config, bounds, exclude_net_id=net_id)
    draw_net_stubs(ax1, pcb_data, net_id)
    draw_endpoints(ax1, endpoints)
    ax1.set_xlim(bounds[0], bounds[1])
    ax1.set_ylim(bounds[3], bounds[2])  # Flip Y for PCB coordinates
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)

    # Route between first two endpoints
    start = State(endpoints[0][0], endpoints[0][1], endpoints[0][2])
    goal = State(endpoints[1][0], endpoints[1][1], endpoints[1][2])

    print(f"  Routing from {start} to {goal}")
    path = router.route(start, goal, escape_start=(start.x, start.y))

    # Plot 2: Search exploration colored by iteration
    ax2 = axes[0, 1]
    ax2.set_title(f"A* Exploration (colored by iteration)")
    draw_obstacles(ax2, pcb_data, config, bounds, exclude_net_id=net_id)
    draw_net_stubs(ax2, pcb_data, net_id)
    draw_endpoints(ax2, endpoints)

    if router.exploration_history:
        # Color points by iteration
        iterations = [h[0] for h in router.exploration_history]
        xs = [h[1] for h in router.exploration_history]
        ys = [h[2] for h in router.exploration_history]
        layers = [h[3] for h in router.exploration_history]

        # Normalize iterations for coloring
        max_iter = max(iterations)
        colors = [i / max_iter for i in iterations]

        scatter = ax2.scatter(xs, ys, c=colors, cmap='viridis',
                             s=20, alpha=0.7, zorder=3)
        plt.colorbar(scatter, ax=ax2, label='Iteration (normalized)')

    ax2.set_xlim(bounds[0], bounds[1])
    ax2.set_ylim(bounds[3], bounds[2])
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)

    # Plot 3: Search exploration by layer
    ax3 = axes[1, 0]
    ax3.set_title(f"A* Exploration by Layer")
    draw_obstacles(ax3, pcb_data, config, bounds, exclude_net_id=net_id)
    draw_net_stubs(ax3, pcb_data, net_id)
    draw_endpoints(ax3, endpoints)

    if router.exploration_history:
        for layer in config.layers:
            layer_points = [(h[1], h[2]) for h in router.exploration_history if h[3] == layer]
            if layer_points:
                xs, ys = zip(*layer_points)
                color = LAYER_COLORS.get(layer, 'gray')
                ax3.scatter(xs, ys, c=color, s=25, alpha=0.7,
                           label=f"{layer} ({len(layer_points)} pts)", zorder=3)
        ax3.legend(loc='upper right', fontsize=8)

    ax3.set_xlim(bounds[0], bounds[1])
    ax3.set_ylim(bounds[3], bounds[2])
    ax3.set_aspect('equal')
    ax3.grid(True, alpha=0.3)

    # Plot 4: Final path
    ax4 = axes[1, 1]
    ax4.set_title(f"Final Path" + (" (FOUND)" if path else " (NOT FOUND)"))
    draw_obstacles(ax4, pcb_data, config, bounds, exclude_net_id=net_id)
    draw_net_stubs(ax4, pcb_data, net_id)
    draw_endpoints(ax4, endpoints)

    if path:
        # Draw path segments
        for i in range(len(path) - 1):
            curr = path[i]
            next_ = path[i + 1]

            if curr.layer == next_.layer:
                # Same layer - draw segment
                color = LAYER_COLORS.get(curr.layer, 'orange')
                ax4.plot([curr.x, next_.x], [curr.y, next_.y],
                        color=color, linewidth=3, alpha=0.9, zorder=8)
            else:
                # Via
                ax4.plot(curr.x, curr.y, 'ko', markersize=8, zorder=9)

        print(f"  Path has {len(path)} waypoints")

    ax4.set_xlim(bounds[0], bounds[1])
    ax4.set_ylim(bounds[3], bounds[2])
    ax4.set_aspect('equal')
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()

    # Save figure
    output_file = f"{output_prefix}_net{net_id}.png"
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"  Saved visualization to {output_file}")

    plt.close()

    # Print statistics
    if router.exploration_history:
        print(f"\n  Exploration statistics:")
        print(f"    Total iterations: {len(router.exploration_history)}")
        print(f"    Unique positions explored: {len(router.unique_positions)}")

        layer_counts = {}
        for h in router.exploration_history:
            layer = h[3]
            layer_counts[layer] = layer_counts.get(layer, 0) + 1

        print(f"    Iterations per layer:")
        for layer, count in sorted(layer_counts.items()):
            print(f"      {layer}: {count}")

        # Count unique positions by layer
        unique_layer_counts = {}
        for pos in router.unique_positions:
            layer = pos[2]
            unique_layer_counts[layer] = unique_layer_counts.get(layer, 0) + 1

        print(f"    Unique positions per layer:")
        for layer, count in sorted(unique_layer_counts.items()):
            print(f"      {layer}: {count}")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python visualize_routing.py input.kicad_pcb net_name [output_prefix]")
        print("\nExample:")
        print('  python visualize_routing.py fanout_starting_point.kicad_pcb "Net-(U2A-DATA_0)"')
        sys.exit(1)

    input_file = sys.argv[1]
    net_name = sys.argv[2]
    output_prefix = sys.argv[3] if len(sys.argv) > 3 else "route_viz"

    print(f"Loading {input_file}...")
    pcb_data = parse_kicad_pcb(input_file)

    # Find net ID
    net_id = None
    for nid, net in pcb_data.nets.items():
        if net.name == net_name:
            net_id = nid
            break

    if net_id is None:
        print(f"Net '{net_name}' not found")
        sys.exit(1)

    config = RouteConfig(
        track_width=0.1,
        clearance=0.1,
        via_size=0.3,
        via_drill=0.2,
        grid_step=0.1,  # Fine grid to show actual search behavior
        via_cost=0.5,
        layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
        max_iterations=10000,  # Limit for quick visualization
        escape_radius=2.0,
        escape_clearance=0.02,
        heuristic_weight=1.5,  # Moderate heuristic weighting
        # BGA exclusion zone - vias inside are ignored as obstacles
        # bga_exclusion_zone disabled - let repulsion handle obstacle avoidance
        # Obstacle repulsion - keeps routes away from obstacles
        repulsion_distance=2.0,  # mm - repulsion field radius
        repulsion_cost=2.0,  # mm equivalent cost at clearance boundary
    )

    visualize_search(pcb_data, net_id, config, output_prefix)
