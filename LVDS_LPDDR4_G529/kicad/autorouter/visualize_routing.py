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
from grid_astar_router import (
    GridRouteConfig, GridObstacleMap, GridAStarRouter, GridState, GridCoord
)
from batch_grid_router import find_connected_groups


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


def draw_obstacles(ax, pcb_data: PCBData, config: GridRouteConfig,
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


def draw_stub_groups(ax, groups: List[List[Segment]]):
    """Draw stub groups with different markers."""
    markers = ['*', 'o', 's', '^', 'v', 'D']
    for i, group in enumerate(groups):
        # Calculate centroid
        points = []
        for seg in group:
            points.append((seg.start_x, seg.start_y))
            points.append((seg.end_x, seg.end_y))
        if points:
            cx = sum(p[0] for p in points) / len(points)
            cy = sum(p[1] for p in points) / len(points)
            marker = markers[i % len(markers)]
            color = ['red', 'blue', 'green', 'orange'][i % 4]
            ax.plot(cx, cy, marker=marker, markersize=15, color=color,
                    markeredgecolor='black', markeredgewidth=1, zorder=10)
            ax.annotate(f'G{i}', (cx, cy), textcoords="offset points",
                       xytext=(5, 5), fontsize=10, fontweight='bold', zorder=11)


def draw_stub_proximity(ax, obstacles: GridObstacleMap, bounds: Tuple[float, float, float, float]):
    """Draw stub proximity cost field as a heatmap."""
    min_x, max_x, min_y, max_y = bounds
    coord = obstacles.coord

    # Sample the proximity field
    step = obstacles.config.grid_step
    xs = []
    ys = []
    costs = []

    x = min_x
    while x <= max_x:
        y = min_y
        while y <= max_y:
            gx, gy = coord.to_grid(x, y)
            cost = obstacles.stub_proximity.get((gx, gy), 0)
            if cost > 0:
                xs.append(x)
                ys.append(y)
                costs.append(cost)
            y += step
        x += step

    if xs:
        scatter = ax.scatter(xs, ys, c=costs, cmap='Reds', s=20, alpha=0.5, zorder=2)
        return scatter
    return None


class VisualizingGridRouter(GridAStarRouter):
    """Grid Router that records exploration for visualization."""

    def __init__(self, obstacles: GridObstacleMap, config: GridRouteConfig):
        super().__init__(obstacles, config)
        self.exploration_history = []  # List of (iteration, gx, gy, layer_idx, g_cost)
        self.unique_positions = set()

    def route_segments_to_segments_visualized(self, source_segments: List[Segment],
                                               target_segments: List[Segment],
                                               max_iterations: int = None):
        """Route with exploration recording."""
        import heapq

        if max_iterations is None:
            max_iterations = self.config.max_iterations

        self.exploration_history = []
        self.unique_positions = set()

        # Sample source segments to get starting points
        start_points = []
        for seg in source_segments:
            layer_idx = self.layer_mapper.get_idx(seg.layer)
            if layer_idx is None:
                continue
            dx = seg.end_x - seg.start_x
            dy = seg.end_y - seg.start_y
            length = math.sqrt(dx*dx + dy*dy)
            if length < 0.001:
                gx, gy = self.coord.to_grid(seg.start_x, seg.start_y)
                start_points.append((GridState(gx, gy, layer_idx), (seg.start_x, seg.start_y)))
                continue
            num_points = max(2, int(length / self.config.grid_step) + 1)
            for i in range(num_points):
                t = i / (num_points - 1)
                x = seg.start_x + t * dx
                y = seg.start_y + t * dy
                gx, gy = self.coord.to_grid(x, y)
                start_points.append((GridState(gx, gy, layer_idx), (x, y)))

        if not start_points:
            return None

        # Sample target segments
        target_points = []
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

        target_float_coords = {pt[0]: pt[1] for pt in target_points}
        target_keys = [pt[0] for pt in target_points]
        target_set = set(target_keys)

        # Multi-source A*
        counter = 0
        open_set = []
        g_costs = {}
        parents = {}
        source_origins = {}
        closed = set()

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

            # Record exploration
            self.exploration_history.append((
                iterations, current.gx, current.gy, current.layer_idx, g
            ))
            self.unique_positions.add(current_key)

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


def visualize_search(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
                     output_prefix: str = "route_viz",
                     unrouted_stubs: List[Tuple[float, float]] = None):
    """Create visualization of A* search for a net."""
    if not HAS_MATPLOTLIB:
        print("Cannot create visualization - matplotlib not installed")
        return

    # Get segments for this net
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    if len(net_segments) < 2:
        print(f"Net {net_id} has fewer than 2 segments")
        return

    # Find disconnected groups
    groups = find_connected_groups(net_segments)
    if len(groups) < 2:
        print(f"Net {net_id} segments are already connected")
        return

    print(f"Visualizing route for net {net_id}")
    print(f"  Found {len(groups)} disconnected stub groups")

    # Get bounds
    bounds = get_bounds(pcb_data, net_id, padding=3.0)

    # Build obstacles with stub proximity
    obstacles = GridObstacleMap(pcb_data, config, exclude_net_id=net_id,
                                unrouted_stubs=unrouted_stubs)
    router = VisualizingGridRouter(obstacles, config)
    coord = GridCoord(config.grid_step)

    # Create figure
    fig, axes = plt.subplots(2, 2, figsize=(16, 16))

    # Plot 1: Overview with obstacles and stub groups
    ax1 = axes[0, 0]
    ax1.set_title(f"Net {net_id} - Overview ({len(groups)} stub groups)")
    draw_obstacles(ax1, pcb_data, config, bounds, exclude_net_id=net_id)
    draw_net_stubs(ax1, pcb_data, net_id)
    draw_stub_groups(ax1, groups)
    ax1.set_xlim(bounds[0], bounds[1])
    ax1.set_ylim(bounds[3], bounds[2])  # Flip Y for PCB coordinates
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)

    # Route between the two largest groups
    groups.sort(key=len, reverse=True)
    source_segs = groups[0]
    target_segs = groups[1]

    print(f"  Routing from group 0 ({len(source_segs)} segs) to group 1 ({len(target_segs)} segs)")
    result = router.route_segments_to_segments_visualized(source_segs, target_segs,
                                                          max_iterations=config.max_iterations)

    # Try reverse direction if first direction failed
    if not result:
        print(f"  Trying reverse direction...")
        router.exploration_history = []
        router.unique_positions = set()
        result = router.route_segments_to_segments_visualized(target_segs, source_segs,
                                                              max_iterations=config.max_iterations)

    # Plot 2: Search exploration colored by iteration
    ax2 = axes[0, 1]
    ax2.set_title(f"A* Exploration (colored by iteration)")
    draw_obstacles(ax2, pcb_data, config, bounds, exclude_net_id=net_id)
    draw_net_stubs(ax2, pcb_data, net_id)

    if router.exploration_history:
        iterations = [h[0] for h in router.exploration_history]
        xs = [coord.to_float(h[1], 0)[0] for h in router.exploration_history]
        ys = [coord.to_float(0, h[2])[1] for h in router.exploration_history]

        max_iter = max(iterations)
        colors = [i / max_iter for i in iterations]

        scatter = ax2.scatter(xs, ys, c=colors, cmap='viridis',
                             s=20, alpha=0.7, zorder=3)
        plt.colorbar(scatter, ax=ax2, label='Iteration (normalized)')

    ax2.set_xlim(bounds[0], bounds[1])
    ax2.set_ylim(bounds[3], bounds[2])
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)

    # Plot 3: Search exploration by layer + stub proximity field
    ax3 = axes[1, 0]
    ax3.set_title(f"A* Exploration by Layer + Stub Proximity")
    draw_obstacles(ax3, pcb_data, config, bounds, exclude_net_id=net_id)
    draw_net_stubs(ax3, pcb_data, net_id)

    # Draw stub proximity field
    if unrouted_stubs:
        prox_scatter = draw_stub_proximity(ax3, obstacles, bounds)
        if prox_scatter:
            plt.colorbar(prox_scatter, ax=ax3, label='Stub Proximity Cost')

    if router.exploration_history:
        for layer_idx, layer_name in enumerate(config.layers):
            layer_points = [(coord.to_float(h[1], h[2]) ) for h in router.exploration_history if h[3] == layer_idx]
            if layer_points:
                xs = [p[0] for p in layer_points]
                ys = [p[1] for p in layer_points]
                color = LAYER_COLORS.get(layer_name, 'gray')
                ax3.scatter(xs, ys, c=color, s=25, alpha=0.7,
                           label=f"{layer_name} ({len(layer_points)} pts)", zorder=3)
        ax3.legend(loc='upper right', fontsize=8)

    ax3.set_xlim(bounds[0], bounds[1])
    ax3.set_ylim(bounds[3], bounds[2])
    ax3.set_aspect('equal')
    ax3.grid(True, alpha=0.3)

    # Plot 4: Final path
    ax4 = axes[1, 1]
    ax4.set_title(f"Final Path" + (" (FOUND)" if result else " (NOT FOUND)"))
    draw_obstacles(ax4, pcb_data, config, bounds, exclude_net_id=net_id)
    draw_net_stubs(ax4, pcb_data, net_id)
    draw_stub_groups(ax4, groups)

    if result:
        path, src_coords, tgt_coords = result
        # Convert to float and draw
        float_path = router.path_to_float(path)

        for i in range(len(float_path) - 1):
            x1, y1, layer1 = float_path[i]
            x2, y2, layer2 = float_path[i + 1]

            if layer1 == layer2:
                color = LAYER_COLORS.get(layer1, 'orange')
                ax4.plot([x1, x2], [y1, y2],
                        color=color, linewidth=3, alpha=0.9, zorder=8)
            else:
                # Via
                ax4.plot(x1, y1, 'ko', markersize=8, zorder=9)

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
            layer_idx = h[3]
            layer_name = config.layers[layer_idx] if layer_idx < len(config.layers) else f"L{layer_idx}"
            layer_counts[layer_name] = layer_counts.get(layer_name, 0) + 1

        print(f"    Iterations per layer:")
        for layer, count in sorted(layer_counts.items()):
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

    config = GridRouteConfig(
        track_width=0.1,
        clearance=0.1,
        via_size=0.3,
        via_drill=0.2,
        grid_step=0.1,
        via_cost=500,
        layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
        max_iterations=50000,
        heuristic_weight=1.5,
        bga_exclusion_zone=(185.9, 93.5, 204.9, 112.5),
        stub_proximity_radius=1.0,
        stub_proximity_cost=3.0,
    )

    # For demonstration, create some dummy unrouted stubs
    # In real use, you'd collect these from other nets
    unrouted_stubs = []

    visualize_search(pcb_data, net_id, config, output_prefix, unrouted_stubs)
