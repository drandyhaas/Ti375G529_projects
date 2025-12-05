#!/usr/bin/env python3
"""
Run the PyGame routing visualizer on a KiCad PCB file.

This script loads a KiCad PCB, sets up the routing context, and runs
the Rust A* router with real-time visualization.

Usage:
    python -m pygame_visualizer.run_visualizer input.kicad_pcb "Net-Name"

Or from the autorouter directory:
    python pygame_visualizer/run_visualizer.py input.kicad_pcb "Net-Name"
"""

import sys
import os
import fnmatch

# Add parent directory to path for imports
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

# Add rust_router to path
rust_router_dir = os.path.join(parent_dir, 'rust_router')
if rust_router_dir not in sys.path:
    sys.path.insert(0, rust_router_dir)

from typing import List, Tuple, Set, Optional

from kicad_parser import (
    parse_kicad_pcb, PCBData, Segment,
    auto_detect_bga_exclusion_zones, find_components_by_type
)

# Import Rust router
try:
    import grid_router
    from grid_router import GridObstacleMap, VisualRouter
    print(f"Using Rust router v{grid_router.__version__}")
except ImportError as e:
    print("ERROR: Rust router module not found!")
    print("Build it with:")
    print("  cd rust_router && cargo build --release")
    print("  cp target/release/grid_router.dll grid_router.pyd  # Windows")
    sys.exit(1)

from pygame_visualizer.visualizer import RoutingVisualizer
from pygame_visualizer.config import VisualizerConfig


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

    groups = {}
    for i in range(n):
        root = find(i)
        if root not in groups:
            groups[root] = []
        groups[root].append(segments[i])

    return list(groups.values())


class GridCoord:
    """Coordinate conversion utilities."""

    def __init__(self, grid_step: float = 0.1):
        self.grid_step = grid_step
        self.inv_step = 1.0 / grid_step

    def to_grid(self, x: float, y: float) -> Tuple[int, int]:
        return (round(x * self.inv_step), round(y * self.inv_step))

    def to_float(self, gx: int, gy: int) -> Tuple[float, float]:
        return (gx * self.grid_step, gy * self.grid_step)

    def to_grid_dist(self, dist_mm: float) -> int:
        return round(dist_mm * self.inv_step)


def build_obstacle_map_with_cache(
    pcb_data: PCBData,
    exclude_net_id: int,
    layers: List[str],
    grid_step: float = 0.1,
    track_width: float = 0.1,
    clearance: float = 0.1,
    via_size: float = 0.3,
    bga_zones: List[Tuple[float, float, float, float]] = None
) -> Tuple[GridObstacleMap, List[Set[Tuple[int, int]]], Set[Tuple[int, int]], List[Tuple[int, int, int, int]]]:
    """
    Build Rust obstacle map and return caches for visualization.

    Returns:
        (obstacles, blocked_cells_per_layer, blocked_vias, bga_zones_grid)
    """
    coord = GridCoord(grid_step)
    num_layers = len(layers)
    layer_map = {name: idx for idx, name in enumerate(layers)}

    # Create Rust obstacle map
    obstacles = GridObstacleMap(num_layers)

    # Caches for visualization
    blocked_cells: List[Set[Tuple[int, int]]] = [set() for _ in range(num_layers)]
    blocked_vias: Set[Tuple[int, int]] = set()
    bga_zones_grid: List[Tuple[int, int, int, int]] = []

    # Add BGA exclusion zones
    if bga_zones:
        for zone in bga_zones:
            min_x, min_y, max_x, max_y = zone
            gmin_x, gmin_y = coord.to_grid(min_x, min_y)
            gmax_x, gmax_y = coord.to_grid(max_x, max_y)
            obstacles.set_bga_zone(gmin_x, gmin_y, gmax_x, gmax_y)
            bga_zones_grid.append((gmin_x, gmin_y, gmax_x, gmax_y))
            # Block F.Cu cells in BGA zone
            f_cu_idx = layer_map.get('F.Cu')
            if f_cu_idx is not None:
                for gx in range(gmin_x, gmax_x + 1):
                    for gy in range(gmin_y, gmax_y + 1):
                        obstacles.add_blocked_cell(gx, gy, f_cu_idx)
                        blocked_cells[f_cu_idx].add((gx, gy))

    # Add segments as obstacles
    expansion_mm = track_width / 2 + clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_block_mm = via_size / 2 + track_width / 2 + clearance
    via_block_grid = max(1, coord.to_grid_dist(via_block_mm))

    for seg in pcb_data.segments:
        if seg.net_id == exclude_net_id:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue

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
                    blocked_cells[layer_idx].add((gx + ex, gy + ey))
            for ex in range(-via_block_grid, via_block_grid + 1):
                for ey in range(-via_block_grid, via_block_grid + 1):
                    if ex * ex + ey * ey <= via_block_grid * via_block_grid:
                        obstacles.add_blocked_via(gx + ex, gy + ey)
                        blocked_vias.add((gx + ex, gy + ey))

            if gx == gx2 and gy == gy2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                gx += sx
            if e2 < dx:
                err += dx
                gy += sy

    # Add vias as obstacles
    via_track_expansion = max(1, coord.to_grid_dist(via_size / 2 + track_width / 2 + clearance))
    via_via_expansion = max(1, coord.to_grid_dist(via_size + clearance))

    for via in pcb_data.vias:
        if via.net_id == exclude_net_id:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        for ex in range(-via_track_expansion, via_track_expansion + 1):
            for ey in range(-via_track_expansion, via_track_expansion + 1):
                if ex * ex + ey * ey <= via_track_expansion * via_track_expansion:
                    for layer_idx in range(num_layers):
                        obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
                        blocked_cells[layer_idx].add((gx + ex, gy + ey))
        for ex in range(-via_via_expansion, via_via_expansion + 1):
            for ey in range(-via_via_expansion, via_via_expansion + 1):
                if ex * ex + ey * ey <= via_via_expansion * via_via_expansion:
                    obstacles.add_blocked_via(gx + ex, gy + ey)
                    blocked_vias.add((gx + ex, gy + ey))

    # Add pads as obstacles
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id == exclude_net_id:
            continue
        for pad in pads:
            gx, gy = coord.to_grid(pad.global_x, pad.global_y)
            half_x_mm = pad.size_x / 2 + clearance
            half_y_mm = pad.size_y / 2 + clearance
            expand_x = coord.to_grid_dist(half_x_mm)
            expand_y = coord.to_grid_dist(half_y_mm)

            for ex in range(-expand_x, expand_x + 1):
                for ey in range(-expand_y, expand_y + 1):
                    for layer in pad.layers:
                        layer_idx = layer_map.get(layer)
                        if layer_idx is not None:
                            obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
                            blocked_cells[layer_idx].add((gx + ex, gy + ey))

            if 'F.Cu' in pad.layers or 'B.Cu' in pad.layers:
                via_clear_mm = via_size / 2 + clearance
                via_expand_x = int((pad.size_x / 2 + via_clear_mm) / grid_step)
                via_expand_y = int((pad.size_y / 2 + via_clear_mm) / grid_step)
                for ex in range(-via_expand_x, via_expand_x + 1):
                    for ey in range(-via_expand_y, via_expand_y + 1):
                        obstacles.add_blocked_via(gx + ex, gy + ey)
                        blocked_vias.add((gx + ex, gy + ey))

    return obstacles, blocked_cells, blocked_vias, bga_zones_grid


def get_bounds(pcb_data: PCBData, net_ids: List[int], padding: float = 5.0) -> Tuple[float, float, float, float]:
    """Get bounding box around all the nets' components."""
    xs = []
    ys = []

    for net_id in net_ids:
        pads = pcb_data.pads_by_net.get(net_id, [])
        segments = [s for s in pcb_data.segments if s.net_id == net_id]

        for pad in pads:
            xs.append(pad.global_x)
            ys.append(pad.global_y)

        for seg in segments:
            xs.extend([seg.start_x, seg.end_x])
            ys.extend([seg.start_y, seg.end_y])

    if not xs or not ys:
        return (0, 0, 100, 100)

    return (min(xs) - padding, min(ys) - padding, max(xs) + padding, max(ys) + padding)


def run_visualization(
    pcb_file: str,
    net_names: List[str],
    layers: List[str] = None,
    grid_step: float = 0.1,
    max_iterations: int = 100000
):
    """Run the visualization for multiple nets using the Rust router."""
    print(f"Loading {pcb_file}...")
    pcb_data = parse_kicad_pcb(pcb_file)

    # Default layers
    if layers is None:
        layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']

    print(f"Using layers: {layers}")

    # Auto-detect BGA zones
    bga_zones = auto_detect_bga_exclusion_zones(pcb_data, margin=0.5)
    if bga_zones:
        print(f"Detected {len(bga_zones)} BGA exclusion zone(s)")

    # Find all net IDs and validate
    net_queue = []  # List of (net_name, net_id)
    for net_name in net_names:
        net_id = None
        for nid, net in pcb_data.nets.items():
            if net.name == net_name:
                net_id = nid
                break
        if net_id is None:
            print(f"Warning: Net '{net_name}' not found, skipping")
        else:
            net_queue.append((net_name, net_id))

    if not net_queue:
        print("No valid nets to route!")
        return

    print(f"\nWill route {len(net_queue)} nets")

    coord = GridCoord(grid_step)
    layer_map = {name: idx for idx, name in enumerate(layers)}

    # Compute global bounds for all nets once (so camera shows full routing area)
    all_net_ids = [net_id for _, net_id in net_queue]
    global_bounds = get_bounds(pcb_data, all_net_ids, padding=5.0)

    # Track added segments/vias per net for restart capability
    # Key: net_idx, Value: (num_segments_added, num_vias_added)
    added_geometry = {}

    # Initialize visualizer once
    config = VisualizerConfig(layers=layers)
    visualizer = RoutingVisualizer(config)

    # Routing state
    current_net_idx = 0
    router = None
    sources = []
    targets = []
    obstacles = None
    successful = 0
    failed = 0

    def setup_net(net_idx: int) -> bool:
        """Set up routing for the net at the given index. Returns False if net should be skipped."""
        nonlocal router, sources, targets, obstacles

        if net_idx >= len(net_queue):
            return False

        net_name, net_id = net_queue[net_idx]
        print(f"\n[{net_idx + 1}/{len(net_queue)}] Setting up {net_name}...")

        # Get segments for this net
        net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
        if len(net_segments) < 2:
            print(f"  Net has only {len(net_segments)} segments, skipping")
            return False

        groups = find_connected_groups(net_segments)
        if len(groups) < 2:
            print(f"  Net already connected ({len(groups)} group), skipping")
            return False

        print(f"  Found {len(groups)} disconnected stub groups")

        # Get source and target groups
        groups.sort(key=len, reverse=True)
        source_segs = groups[0]
        target_segs = groups[1]

        # Build sources and targets
        sources = []
        for seg in source_segs:
            layer_idx = layer_map.get(seg.layer)
            if layer_idx is not None:
                gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
                gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
                sources.append((gx1, gy1, layer_idx))
                if (gx1, gy1) != (gx2, gy2):
                    sources.append((gx2, gy2, layer_idx))

        targets = []
        for seg in target_segs:
            layer_idx = layer_map.get(seg.layer)
            if layer_idx is not None:
                gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
                gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
                targets.append((gx1, gy1, layer_idx))
                if (gx1, gy1) != (gx2, gy2):
                    targets.append((gx2, gy2, layer_idx))

        if not sources or not targets:
            print("  No valid source/target points, skipping")
            return False

        print(f"  Source points: {len(sources)}, Target points: {len(targets)}")

        # Build obstacles with cache for visualization
        print("  Building obstacle map...")
        obstacles, blocked_cells, blocked_vias, bga_zones_grid = build_obstacle_map_with_cache(
            pcb_data, net_id, layers,
            grid_step=grid_step,
            bga_zones=bga_zones
        )

        # Add allowed cells around sources/targets
        allow_radius = 10
        for gx, gy, _ in sources + targets:
            for dx in range(-allow_radius, allow_radius + 1):
                for dy in range(-allow_radius, allow_radius + 1):
                    obstacles.add_allowed_cell(gx + dx, gy + dy)

        # Update visualizer context (use global_bounds to show full routing area)
        visualizer.set_routing_context(
            obstacles,
            sources,
            targets,
            grid_step=grid_step,
            bounds=global_bounds,
            bga_zones=bga_zones_grid,
            blocked_cells=blocked_cells,
            blocked_vias=blocked_vias,
        )

        # Initialize Rust VisualRouter
        via_cost = 500 * 1000  # Same as batch_grid_router
        h_weight = 1.5
        router = VisualRouter(via_cost=via_cost, h_weight=h_weight)
        router.init(sources, targets, max_iterations)

        return True

    def add_path_to_pcb(path, net_id: int, net_idx: int):
        """Add a routed path to pcb_data so subsequent routes avoid it."""
        from kicad_parser import Segment, Via
        layer_names = layers

        segs_before = len(pcb_data.segments)
        vias_before = len(pcb_data.vias)

        for i in range(len(path) - 1):
            gx1, gy1, layer1 = path[i]
            gx2, gy2, layer2 = path[i + 1]

            x1, y1 = coord.to_float(gx1, gy1)
            x2, y2 = coord.to_float(gx2, gy2)

            if layer1 != layer2:
                # Via
                via = Via(
                    x=x1, y=y1,
                    size=0.3, drill=0.2,
                    layers=[layer_names[layer1], layer_names[layer2]],
                    net_id=net_id
                )
                pcb_data.vias.append(via)
            else:
                # Segment
                if (x1, y1) != (x2, y2):
                    seg = Segment(
                        start_x=x1, start_y=y1,
                        end_x=x2, end_y=y2,
                        width=0.1,
                        layer=layer_names[layer1],
                        net_id=net_id
                    )
                    pcb_data.segments.append(seg)

        # Track how many were added so we can remove them on restart
        segs_added = len(pcb_data.segments) - segs_before
        vias_added = len(pcb_data.vias) - vias_before
        added_geometry[net_idx] = (segs_added, vias_added)

    def remove_net_geometry(net_idx: int):
        """Remove previously routed geometry for a net."""
        if net_idx in added_geometry:
            segs_added, vias_added = added_geometry[net_idx]
            if segs_added > 0:
                pcb_data.segments = pcb_data.segments[:-segs_added]
            if vias_added > 0:
                pcb_data.vias = pcb_data.vias[:-vias_added]
            del added_geometry[net_idx]

    # Set up first net
    while current_net_idx < len(net_queue) and not setup_net(current_net_idx):
        failed += 1
        current_net_idx += 1

    if current_net_idx >= len(net_queue):
        print("\nNo routable nets found!")
        return

    print("\nStarting visualization with Rust router...")
    print("Controls: Space=pause, R=restart net, Ctrl+R=restart all, +/-=speed, Q=quit")

    # Keep original pcb_data for restart all
    original_segments = list(pcb_data.segments)
    original_vias = list(pcb_data.vias)

    try:
        while visualizer.running:
            # Handle events
            if not visualizer.handle_events():
                break

            # Check for restart all request (Ctrl+R)
            if visualizer.restart_all_requested:
                visualizer.restart_all_requested = False
                # Reset pcb_data to original state
                pcb_data.segments = list(original_segments)
                pcb_data.vias = list(original_vias)
                # Reset counters and start from first net
                current_net_idx = 0
                successful = 0
                failed = 0
                while current_net_idx < len(net_queue) and not setup_net(current_net_idx):
                    failed += 1
                    current_net_idx += 1
                print("\nRestarting all nets...")
                continue

            # Check for restart request (restart last/current net)
            if visualizer.restart_requested:
                visualizer.restart_requested = False
                # If router is done or None, go back to previous net
                if router is None or router.is_done():
                    if current_net_idx > 0:
                        current_net_idx -= 1
                # Remove any geometry added by this net before restarting
                remove_net_geometry(current_net_idx)
                # Re-setup the current net
                if current_net_idx < len(net_queue):
                    setup_net(current_net_idx)
                    print(f"\nRestarting {net_queue[current_net_idx][0]}...")

            # Advance search if not paused and not done
            if visualizer.should_step() and router and not router.is_done():
                # Run multiple iterations using Rust router
                snapshot = router.step(obstacles, visualizer.iterations_per_frame)
                visualizer.update_snapshot(snapshot)

                if snapshot.found:
                    net_name, net_id = net_queue[current_net_idx]
                    print(f"\nPath found for {net_name} in {snapshot.iteration} iterations!")
                    print(f"Path length: {len(snapshot.path) if snapshot.path else 0}")
                    successful += 1

                    # Add path to pcb_data for subsequent routes
                    if snapshot.path:
                        add_path_to_pcb(snapshot.path, net_id, current_net_idx)

                    # Auto-advance to next net after a short delay
                    # (user can see the completed path)

            # Check if current route is done (found or exhausted)
            if router and router.is_done():
                snapshot = visualizer.snapshot
                if snapshot and not snapshot.found:
                    net_name, _ = net_queue[current_net_idx]
                    print(f"\nNo path found for {net_name} after {snapshot.iteration} iterations")
                    failed += 1

                # Move to next net
                current_net_idx += 1
                while current_net_idx < len(net_queue) and not setup_net(current_net_idx):
                    failed += 1
                    current_net_idx += 1

                if current_net_idx >= len(net_queue):
                    print(f"\n{'='*60}")
                    print(f"All nets processed: {successful} successful, {failed} failed")
                    print("Press Q to quit or close window")
                    router = None  # Stop routing

            # Render
            visualizer.render()
            visualizer.tick()

    finally:
        visualizer.quit()

    return successful, failed


def expand_net_pattern(pcb_data: PCBData, pattern: str) -> List[str]:
    """
    Expand a wildcard pattern to matching net names.

    Patterns can include * and ? wildcards (fnmatch style).
    Example: "Net-(U2A-DATA_*)" matches Net-(U2A-DATA_0), Net-(U2A-DATA_1), etc.

    Returns list of matching net names in sorted order.
    """
    all_net_names = [net.name for net in pcb_data.nets.values()]

    if '*' in pattern or '?' in pattern:
        matches = sorted([name for name in all_net_names if fnmatch.fnmatch(name, pattern)])
        return matches
    else:
        # Literal net name
        return [pattern] if pattern in all_net_names else []


def main():
    if len(sys.argv) < 3:
        print("Usage: python run_visualizer.py input.kicad_pcb \"Net-Pattern\" [\"Net-Pattern2\" ...]")
        print("\nWildcard patterns supported:")
        print('  "Net-(U2A-DATA_*)"  - matches Net-(U2A-DATA_0), Net-(U2A-DATA_1), etc.')
        print("\nExample:")
        print('  python run_visualizer.py fanout_starting_point.kicad_pcb "Net-(U2A-DATA_*)"')
        sys.exit(1)

    pcb_file = sys.argv[1]
    net_patterns = sys.argv[2:]

    if not os.path.exists(pcb_file):
        print(f"File not found: {pcb_file}")
        sys.exit(1)

    # Load PCB to expand wildcards
    print(f"Loading {pcb_file}...")
    pcb_data = parse_kicad_pcb(pcb_file)

    # Expand all patterns
    all_nets = []
    seen = set()
    for pattern in net_patterns:
        matching_nets = expand_net_pattern(pcb_data, pattern)
        if not matching_nets:
            print(f"Warning: Pattern '{pattern}' matched no nets")
        else:
            if '*' in pattern or '?' in pattern:
                print(f"Pattern '{pattern}' matched {len(matching_nets)} nets")
            for net in matching_nets:
                if net not in seen:
                    all_nets.append(net)
                    seen.add(net)

    if not all_nets:
        print("No nets matched the given patterns!")
        print("\nAvailable nets:")
        for nid, net in list(pcb_data.nets.items())[:20]:
            print(f"  {net.name}")
        if len(pcb_data.nets) > 20:
            print(f"  ... and {len(pcb_data.nets) - 20} more")
        sys.exit(1)

    print(f"\nWill visualize routing of {len(all_nets)} nets")

    run_visualization(pcb_file, all_nets)


if __name__ == "__main__":
    main()
