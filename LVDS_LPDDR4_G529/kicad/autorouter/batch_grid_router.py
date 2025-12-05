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
    via_cost: int = 500  # grid steps equivalent penalty for via
    layers: List[str] = field(default_factory=lambda: ['F.Cu', 'B.Cu'])
    max_iterations: int = 100000
    heuristic_weight: float = 1.5
    # BGA exclusion zones (auto-detected from PCB) - vias blocked inside these areas
    bga_exclusion_zones: List[Tuple[float, float, float, float]] = field(default_factory=list)
    stub_proximity_radius: float = 1.0  # mm - radius around stubs to penalize
    stub_proximity_cost: float = 3.0  # mm equivalent cost at stub center


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


def build_obstacle_map(pcb_data: PCBData, config: GridRouteConfig,
                       exclude_net_id: int, unrouted_stubs: Optional[List[Tuple[float, float]]] = None) -> GridObstacleMap:
    """Build Rust obstacle map from PCB data."""
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)
    layer_map = {name: idx for idx, name in enumerate(config.layers)}

    obstacles = GridObstacleMap(num_layers)

    # Set BGA exclusion zones
    for zone in config.bga_exclusion_zones:
        min_x, min_y, max_x, max_y = zone
        gmin_x, gmin_y = coord.to_grid(min_x, min_y)
        gmax_x, gmax_y = coord.to_grid(max_x, max_y)
        obstacles.set_bga_zone(gmin_x, gmin_y, gmax_x, gmax_y)

    # Add segments as obstacles
    expansion_mm = config.track_width / 2 + config.clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_block_mm = config.via_size / 2 + config.track_width / 2 + config.clearance
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

    # Add vias as obstacles
    via_expansion_grid = max(1, coord.to_grid_dist(config.via_size / 2 + config.track_width / 2 + config.clearance))
    for via in pcb_data.vias:
        if via.net_id == exclude_net_id:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        for ex in range(-via_expansion_grid, via_expansion_grid + 1):
            for ey in range(-via_expansion_grid, via_expansion_grid + 1):
                if ex*ex + ey*ey <= via_expansion_grid * via_expansion_grid:
                    for layer_idx in range(num_layers):
                        obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
                    obstacles.add_blocked_via(gx + ex, gy + ey)

    # Add pads as obstacles with RECTANGULAR bounds
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id == exclude_net_id:
            continue
        for pad in pads:
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

            # Via blocking near pads - use rectangular bounds
            # Use int() instead of round() to avoid over-blocking
            if 'F.Cu' in pad.layers or 'B.Cu' in pad.layers:
                via_clear_mm = config.via_size / 2 + config.clearance
                via_expand_x = int((pad.size_x / 2 + via_clear_mm) / config.grid_step)
                via_expand_y = int((pad.size_y / 2 + via_clear_mm) / config.grid_step)
                for ex in range(-via_expand_x, via_expand_x + 1):
                    for ey in range(-via_expand_y, via_expand_y + 1):
                        obstacles.add_blocked_via(gx + ex, gy + ey)

    # Add stub proximity costs
    if unrouted_stubs:
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

    return obstacles


def route_net(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
              unrouted_stubs: Optional[List[Tuple[float, float]]] = None) -> Optional[dict]:
    """Route a single net using the Rust router."""
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    if len(net_segments) < 2:
        print(f"  Net has only {len(net_segments)} segments, need at least 2")
        return None

    groups = find_connected_groups(net_segments)
    if len(groups) < 2:
        print(f"  Net segments are already connected (only {len(groups)} group)")
        return None

    groups.sort(key=len, reverse=True)
    source_segs = groups[0]
    target_segs = groups[1]

    coord = GridCoord(config.grid_step)
    layer_map = {name: idx for idx, name in enumerate(config.layers)}
    layer_names = config.layers

    # Sample source and target points - keep both grid and original coords
    # Format: (gx, gy, layer_idx, original_x, original_y)
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

    if not sources or not targets:
        return None

    # Extract grid-only coords for routing
    sources_grid = [(s[0], s[1], s[2]) for s in sources]
    targets_grid = [(t[0], t[1], t[2]) for t in targets]

    # Build obstacles
    obstacles = build_obstacle_map(pcb_data, config, net_id, unrouted_stubs)

    # Add source and target positions as allowed cells to override BGA zone blocking
    allow_radius = 10
    for gx, gy, _ in sources_grid + targets_grid:
        for dx in range(-allow_radius, allow_radius + 1):
            for dy in range(-allow_radius, allow_radius + 1):
                obstacles.add_allowed_cell(gx + dx, gy + dy)

    router = GridRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight)

    # Smart direction search:
    # 1. Try forward direction with quick check (5000 iterations)
    # 2. If fails, try reverse direction with full iterations
    # 3. If still fails, try forward direction with full iterations
    quick_iterations = 5000
    reversed_path = False
    total_iterations = 0

    path, iterations = router.route_multi(obstacles, sources_grid, targets_grid, quick_iterations)
    total_iterations += iterations

    if path is None:
        print(f"No route found after {iterations} iterations")
        path, iterations = router.route_multi(obstacles, targets_grid, sources_grid, config.max_iterations)
        total_iterations += iterations
        if path is not None:
            reversed_path = True

    if path is None:
        print(f"No route found after {iterations} iterations")
        path, iterations = router.route_multi(obstacles, sources_grid, targets_grid, config.max_iterations)
        total_iterations += iterations

    if path is None:
        print(f"No route found after {iterations} iterations")
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
        'path_length': len(path)
    }


def add_route_to_pcb_data(pcb_data: PCBData, result: dict) -> None:
    """Add routed segments and vias to PCB data for subsequent routes to see."""
    for seg in result['new_segments']:
        pcb_data.segments.append(seg)
    for via in result['new_vias']:
        pcb_data.vias.append(via)


def batch_route(input_file: str, output_file: str, net_names: List[str],
                layers: List[str] = None,
                bga_exclusion_zones: Optional[List[Tuple[float, float, float, float]]] = None) -> Tuple[int, int, float]:
    """
    Route multiple nets using the Rust router.

    Args:
        input_file: Path to input KiCad PCB file
        output_file: Path to output KiCad PCB file
        net_names: List of net names to route
        layers: List of copper layers to route on (must be specified - cannot auto-detect
                which layers are ground planes vs signal layers)
        bga_exclusion_zones: Optional list of BGA exclusion zones (auto-detected if None)

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
    if bga_exclusion_zones is None:
        bga_exclusion_zones = auto_detect_bga_exclusion_zones(pcb_data, margin=0.5)
        if bga_exclusion_zones:
            bga_components = find_components_by_type(pcb_data, 'BGA')
            print(f"Auto-detected {len(bga_exclusion_zones)} BGA exclusion zone(s):")
            for i, (fp, zone) in enumerate(zip(bga_components, bga_exclusion_zones)):
                print(f"  {fp.reference}: ({zone[0]:.1f}, {zone[1]:.1f}) to ({zone[2]:.1f}, {zone[3]:.1f})")
        else:
            print("No BGA components detected - no exclusion zones needed")

    config = GridRouteConfig(
        track_width=0.1,
        clearance=0.1,
        via_size=0.3,
        via_drill=0.2,
        grid_step=0.1,
        via_cost=500,
        layers=layers,
        max_iterations=100000,
        heuristic_weight=1.5,
        bga_exclusion_zones=bga_exclusion_zones,
        stub_proximity_radius=1.0,
        stub_proximity_cost=3.0,
    )

    # Find net IDs
    net_ids = []
    for net_name in net_names:
        net_id = None
        for nid, net in pcb_data.nets.items():
            if net.name == net_name:
                net_id = nid
                break
        if net_id is None:
            print(f"Warning: Net '{net_name}' not found, skipping")
        else:
            net_ids.append((net_name, net_id))

    if not net_ids:
        print("No valid nets to route!")
        return 0, 0, 0.0

    print(f"\nRouting {len(net_ids)} nets...")
    print("=" * 60)

    results = []
    successful = 0
    failed = 0
    total_time = 0
    total_iterations = 0

    remaining_net_ids = [nid for _, nid in net_ids]

    for i, (net_name, net_id) in enumerate(net_ids):
        print(f"\n[{i+1}/{len(net_ids)}] Routing {net_name} (id={net_id})")
        print("-" * 40)

        other_unrouted = [nid for nid in remaining_net_ids if nid != net_id]
        unrouted_stubs = get_stub_endpoints(pcb_data, other_unrouted)

        start_time = time.time()
        result = route_net(pcb_data, net_id, config, unrouted_stubs)
        elapsed = time.time() - start_time
        total_time += elapsed

        if result:
            print(f"  SUCCESS: {len(result['new_segments'])} segments, {len(result['new_vias'])} vias ({elapsed:.2f}s)")
            results.append(result)
            successful += 1
            total_iterations += result['iterations']
            add_route_to_pcb_data(pcb_data, result)
            remaining_net_ids.remove(net_id)
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


if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: python batch_grid_router.py input.kicad_pcb output.kicad_pcb net1 [net2 net3 ...]")
        print("\nExample:")
        print('  python batch_grid_router.py fanout_starting_point.kicad_pcb routed.kicad_pcb "Net-(U2A-DATA_0)" "Net-(U2A-DATA_1)"')
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    net_names = sys.argv[3:]

    batch_route(input_file, output_file, net_names)
