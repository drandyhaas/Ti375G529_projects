"""
Benchmark script to compare Python vs Rust router performance.
"""

import sys
import time
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import parse_kicad_pcb
from grid_astar_router import GridRouteConfig, GridObstacleMap as PyObstacleMap, GridAStarRouter, GridCoord
from batch_grid_router import find_connected_groups

# Try to import Rust router
try:
    from grid_router import GridObstacleMap as RustObstacleMap, GridRouter as RustRouter
    HAS_RUST = True
    print("Rust router available")
except ImportError:
    HAS_RUST = False
    print("Rust router not available - build with 'maturin develop --release'")


def build_rust_obstacles(pcb_data, config, exclude_net_id, unrouted_stubs=None):
    """Build Rust obstacle map from PCB data."""
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)
    layer_map = {name: idx for idx, name in enumerate(config.layers)}

    obstacles = RustObstacleMap(num_layers)

    # Set BGA zone
    if config.bga_exclusion_zone:
        min_x, min_y, max_x, max_y = config.bga_exclusion_zone
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

        # Rasterize segment
        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

        # Simple line rasterization with expansion
        dx = abs(gx2 - gx1)
        dy = abs(gy2 - gy1)
        sx = 1 if gx1 < gx2 else -1
        sy = 1 if gy1 < gy2 else -1

        gx, gy = gx1, gy1
        steps = max(dx, dy) + 1
        for _ in range(steps):
            # Add blocked cells with expansion
            for ex in range(-expansion_grid, expansion_grid + 1):
                for ey in range(-expansion_grid, expansion_grid + 1):
                    obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
            # Add via blocking
            for ex in range(-via_block_grid, via_block_grid + 1):
                for ey in range(-via_block_grid, via_block_grid + 1):
                    if ex*ex + ey*ey <= via_block_grid * via_block_grid:
                        obstacles.add_blocked_via(gx + ex, gy + ey)

            if gx == gx2 and gy == gy2:
                break
            e2 = 2 * (dx - dy) if dx > dy else 2 * (dy - dx)
            if dx > dy:
                if e2 > -dy:
                    gx += sx
                if e2 < dx:
                    gy += sy
            else:
                if e2 > -dx:
                    gy += sy
                if e2 < dy:
                    gx += sx

    # Add vias as obstacles
    via_expansion_grid = max(1, coord.to_grid_dist(config.via_size / 2 + config.clearance))
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

    # Add pads as obstacles
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id == exclude_net_id:
            continue
        for pad in pads:
            radius = max(pad.size_x, pad.size_y) / 2
            radius_grid = max(1, coord.to_grid_dist(radius + config.clearance))
            gx, gy = coord.to_grid(pad.global_x, pad.global_y)
            for ex in range(-radius_grid, radius_grid + 1):
                for ey in range(-radius_grid, radius_grid + 1):
                    if ex*ex + ey*ey <= radius_grid * radius_grid:
                        for layer in pad.layers:
                            layer_idx = layer_map.get(layer)
                            if layer_idx is not None:
                                obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)

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


def benchmark_single_net(pcb_data, net_id, config, use_rust=False):
    """Benchmark routing a single net."""
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    if len(net_segments) < 2:
        return None, 0, 0

    groups = find_connected_groups(net_segments)
    if len(groups) < 2:
        return None, 0, 0

    groups.sort(key=len, reverse=True)
    source_segs = groups[0]
    target_segs = groups[1]

    coord = GridCoord(config.grid_step)
    layer_map = {name: idx for idx, name in enumerate(config.layers)}

    # Sample source and target points
    sources = []
    for seg in source_segs:
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
        sources.append((gx1, gy1, layer_idx))
        if (gx1, gy1) != (gx2, gy2):
            sources.append((gx2, gy2, layer_idx))

    targets = []
    for seg in target_segs:
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
        targets.append((gx1, gy1, layer_idx))
        if (gx1, gy1) != (gx2, gy2):
            targets.append((gx2, gy2, layer_idx))

    if not sources or not targets:
        return None, 0, 0

    start_time = time.perf_counter()

    if use_rust and HAS_RUST:
        obstacles = build_rust_obstacles(pcb_data, config, net_id)
        router = RustRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight)
        path, iterations = router.route_multi(obstacles, sources, targets, config.max_iterations)
    else:
        obstacles = PyObstacleMap(pcb_data, config, exclude_net_id=net_id)
        router = GridAStarRouter(obstacles, config)
        result = router.route_segments_to_segments(source_segs, target_segs, config.max_iterations)
        if result:
            path = [(s.gx, s.gy, s.layer_idx) for s in result[0]]
            iterations = len(path)  # Approximate
        else:
            path = None
            iterations = config.max_iterations

    elapsed = time.perf_counter() - start_time

    return path, iterations, elapsed


def main():
    if len(sys.argv) < 2:
        print("Usage: python benchmark.py <pcb_file> [net_name]")
        sys.exit(1)

    pcb_file = sys.argv[1]
    net_name = sys.argv[2] if len(sys.argv) > 2 else "Net-(U2A-DATA_0)"

    print(f"Loading {pcb_file}...")
    pcb_data = parse_kicad_pcb(pcb_file)

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
        max_iterations=100000,
        heuristic_weight=1.5,
        bga_exclusion_zone=(185.9, 93.5, 204.9, 112.5),
        stub_proximity_radius=1.0,
        stub_proximity_cost=3.0,
    )

    print(f"\nBenchmarking {net_name}...")
    print("=" * 60)

    # Python benchmark
    print("\nPython router:")
    path_py, iter_py, time_py = benchmark_single_net(pcb_data, net_id, config, use_rust=False)
    if path_py:
        print(f"  Found path with {len(path_py)} points")
    else:
        print(f"  No path found")
    print(f"  Time: {time_py*1000:.1f}ms")

    # Rust benchmark (if available)
    if HAS_RUST:
        print("\nRust router:")
        path_rust, iter_rust, time_rust = benchmark_single_net(pcb_data, net_id, config, use_rust=True)
        if path_rust:
            print(f"  Found path with {len(path_rust)} points in {iter_rust} iterations")
        else:
            print(f"  No path found after {iter_rust} iterations")
        print(f"  Time: {time_rust*1000:.1f}ms")

        if time_py > 0 and time_rust > 0:
            speedup = time_py / time_rust
            print(f"\nSpeedup: {speedup:.1f}x")
    else:
        print("\nRust router not available")
        print("Build with: cd rust_router && maturin develop --release")


if __name__ == "__main__":
    main()
