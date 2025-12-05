"""
Full 32-net benchmark comparing Python vs Rust router.
"""

import sys
import time
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import parse_kicad_pcb, Segment, Via
from grid_astar_router import GridRouteConfig, GridObstacleMap as PyObstacleMap, GridAStarRouter, GridCoord
from batch_grid_router import find_connected_groups, get_stub_endpoints, add_route_to_pcb_data
from kicad_writer import generate_segment_sexpr, generate_via_sexpr

# Try to import Rust router
try:
    import grid_router
    from grid_router import GridObstacleMap as RustObstacleMap, GridRouter as RustRouter
    HAS_RUST = True
    version = getattr(grid_router, '__version__', 'unknown')
    print(f"Rust router v{version} available")
except ImportError:
    HAS_RUST = False
    print("Rust router not available")
    sys.exit(1)


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


def route_net_rust(pcb_data, net_id, config, unrouted_stubs=None):
    """Route a single net using Rust router."""
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    if len(net_segments) < 2:
        return None

    groups = find_connected_groups(net_segments)
    if len(groups) < 2:
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
    obstacles = build_rust_obstacles(pcb_data, config, net_id, unrouted_stubs)

    # Add source and target positions (and surrounding area) as allowed cells
    # to override BGA zone blocking - needed because stubs may be inside BGA zone
    # Use radius of 10 grid cells to allow routing to reach stubs inside BGA
    allow_radius = 10
    for gx, gy, _ in sources_grid + targets_grid:
        for dx in range(-allow_radius, allow_radius + 1):
            for dy in range(-allow_radius, allow_radius + 1):
                obstacles.add_allowed_cell(gx + dx, gy + dy)

    router = RustRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight)

    # Smart direction search
    quick_iterations = 5000
    reversed_path = False

    path, iterations = router.route_multi(obstacles, sources_grid, targets_grid, quick_iterations)

    if path is None:
        path, iterations = router.route_multi(obstacles, targets_grid, sources_grid, config.max_iterations)
        if path is not None:
            reversed_path = True

    if path is None:
        path, iterations = router.route_multi(obstacles, sources_grid, targets_grid, config.max_iterations)

    if path is None:
        return None

    # If path was found in reverse direction, swap sources/targets for connection logic
    if reversed_path:
        sources, targets = targets, sources

    # Find which source/target the path actually connects to
    path_start = path[0]  # (gx, gy, layer)
    path_end = path[-1]

    # Find the original coordinates for path start and end
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
            # Via
            via = Via(
                x=x1, y=y1,
                size=config.via_size,
                drill=config.via_drill,
                layers=[layer_names[layer1], layer_names[layer2]],
                net_id=net_id
            )
            new_vias.append(via)
        else:
            # Segment
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
        'iterations': iterations,
        'path_length': len(path)
    }


def batch_route_rust(input_file, output_file, net_names):
    """Route multiple nets using Rust router."""
    print(f"Loading {input_file}...")
    pcb_data = parse_kicad_pcb(input_file)

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

    # Find net IDs
    net_ids = []
    for net_name in net_names:
        net_id = None
        for nid, net in pcb_data.nets.items():
            if net.name == net_name:
                net_id = nid
                break
        if net_id is None:
            print(f"Warning: Net '{net_name}' not found")
        else:
            net_ids.append((net_name, net_id))

    if not net_ids:
        print("No valid nets!")
        return

    print(f"\nRouting {len(net_ids)} nets with RUST router...")
    print("=" * 60)

    results = []
    successful = 0
    failed = 0
    total_time = 0
    total_iterations = 0

    remaining_net_ids = [nid for _, nid in net_ids]

    for i, (net_name, net_id) in enumerate(net_ids):
        print(f"\n[{i+1}/{len(net_ids)}] Routing {net_name} (id={net_id})")

        other_unrouted = [nid for nid in remaining_net_ids if nid != net_id]
        unrouted_stubs = get_stub_endpoints(pcb_data, other_unrouted)

        start_time = time.time()
        result = route_net_rust(pcb_data, net_id, config, unrouted_stubs)
        elapsed = time.time() - start_time
        total_time += elapsed

        if result:
            print(f"  SUCCESS: {len(result['new_segments'])} segments, {len(result['new_vias'])} vias, {result['iterations']} iter ({elapsed*1000:.0f}ms)")
            results.append(result)
            successful += 1
            total_iterations += result['iterations']
            add_route_to_pcb_data(pcb_data, result)
            remaining_net_ids.remove(net_id)
        else:
            print(f"  FAILED ({elapsed*1000:.0f}ms)")
            failed += 1

    print("\n" + "=" * 60)
    print(f"Routing complete: {successful} successful, {failed} failed")
    print(f"Total time: {total_time:.2f}s")
    print(f"Total iterations: {total_iterations}")

    # Write output
    if results:
        print(f"\nWriting {output_file}...")
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

        print(f"Written to {output_file}")

    return successful, failed, total_time


# Inside-out net ordering for BGA breakout
NET_ORDER = [
    "Net-(U2A-DATA_15)", "Net-(U2A-DATA_16)", "Net-(U2A-DATA_22)", "Net-(U2A-DATA_21)",
    "Net-(U2A-DATA_23)", "Net-(U2A-DATA_14)", "Net-(U2A-DATA_17)", "Net-(U2A-DATA_20)",
    "Net-(U2A-DATA_24)", "Net-(U2A-DATA_13)", "Net-(U2A-DATA_18)", "Net-(U2A-DATA_19)",
    "Net-(U2A-DATA_25)", "Net-(U2A-DATA_26)", "Net-(U2A-DATA_27)", "Net-(U2A-DATA_28)",
    "Net-(U2A-DATA_29)", "Net-(U2A-DATA_30)", "Net-(U2A-DATA_31)", "Net-(U2A-DATA_8)",
    "Net-(U2A-DATA_9)", "Net-(U2A-DATA_10)", "Net-(U2A-DATA_11)", "Net-(U2A-DATA_12)",
    "Net-(U2A-DATA_7)", "Net-(U2A-DATA_6)", "Net-(U2A-DATA_5)", "Net-(U2A-DATA_4)",
    "Net-(U2A-DATA_3)", "Net-(U2A-DATA_2)", "Net-(U2A-DATA_1)", "Net-(U2A-DATA_0)"
]


if __name__ == "__main__":
    input_file = "../fanout_starting_point.kicad_pcb"
    output_file = "../routed_rust.kicad_pcb"

    batch_route_rust(input_file, output_file, NET_ORDER)
