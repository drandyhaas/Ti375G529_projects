"""
Batch PCB Router using Grid-based A* - Routes multiple nets sequentially.

Usage:
    python batch_grid_router.py input.kicad_pcb output.kicad_pcb net1 net2 net3 ...
"""

import sys
import time
from typing import List, Optional, Tuple
from kicad_parser import parse_kicad_pcb, PCBData, Segment, Via
from kicad_writer import generate_segment_sexpr, generate_via_sexpr
from grid_astar_router import (
    GridRouteConfig, GridObstacleMap, GridAStarRouter
)


def find_stub_endpoints(segments: List[Segment]) -> List[Tuple[float, float, str]]:
    """Find endpoint coordinates of stub segments."""
    endpoints = []
    for seg in segments:
        endpoints.append((seg.start_x, seg.start_y, seg.layer))
        endpoints.append((seg.end_x, seg.end_y, seg.layer))
    return endpoints


def find_connected_groups(segments: List[Segment], tolerance: float = 0.01) -> List[List[Segment]]:
    """Find groups of connected segments using union-find."""
    if not segments:
        return []

    # Build adjacency: two segments are connected if they share an endpoint
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
            # Get endpoints
            pts_i = [(si.start_x, si.start_y), (si.end_x, si.end_y)]
            pts_j = [(sj.start_x, sj.start_y), (sj.end_x, sj.end_y)]
            # Check if any endpoints match
            for pi in pts_i:
                for pj in pts_j:
                    if abs(pi[0] - pj[0]) < tolerance and abs(pi[1] - pj[1]) < tolerance:
                        union(i, j)
                        break

    # Group segments by their root
    groups = {}
    for i in range(n):
        root = find(i)
        if root not in groups:
            groups[root] = []
        groups[root].append(segments[i])

    return list(groups.values())


def route_net_grid(pcb_data: PCBData, net_id: int, config: GridRouteConfig) -> Optional[dict]:
    """Route a single net using the grid-based router."""
    # Find segments belonging to this net
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    if len(net_segments) < 2:
        print(f"  Net has only {len(net_segments)} segments, need at least 2")
        return None

    # Build obstacle map excluding this net
    obstacles = GridObstacleMap(pcb_data, config, exclude_net_id=net_id)
    router = GridAStarRouter(obstacles, config)

    # Find disconnected groups of segments
    groups = find_connected_groups(net_segments)

    if len(groups) < 2:
        print(f"  Net segments are already connected (only {len(groups)} group)")
        return None

    # Route between the two largest groups
    groups.sort(key=len, reverse=True)
    source_segs = groups[0]
    target_segs = groups[1]

    # Try routing
    result = router.route_segments_to_segments(
        source_segs, target_segs,
        max_iterations=config.max_iterations
    )

    if not result:
        # Try reverse direction
        result = router.route_segments_to_segments(
            target_segs, source_segs,
            max_iterations=config.max_iterations
        )

    if not result:
        return None

    path, src_pt, tgt_pt = result

    # Simplify and convert to float coordinates
    simplified = router.simplify_path(path)
    float_path = router.path_to_float(simplified)

    # Generate segments and vias
    new_segments = []
    new_vias = []

    for i in range(len(float_path) - 1):
        x1, y1, layer1 = float_path[i]
        x2, y2, layer2 = float_path[i + 1]

        if layer1 != layer2:
            # Layer change - add via at transition point
            via = Via(
                x=x1, y=y1,
                size=config.via_size,
                drill=config.via_drill,
                layers=[layer1, layer2],
                net_id=net_id
            )
            new_vias.append(via)
        else:
            # Same layer - add segment
            seg = Segment(
                start_x=x1, start_y=y1,
                end_x=x2, end_y=y2,
                width=config.track_width,
                layer=layer1,
                net_id=net_id
            )
            new_segments.append(seg)

    return {
        'new_segments': new_segments,
        'new_vias': new_vias,
        'path_length': len(float_path)
    }


def add_route_to_pcb_data(pcb_data: PCBData, result: dict) -> None:
    """Add routed segments and vias to PCB data for subsequent routes to see."""
    for seg in result['new_segments']:
        pcb_data.segments.append(seg)
    for via in result['new_vias']:
        pcb_data.vias.append(via)


def apply_all_routes_to_file(input_path: str, output_path: str,
                              results: List[dict]) -> bool:
    """Apply all routing results to the PCB file."""
    with open(input_path, 'r', encoding='utf-8') as f:
        content = f.read()

    elements = []

    for result in results:
        # Add new segments
        for seg in result['new_segments']:
            elements.append(generate_segment_sexpr(
                (seg.start_x, seg.start_y),
                (seg.end_x, seg.end_y),
                seg.width,
                seg.layer,
                seg.net_id
            ))

        # Add new vias
        for via in result['new_vias']:
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


def batch_route_grid(input_file: str, output_file: str, net_names: List[str]) -> None:
    """Route multiple nets using the grid-based router."""
    print(f"Loading {input_file}...")
    pcb_data = parse_kicad_pcb(input_file)

    # Configure routing
    config = GridRouteConfig(
        track_width=0.1,
        clearance=0.1,
        via_size=0.3,
        via_drill=0.2,
        grid_step=0.1,
        via_cost=500,  # Integer cost (1000 = 1 grid step)
        layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
        max_iterations=100000,
        heuristic_weight=1.5,
        bga_exclusion_zone=(185.9, 93.5, 204.9, 112.5),
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
        return

    print(f"\nRouting {len(net_ids)} nets with GRID router...")
    print("=" * 60)

    results = []
    successful = 0
    failed = 0
    total_time = 0

    for i, (net_name, net_id) in enumerate(net_ids):
        print(f"\n[{i+1}/{len(net_ids)}] Routing {net_name} (id={net_id})")
        print("-" * 40)

        start_time = time.time()
        result = route_net_grid(pcb_data, net_id, config)
        elapsed = time.time() - start_time
        total_time += elapsed

        if result:
            print(f"  SUCCESS: {len(result['new_segments'])} segments, {len(result['new_vias'])} vias ({elapsed:.2f}s)")
            results.append(result)
            successful += 1

            # Update pcb_data with new route
            add_route_to_pcb_data(pcb_data, result)
        else:
            print(f"  FAILED: Could not find route ({elapsed:.2f}s)")
            failed += 1

    print("\n" + "=" * 60)
    print(f"Routing complete: {successful} successful, {failed} failed")
    print(f"Total routing time: {total_time:.2f}s")

    if results:
        print(f"\nWriting output to {output_file}...")
        success = apply_all_routes_to_file(input_file, output_file, results)
        if success:
            print(f"Successfully wrote {output_file}")
        else:
            print("Failed to write output file!")


if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: python batch_grid_router.py input.kicad_pcb output.kicad_pcb net1 [net2 net3 ...]")
        print("\nExample:")
        print('  python batch_grid_router.py fanout_starting_point.kicad_pcb routed_grid.kicad_pcb "Net-(U2A-DATA_0)" "Net-(U2A-DATA_1)"')
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    net_names = sys.argv[3:]

    batch_route_grid(input_file, output_file, net_names)
