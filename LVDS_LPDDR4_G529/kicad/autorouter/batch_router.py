"""
Batch PCB Router - Routes multiple nets sequentially, with each new route
avoiding previously routed tracks.

Usage:
    python batch_router.py input.kicad_pcb output.kicad_pcb net1 net2 net3 ...
"""

import sys
from typing import List, Optional
from kicad_parser import parse_kicad_pcb, PCBData, Segment, Via
from astar_router import (
    RouteConfig, RouteResult, route_net,
    path_to_segments, path_to_vias,
    find_stub_intersection_and_trim
)


def add_route_to_pcb_data(pcb_data: PCBData, result: RouteResult) -> None:
    """
    Add routed segments and vias to the PCB data structure in memory.
    This allows subsequent routes to see these as obstacles.
    """
    # Add new segments
    for seg in result.new_segments:
        pcb_data.segments.append(seg)

    # Add new vias
    for via in result.new_vias:
        pcb_data.vias.append(via)

    # Handle segment removals (from stub trimming)
    for seg_to_remove in result.segments_to_remove:
        # Find and remove matching segment
        for i, seg in enumerate(pcb_data.segments):
            if (abs(seg.start_x - seg_to_remove.start_x) < 0.001 and
                abs(seg.start_y - seg_to_remove.start_y) < 0.001 and
                abs(seg.end_x - seg_to_remove.end_x) < 0.001 and
                abs(seg.end_y - seg_to_remove.end_y) < 0.001 and
                seg.layer == seg_to_remove.layer and
                seg.net_id == seg_to_remove.net_id):
                pcb_data.segments.pop(i)
                break

    # Handle segment shortenings
    for seg, new_sx, new_sy, new_ex, new_ey in result.segments_to_shorten:
        # Find and update matching segment
        for i, s in enumerate(pcb_data.segments):
            if (abs(s.start_x - seg.start_x) < 0.001 and
                abs(s.start_y - seg.start_y) < 0.001 and
                abs(s.end_x - seg.end_x) < 0.001 and
                abs(s.end_y - seg.end_y) < 0.001 and
                s.layer == seg.layer and
                s.net_id == seg.net_id):
                # Update coordinates
                s.start_x = new_sx
                s.start_y = new_sy
                s.end_x = new_ex
                s.end_y = new_ey
                break


def apply_all_routes_to_file(input_path: str, output_path: str,
                              results: List[RouteResult]) -> bool:
    """
    Apply all routing results to the PCB file at once.
    """
    import re
    from kicad_writer import generate_segment_sexpr, generate_via_sexpr

    with open(input_path, 'r', encoding='utf-8') as f:
        content = f.read()

    elements = []

    for result in results:
        # Handle segment removals
        for seg in result.segments_to_remove:
            # Use original string coordinates if available, otherwise format floats
            sx_str = seg.start_x_str if seg.start_x_str else f"{seg.start_x:.6f}"
            sy_str = seg.start_y_str if seg.start_y_str else f"{seg.start_y:.6f}"
            ex_str = seg.end_x_str if seg.end_x_str else f"{seg.end_x:.6f}"
            ey_str = seg.end_y_str if seg.end_y_str else f"{seg.end_y:.6f}"
            pattern = (
                rf'\(segment\s+'
                rf'\(start\s+{re.escape(sx_str)}\s+{re.escape(sy_str)}\)\s+'
                rf'\(end\s+{re.escape(ex_str)}\s+{re.escape(ey_str)}\)\s+'
                rf'\(width\s+[\d.]+\)\s+'
                rf'\(layer\s+"{re.escape(seg.layer)}"\)\s+'
                rf'\(net\s+{seg.net_id}\)\s+'
                rf'\(uuid\s+"[^"]+"\)\s*\)\s*'
            )
            content = re.sub(pattern, '', content)

        # Handle segment shortenings
        for seg, new_sx, new_sy, new_ex, new_ey in result.segments_to_shorten:
            # Use original string coordinates if available
            sx_str = seg.start_x_str if seg.start_x_str else f"{seg.start_x:.6f}"
            sy_str = seg.start_y_str if seg.start_y_str else f"{seg.start_y:.6f}"
            ex_str = seg.end_x_str if seg.end_x_str else f"{seg.end_x:.6f}"
            ey_str = seg.end_y_str if seg.end_y_str else f"{seg.end_y:.6f}"
            pattern = (
                rf'\(segment\s+'
                rf'\(start\s+{re.escape(sx_str)}\s+{re.escape(sy_str)}\)\s+'
                rf'\(end\s+{re.escape(ex_str)}\s+{re.escape(ey_str)}\)\s+'
                rf'\(width\s+[\d.]+\)\s+'
                rf'\(layer\s+"{re.escape(seg.layer)}"\)\s+'
                rf'\(net\s+{seg.net_id}\)\s+'
                rf'\(uuid\s+"[^"]+"\)\s*\)\s*'
            )
            content = re.sub(pattern, '', content)

            # Add shortened version
            elements.append(generate_segment_sexpr(
                (new_sx, new_sy),
                (new_ex, new_ey),
                seg.width,
                seg.layer,
                seg.net_id
            ))

        # Add new segments
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


def batch_route(input_file: str, output_file: str, net_names: List[str]) -> None:
    """
    Route multiple nets sequentially, each avoiding previously routed tracks.
    """
    print(f"Loading {input_file}...")
    pcb_data = parse_kicad_pcb(input_file)

    # Configure routing
    config = RouteConfig(
        track_width=0.1,
        clearance=0.1,
        via_size=0.4,
        via_drill=0.2,
        grid_step=0.1,
        via_cost=0.5,
        layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
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

    print(f"\nRouting {len(net_ids)} nets...")
    print("=" * 60)

    results = []
    successful = 0
    failed = 0

    for i, (net_name, net_id) in enumerate(net_ids):
        print(f"\n[{i+1}/{len(net_ids)}] Routing {net_name} (id={net_id})")
        print("-" * 40)

        # Route this net (pcb_data now includes previously routed tracks as obstacles)
        result = route_net(pcb_data, net_id, config)

        if result:
            print(f"  SUCCESS: {len(result.new_segments)} segments, {len(result.new_vias)} vias")
            results.append(result)
            successful += 1

            # Update pcb_data with new route so next iteration sees it as obstacle
            add_route_to_pcb_data(pcb_data, result)
        else:
            print(f"  FAILED: Could not find route")
            failed += 1

    print("\n" + "=" * 60)
    print(f"Routing complete: {successful} successful, {failed} failed")

    if results:
        print(f"\nWriting output to {output_file}...")
        success = apply_all_routes_to_file(input_file, output_file, results)
        if success:
            print(f"Successfully wrote {output_file}")
        else:
            print("Failed to write output file!")


if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: python batch_router.py input.kicad_pcb output.kicad_pcb net1 [net2 net3 ...]")
        print("\nExample:")
        print('  python batch_router.py fanout_starting_point.kicad_pcb routed.kicad_pcb "Net-(U2A-DATA_0)" "Net-(U2A-DATA_1)"')
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    net_names = sys.argv[3:]

    batch_route(input_file, output_file, net_names)
