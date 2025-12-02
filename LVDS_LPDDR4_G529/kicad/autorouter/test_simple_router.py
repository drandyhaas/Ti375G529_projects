#!/usr/bin/env python3
"""
Test the simple router on actual PCB data and output to KiCad.
Supports multi-layer routing with vias when tracks need to cross.

Usage:
    python test_simple_router.py [options] <net_pattern> [net_pattern...]

Options:
    --layers <layer1,layer2,...>  Specify allowed routing layers (default: F.Cu,B.Cu)
                                  Example: --layers F.Cu,B.Cu,In2.Cu,In7.Cu
"""

import sys
import math
import argparse
from pathlib import Path
from simple_router import route_single_net, route_bidirectional, simplify_path, simplify_path_safe, Obstacle, optimize_simplified_path, snap_to_45_degrees, verify_path, enforce_45_degree_path, remove_kinks, try_multi_zone_centering, SpatialIndex
from kicad_parser import parse_kicad_pcb, get_nets_to_route
from kicad_writer import add_tracks_and_vias_to_pcb

# Via parameters
VIA_SIZE = 0.6  # Via pad size
VIA_DRILL = 0.3  # Via drill size


def build_obstacles(pcb_data, net, all_routed_tracks, layer="F.Cu"):
    """Build obstacle list for routing on a specific layer."""
    obstacles = []
    net_pad_positions = set()
    for pad in net.pads:
        px = getattr(pad, 'global_x', 0)
        py = getattr(pad, 'global_y', 0)
        net_pad_positions.add((round(px, 3), round(py, 3)))

    # Add pad obstacles - using full rectangular shape
    for fp in pcb_data.footprints.values():
        for pad in fp.pads:
            px = getattr(pad, 'global_x', 0)
            py = getattr(pad, 'global_y', 0)

            # Skip this net's own pads
            if (round(px, 3), round(py, 3)) in net_pad_positions:
                continue

            # Get pad dimensions
            width = getattr(pad, 'size_x', 0.6)
            height = getattr(pad, 'size_y', 0.6)
            shape = getattr(pad, 'shape', 'rect')

            # Get footprint rotation (pads inherit component rotation)
            rotation = getattr(fp, 'rotation', 0)

            # Set corner radius based on shape
            if shape == 'circle':
                corner_radius = min(width, height) / 2
            elif shape == 'roundrect':
                corner_radius = min(width, height) * 0.25
            else:
                corner_radius = 0

            obstacles.append(Obstacle(px, py, width, height, rotation, corner_radius))

    # Add via obstacles (except this net's vias) - vias are circular
    for via in pcb_data.vias:
        if via.net_id != net.net_id:
            obstacles.append(Obstacle(via.x, via.y, via.size, via.size, 0, via.size/2))

    # Add existing track segments as obstacles (only on same layer)
    for track in pcb_data.segments:
        if track.net_id != net.net_id and track.layer == layer:
            start_x, start_y = track.start_x, track.start_y
            end_x, end_y = track.end_x, track.end_y
            track_width = track.width

            dx = end_x - start_x
            dy = end_y - start_y
            length = (dx*dx + dy*dy)**0.5
            if length > 0:
                track_angle = math.degrees(math.atan2(dy, dx))
                num_points = max(2, int(length / 0.2))
                for j in range(num_points + 1):
                    t = j / num_points
                    px = start_x + t * dx
                    py = start_y + t * dy
                    obstacles.append(Obstacle(px, py, 0.25, track_width, track_angle, track_width/2))

    # Add already routed tracks as obstacles (only on same layer)
    for track in all_routed_tracks:
        if track['layer'] != layer:
            continue
        start_x, start_y = track['start']
        end_x, end_y = track['end']
        track_width = track['width']
        dx = end_x - start_x
        dy = end_y - start_y
        length = (dx*dx + dy*dy)**0.5
        if length > 0:
            track_angle = math.degrees(math.atan2(dy, dx))
            num_points = max(2, int(length / 0.2))
            for j in range(num_points + 1):
                t = j / num_points
                px = start_x + t * dx
                py = start_y + t * dy
                obstacles.append(Obstacle(px, py, 0.25, track_width, track_angle, track_width/2))

    return obstacles


def route_path(source, sink, obstacles, step_size=0.1, routing_clearance=0.05, max_time=1.0):
    """Try to route a path between source and sink."""
    path = route_bidirectional(
        source=source,
        sink=sink,
        obstacles=obstacles,
        step_size=step_size,
        clearance=routing_clearance,
        max_time=max_time
    )
    return path


def process_path(path, obstacles, routing_clearance):
    """Process a raw path: simplify, enforce 45-deg, optimize, remove kinks, center."""
    # Step 1: Simplify using collision-aware simplification
    simplified = simplify_path_safe(path, obstacles, routing_clearance, tolerance=0.1)

    # Step 2: Enforce exact 45-degree angles
    path_45 = enforce_45_degree_path(simplified, obstacles, routing_clearance)

    # Step 3: Optimize the path while maintaining 45-degree constraints
    optimized = optimize_simplified_path(path_45, obstacles, routing_clearance,
                                          iterations=50, initial_step=0.1)

    # Step 4: Remove any kinks/zigzags
    final_path = remove_kinks(optimized, obstacles, routing_clearance)

    # Step 5: Multi-zone centering
    index = SpatialIndex(obstacles)
    final_path = try_multi_zone_centering(final_path, obstacles, index, routing_clearance,
                                           length_weight=1.0, clearance_weight=1000.0)

    return final_path


def find_crossing_point(source, sink, blocking_tracks, layer):
    """
    Find where the route would need to cross a blocking track.
    Returns (crossing_x, crossing_y, track_being_crossed) or None.

    Only considers tracks that are within the Y corridor between source and sink,
    plus a small margin to account for tracks that might be just outside.
    """
    mid_x = (source[0] + sink[0]) / 2
    mid_y = (source[1] + sink[1]) / 2

    # Define the Y corridor - tracks must be within this range to be relevant
    route_min_y = min(source[1], sink[1])
    route_max_y = max(source[1], sink[1])
    # Add margin to catch tracks that are slightly outside the direct corridor
    y_margin = 1.0  # 1mm margin
    corridor_min_y = route_min_y - y_margin
    corridor_max_y = route_max_y + y_margin

    best_track = None
    best_dist = float('inf')
    best_cross_point = None

    for track in blocking_tracks:
        if track['layer'] != layer:
            continue

        start_x, start_y = track['start']
        end_x, end_y = track['end']

        # Check if track is between source and sink in X
        track_min_x = min(start_x, end_x)
        track_max_x = max(start_x, end_x)
        route_min_x = min(source[0], sink[0])
        route_max_x = max(source[0], sink[0])

        # Check for X overlap
        if track_max_x < route_min_x or track_min_x > route_max_x:
            continue

        # Check for Y corridor overlap - track must be in the Y corridor
        track_min_y = min(start_y, end_y)
        track_max_y = max(start_y, end_y)
        if track_max_y < corridor_min_y or track_min_y > corridor_max_y:
            continue

        # Find crossing point - where our route's Y would intersect this track's X range
        # Use interpolation along the track
        cross_x = max(track_min_x, min(track_max_x, mid_x))

        if abs(end_x - start_x) > 0.001:
            t = (cross_x - start_x) / (end_x - start_x)
            t = max(0, min(1, t))
            cross_y = start_y + t * (end_y - start_y)
        else:
            cross_y = (start_y + end_y) / 2

        dist = abs(cross_x - mid_x)
        if dist < best_dist:
            best_dist = dist
            best_track = track
            best_cross_point = (cross_x, cross_y)

    if best_track:
        return best_cross_point, best_track
    return None, None


def find_via_at_pad(pcb_data, pad, net_id, tolerance=0.5):
    """Check if there's a via at or near this pad on the same net."""
    pad_x = getattr(pad, 'global_x', 0)
    pad_y = getattr(pad, 'global_y', 0)

    for via in pcb_data.vias:
        if via.net_id == net_id:
            dist = math.sqrt((via.x - pad_x)**2 + (via.y - pad_y)**2)
            if dist < tolerance:
                return via
    return None


def route_net(pcb_data, net, all_routed_tracks, all_vias, allowed_layers=None):
    """Route a single net and return the track segments and vias.

    Args:
        pcb_data: Parsed PCB data
        net: Net to route
        all_routed_tracks: Previously routed tracks (obstacles)
        all_vias: Previously placed vias
        allowed_layers: List of allowed routing layers (default: ["F.Cu", "B.Cu"])
    """
    if allowed_layers is None:
        allowed_layers = ["F.Cu", "B.Cu"]

    print(f"\nRouting net: {net.name}")
    print(f"  Pads: {len(net.pads)}")
    print(f"  Allowed layers: {', '.join(allowed_layers)}")

    first_pad = net.pads[0]
    second_pad = net.pads[1]

    source_x = getattr(first_pad, 'global_x', 0)
    source_y = getattr(first_pad, 'global_y', 0)
    sink_x = getattr(second_pad, 'global_x', 0)
    sink_y = getattr(second_pad, 'global_y', 0)

    print(f"  Source: ({source_x:.2f}, {source_y:.2f}) - {first_pad.component_ref}.{first_pad.pad_number}")
    print(f"  Sink: ({sink_x:.2f}, {sink_y:.2f}) - {second_pad.component_ref}.{second_pad.pad_number}")

    # Check if either pad already has a via (e.g., BGA escape via)
    source_via = find_via_at_pad(pcb_data, first_pad, net.net_id)
    sink_via = find_via_at_pad(pcb_data, second_pad, net.net_id)

    if source_via:
        print(f"  Source has existing via at ({source_via.x:.2f}, {source_via.y:.2f})")
    if sink_via:
        print(f"  Sink has existing via at ({sink_via.x:.2f}, {sink_via.y:.2f})")

    source = (source_x, source_y)
    sink = (sink_x, sink_y)

    track_width = 0.1
    routing_clearance = 0.05

    # Use first layer as primary, others as alternates
    primary_layer = allowed_layers[0]
    alternate_layers = allowed_layers[1:] if len(allowed_layers) > 1 else []

    # Via layers should span all allowed layers for through-hole, or just the layers we're using
    via_layers = [allowed_layers[0], allowed_layers[-1]] if len(allowed_layers) > 1 else allowed_layers

    # Try routing on primary layer first
    obstacles = build_obstacles(pcb_data, net, all_routed_tracks, primary_layer)
    print(f"  Total obstacles on {primary_layer}: {len(obstacles)}")

    print(f"\nRouting on {primary_layer}...")
    path = route_path(source, sink, obstacles)

    if path:
        print(f"Success! Path has {len(path)} points")
        final_path = process_path(path, obstacles, routing_clearance)
        print(f"Final path: {len(final_path)} points")

        # Calculate path length
        total_length = sum(
            math.sqrt((final_path[i][0] - final_path[i-1][0])**2 +
                     (final_path[i][1] - final_path[i-1][1])**2)
            for i in range(1, len(final_path))
        )
        print(f"Path length: {total_length:.2f} mm")

        # Convert to track segments
        tracks = []
        for i in range(1, len(final_path)):
            tracks.append({
                'start': final_path[i-1],
                'end': final_path[i],
                'width': track_width,
                'layer': primary_layer,
                'net_id': net.net_id
            })
        return tracks, []

    # Primary layer failed - first try other layers directly (no vias needed if source has via)
    print(f"\nPrimary layer routing failed.")

    # If source has a via, try routing entirely on alternate layers first
    if source_via and alternate_layers:
        for alt_layer in alternate_layers:
            print(f"  Trying direct route on {alt_layer}...")
            alt_obstacles = build_obstacles(pcb_data, net, all_routed_tracks, alt_layer)
            alt_path = route_path(source, sink, alt_obstacles, max_time=0.5)
            if alt_path:
                print(f"  Success on {alt_layer}!")
                final_path = process_path(alt_path, alt_obstacles, routing_clearance)
                tracks = []
                for i in range(1, len(final_path)):
                    tracks.append({
                        'start': final_path[i-1],
                        'end': final_path[i],
                        'width': track_width,
                        'layer': alt_layer,
                        'net_id': net.net_id
                    })
                total_length = sum(
                    math.sqrt((t['end'][0] - t['start'][0])**2 + (t['end'][1] - t['start'][1])**2)
                    for t in tracks
                )
                print(f"  Path length: {total_length:.2f} mm (no new vias)")
                return tracks, []

    print("  Trying via crossing...")

    # Find where we need to cross - include both session tracks AND existing PCB tracks
    # Convert pcb_data.segments to the same format as all_routed_tracks
    all_blocking_tracks = list(all_routed_tracks)  # Copy our session tracks
    for seg in pcb_data.segments:
        if seg.net_id != net.net_id:  # Don't consider our own net's tracks as blockers
            all_blocking_tracks.append({
                'start': (seg.start_x, seg.start_y),
                'end': (seg.end_x, seg.end_y),
                'layer': seg.layer,
                'net_id': seg.net_id
            })

    cross_point, blocking_track = find_crossing_point(source, sink, all_blocking_tracks, primary_layer)

    if not cross_point:
        print("  Could not identify crossing point - no viable route")
        return [], []

    print(f"  Need to cross track at ({cross_point[0]:.2f}, {cross_point[1]:.2f})")

    # Calculate via positions - place vias on either side of the blocking track
    via_margin = VIA_SIZE + 0.2  # Via size plus clearance

    # Determine track direction
    track_dx = blocking_track['end'][0] - blocking_track['start'][0]
    track_dy = blocking_track['end'][1] - blocking_track['start'][1]
    track_len = math.sqrt(track_dx**2 + track_dy**2)

    if track_len > 0:
        # Perpendicular to track direction
        perp_x = -track_dy / track_len
        perp_y = track_dx / track_len
    else:
        perp_x, perp_y = 0, 1

    # Place vias perpendicular to the blocking track
    via1_x = cross_point[0] - perp_x * via_margin
    via1_y = cross_point[1] - perp_y * via_margin
    via2_x = cross_point[0] + perp_x * via_margin
    via2_y = cross_point[1] + perp_y * via_margin

    # Make sure via1 is closer to source
    dist1_to_source = (via1_x - source[0])**2 + (via1_y - source[1])**2
    dist2_to_source = (via2_x - source[0])**2 + (via2_y - source[1])**2
    if dist2_to_source < dist1_to_source:
        via1_x, via2_x = via2_x, via1_x
        via1_y, via2_y = via2_y, via1_y

    all_tracks = []
    new_vias = []

    # Check if source already has a via - can start on alternate layer
    if source_via and alternate_layers:
        # Try each alternate layer until one works
        route_success = False
        for alternate_layer in alternate_layers:
            print(f"  Trying existing source via - routing on {alternate_layer}")

            # Only need one via - place it on the sink side of the crossing
            new_via_x = via2_x
            new_via_y = via2_y

            # Segment 1: Source via to new via on alternate layer
            print(f"\n  Routing segment 1: Source via to new via on {alternate_layer}...")
            obstacles1 = build_obstacles(pcb_data, net, all_routed_tracks, alternate_layer)
            source_via_pos = (source_via.x, source_via.y)
            path1 = route_path(source_via_pos, (new_via_x, new_via_y), obstacles1, max_time=0.5)

            if not path1:
                print(f"    Failed to route on {alternate_layer}, trying next layer...")
                continue

            # Found a working layer for segment 1
            print(f"  Single new via at ({new_via_x:.2f}, {new_via_y:.2f})")
            temp_tracks = []
            temp_vias = []

            final_path1 = process_path(path1, obstacles1, routing_clearance)
            print(f"    Segment 1: {len(final_path1)} points")

            for i in range(1, len(final_path1)):
                temp_tracks.append({
                    'start': final_path1[i-1],
                    'end': final_path1[i],
                    'width': track_width,
                    'layer': alternate_layer,
                    'net_id': net.net_id
                })

            # Add the single new via
            temp_vias.append({
                'x': new_via_x,
                'y': new_via_y,
                'size': VIA_SIZE,
                'drill': VIA_DRILL,
                'layers': via_layers,
                'net_id': net.net_id
            })

            # Segment 2: New via to sink on primary layer
            print(f"\n  Routing segment 2: New via to Sink on {primary_layer}...")
            obstacles2 = build_obstacles(pcb_data, net, all_routed_tracks + temp_tracks, primary_layer)
            path2 = route_path((new_via_x, new_via_y), sink, obstacles2, max_time=0.5)

            if not path2:
                print(f"    Failed to route to sink on {primary_layer}, trying next layer...")
                continue

            final_path2 = process_path(path2, obstacles2, routing_clearance)
            print(f"    Segment 2: {len(final_path2)} points")

            for i in range(1, len(final_path2)):
                temp_tracks.append({
                    'start': final_path2[i-1],
                    'end': final_path2[i],
                    'width': track_width,
                    'layer': primary_layer,
                    'net_id': net.net_id
                })

            # Success! Copy temp to final
            all_tracks.extend(temp_tracks)
            new_vias.extend(temp_vias)
            route_success = True
            break

        if not route_success:
            print("  All alternate layers failed, falling back to two-via approach")
            source_via = None  # Force two-via approach

    # Standard two-via approach if no source via or fallback needed
    if not source_via and alternate_layers:
        print(f"  Via 1 at ({via1_x:.2f}, {via1_y:.2f})")
        print(f"  Via 2 at ({via2_x:.2f}, {via2_y:.2f})")

        # Route segment 1 first (on primary layer) - same for all attempts
        print(f"\n  Routing segment 1: Source to Via1 on {primary_layer}...")
        obstacles1 = build_obstacles(pcb_data, net, all_routed_tracks, primary_layer)
        path1 = route_path(source, (via1_x, via1_y), obstacles1, max_time=0.5)

        if not path1:
            print("    Failed to route to Via1")
            return [], []

        final_path1 = process_path(path1, obstacles1, routing_clearance)
        print(f"    Segment 1: {len(final_path1)} points")

        seg1_tracks = []
        for i in range(1, len(final_path1)):
            seg1_tracks.append({
                'start': final_path1[i-1],
                'end': final_path1[i],
                'width': track_width,
                'layer': primary_layer,
                'net_id': net.net_id
            })

        # Try each alternate layer for segment 2
        two_via_success = False
        for alternate_layer in alternate_layers:
            print(f"\n  Trying segment 2 on {alternate_layer}...")
            obstacles2 = build_obstacles(pcb_data, net, all_routed_tracks, alternate_layer)
            path2 = route_path((via1_x, via1_y), (via2_x, via2_y), obstacles2, max_time=0.5)

            if not path2:
                # Try direct connection
                print(f"    Route failed, trying direct connection on {alternate_layer}...")
                path2 = [(via1_x, via1_y), (via2_x, via2_y)]
                final_path2 = path2
            else:
                final_path2 = process_path(path2, obstacles2, routing_clearance)

            print(f"    Segment 2: {len(final_path2)} points")

            seg2_tracks = []
            for i in range(1, len(final_path2)):
                seg2_tracks.append({
                    'start': final_path2[i-1],
                    'end': final_path2[i],
                    'width': track_width,
                    'layer': alternate_layer,
                    'net_id': net.net_id
                })

            # Segment 3: Via2 to Sink on primary layer
            print(f"\n  Routing segment 3: Via2 to Sink on {primary_layer}...")
            obstacles3 = build_obstacles(pcb_data, net, all_routed_tracks + seg1_tracks + seg2_tracks, primary_layer)
            path3 = route_path((via2_x, via2_y), sink, obstacles3, max_time=0.5)

            if not path3:
                print(f"    Failed to route to sink, trying next layer...")
                continue

            final_path3 = process_path(path3, obstacles3, routing_clearance)
            print(f"    Segment 3: {len(final_path3)} points")

            seg3_tracks = []
            for i in range(1, len(final_path3)):
                seg3_tracks.append({
                    'start': final_path3[i-1],
                    'end': final_path3[i],
                    'width': track_width,
                    'layer': primary_layer,
                    'net_id': net.net_id
                })

            # Success! Add all tracks and vias
            all_tracks.extend(seg1_tracks)
            all_tracks.extend(seg2_tracks)
            all_tracks.extend(seg3_tracks)

            new_vias.append({
                'x': via1_x,
                'y': via1_y,
                'size': VIA_SIZE,
                'drill': VIA_DRILL,
                'layers': via_layers,
                'net_id': net.net_id
            })
            new_vias.append({
                'x': via2_x,
                'y': via2_y,
                'size': VIA_SIZE,
                'drill': VIA_DRILL,
                'layers': via_layers,
                'net_id': net.net_id
            })

            two_via_success = True
            break

        if not two_via_success:
            print("  All alternate layers failed for two-via approach!")
            return [], []

    elif not alternate_layers:
        print("  No alternate layers available for via crossing!")
        return [], []

    # Calculate total path length
    total_length = 0
    for track in all_tracks:
        dx = track['end'][0] - track['start'][0]
        dy = track['end'][1] - track['start'][1]
        total_length += math.sqrt(dx**2 + dy**2)
    print(f"\nTotal path length: {total_length:.2f} mm (with {len(new_vias)} vias)")

    return all_tracks, new_vias


def main():
    parser = argparse.ArgumentParser(
        description='Route nets on a KiCad PCB with multi-layer support.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python test_simple_router.py "Net-(U2A-DATA_0)"
    python test_simple_router.py --layers F.Cu,B.Cu,In2.Cu,In7.Cu "Net-(U2A-DATA_0)" "Net-(U2A-DATA_12)"
        """
    )
    parser.add_argument('nets', nargs='*', default=['Net-(U2A-DATA_0)'],
                        help='Net patterns to route (default: Net-(U2A-DATA_0))')
    parser.add_argument('--layers', '-l', type=str, default='F.Cu,B.Cu',
                        help='Comma-separated list of allowed routing layers (default: F.Cu,B.Cu)')
    parser.add_argument('--pcb', '-p', type=str, default='../haasoscope_pro_max/haasoscope_pro_max.kicad_pcb',
                        help='Input PCB file path')
    parser.add_argument('--output', '-o', type=str, default='routed.kicad_pcb',
                        help='Output PCB file path (default: routed.kicad_pcb)')
    parser.add_argument('--incremental', '-i', action='store_true',
                        help='Incremental mode: use output file as input if it exists, skip already-routed nets')

    args = parser.parse_args()

    pcb_path = args.pcb
    output_path = args.output
    net_patterns = args.nets
    allowed_layers = [l.strip() for l in args.layers.split(',')]
    incremental = args.incremental

    # In incremental mode, use output file as input if it exists
    import os
    if incremental and os.path.exists(output_path):
        pcb_path = output_path
        print(f"Incremental mode: using {output_path} as input")

    print(f"Parsing {pcb_path}...")
    print(f"Routing layers: {', '.join(allowed_layers)}")
    pcb_data = parse_kicad_pcb(pcb_path)

    # Get nets to route - preserve order from command line
    all_nets = get_nets_to_route(pcb_data, net_patterns=net_patterns)

    # Reorder to match command line order
    nets = []
    for pattern in net_patterns:
        for net in all_nets:
            if net.name == pattern and net not in nets:
                nets.append(net)
    for net in all_nets:
        if net not in nets:
            nets.append(net)

    if not nets:
        print(f"No nets found matching patterns: {net_patterns}")
        return

    # In incremental mode, find which nets are already routed (have tracks)
    already_routed_nets = set()
    if incremental:
        for segment in pcb_data.segments:
            already_routed_nets.add(segment.net_id)

    # Filter out already-routed nets
    nets_to_route = []
    skipped_count = 0
    for net in nets:
        if incremental and net.net_id in already_routed_nets:
            print(f"Skipping {net.name} (already routed)")
            skipped_count += 1
        else:
            nets_to_route.append(net)

    print(f"Found {len(nets)} nets, {skipped_count} already routed, {len(nets_to_route)} to route")

    all_tracks = []
    all_vias = []
    success_count = 0
    fail_count = 0

    for net in nets_to_route:
        tracks, vias = route_net(pcb_data, net, all_tracks, all_vias, allowed_layers)
        if tracks:
            all_tracks.extend(tracks)
            all_vias.extend(vias)
            success_count += 1
        else:
            fail_count += 1

    if all_tracks or (incremental and skipped_count > 0):
        print(f"\n{'='*50}")
        print(f"ROUTING SUMMARY")
        print(f"{'='*50}")
        if skipped_count > 0:
            print(f"Already routed (skipped): {skipped_count}")
        print(f"Newly routed: {success_count}")
        print(f"Failed: {fail_count}")
        print(f"New track segments: {len(all_tracks)}")
        print(f"New vias: {len(all_vias)}")

        if all_tracks:
            # Count tracks per layer
            layer_counts = {}
            for track in all_tracks:
                layer = track['layer']
                layer_counts[layer] = layer_counts.get(layer, 0) + 1
            for layer, count in sorted(layer_counts.items()):
                print(f"  {layer}: {count} segments")

            print(f"\nWriting to {output_path}...")
            add_tracks_and_vias_to_pcb(pcb_path, output_path, all_tracks, all_vias)
            print(f"Saved routed PCB to {output_path}")
        elif incremental:
            print(f"\nNo new routes needed - all requested nets already routed")
    else:
        print("No routes completed!")


if __name__ == '__main__':
    main()
