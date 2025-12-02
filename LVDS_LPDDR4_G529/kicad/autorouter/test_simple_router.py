#!/usr/bin/env python3
"""
Test the simple router on actual PCB data and output to KiCad.
Supports multi-layer routing with vias when tracks need to cross.
"""

import sys
import math
from pathlib import Path
from simple_router import route_single_net, route_bidirectional, simplify_path, simplify_path_safe, Obstacle, optimize_simplified_path, snap_to_45_degrees, verify_path, enforce_45_degree_path, remove_kinks, try_multi_zone_centering, SpatialIndex
from kicad_parser import parse_kicad_pcb, get_nets_to_route
from kicad_writer import add_tracks_and_vias_to_pcb

# Via parameters
VIA_SIZE = 0.6  # Via pad size
VIA_DRILL = 0.3  # Via drill size
VIA_LAYERS = ["F.Cu", "B.Cu"]  # Through-hole via


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


def route_path(source, sink, obstacles, step_size=0.1, routing_clearance=0.05, max_time=10.0):
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
    """
    # Simple heuristic: find the blocking track closest to the midpoint
    mid_x = (source[0] + sink[0]) / 2
    mid_y = (source[1] + sink[1]) / 2

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


def route_net(pcb_data, net, all_routed_tracks, all_vias):
    """Route a single net and return the track segments and vias."""
    print(f"\nRouting net: {net.name}")
    print(f"  Pads: {len(net.pads)}")

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
    primary_layer = "F.Cu"
    alternate_layer = "B.Cu"

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

    # Primary layer failed - try using vias to cross under blocking tracks
    print(f"\nPrimary layer routing failed. Trying via crossing...")

    # Find where we need to cross
    cross_point, blocking_track = find_crossing_point(source, sink, all_routed_tracks, primary_layer)

    if not cross_point:
        print("  Could not identify crossing point")
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
    if source_via:
        print(f"  Using existing source via - starting on {alternate_layer}")
        # Route entirely on alternate layer from source via to a single new via
        # then back to primary layer for the rest

        # Only need one via - place it on the sink side of the crossing
        new_via_x = via2_x
        new_via_y = via2_y
        print(f"  Single new via at ({new_via_x:.2f}, {new_via_y:.2f})")

        # Segment 1: Source via to new via on alternate layer
        print(f"\n  Routing segment 1: Source via to new via on {alternate_layer}...")
        obstacles1 = build_obstacles(pcb_data, net, all_routed_tracks, alternate_layer)
        source_via_pos = (source_via.x, source_via.y)
        path1 = route_path(source_via_pos, (new_via_x, new_via_y), obstacles1, max_time=5.0)

        if not path1:
            print("    Failed to route on alternate layer")
            # Fall back to standard two-via approach
            source_via = None
        else:
            final_path1 = process_path(path1, obstacles1, routing_clearance)
            print(f"    Segment 1: {len(final_path1)} points")

            for i in range(1, len(final_path1)):
                all_tracks.append({
                    'start': final_path1[i-1],
                    'end': final_path1[i],
                    'width': track_width,
                    'layer': alternate_layer,
                    'net_id': net.net_id
                })

            # Add the single new via
            new_vias.append({
                'x': new_via_x,
                'y': new_via_y,
                'size': VIA_SIZE,
                'drill': VIA_DRILL,
                'layers': VIA_LAYERS,
                'net_id': net.net_id
            })

            # Segment 2: New via to sink on primary layer
            print(f"\n  Routing segment 2: New via to Sink on {primary_layer}...")
            obstacles2 = build_obstacles(pcb_data, net, all_routed_tracks + all_tracks, primary_layer)
            path2 = route_path((new_via_x, new_via_y), sink, obstacles2, max_time=5.0)

            if not path2:
                print("    Failed to route to sink")
                return [], []

            final_path2 = process_path(path2, obstacles2, routing_clearance)
            print(f"    Segment 2: {len(final_path2)} points")

            for i in range(1, len(final_path2)):
                all_tracks.append({
                    'start': final_path2[i-1],
                    'end': final_path2[i],
                    'width': track_width,
                    'layer': primary_layer,
                    'net_id': net.net_id
                })

    # Standard two-via approach if no source via or fallback needed
    if not source_via:
        print(f"  Via 1 at ({via1_x:.2f}, {via1_y:.2f})")
        print(f"  Via 2 at ({via2_x:.2f}, {via2_y:.2f})")

        # Route in three segments:
        # 1. Source to Via1 on primary layer
        # 2. Via1 to Via2 on alternate layer (crossing under)
        # 3. Via2 to Sink on primary layer

        # Segment 1: Source to Via1 on primary layer
        print(f"\n  Routing segment 1: Source to Via1 on {primary_layer}...")
        obstacles1 = build_obstacles(pcb_data, net, all_routed_tracks, primary_layer)
        path1 = route_path(source, (via1_x, via1_y), obstacles1, max_time=5.0)

        if not path1:
            print("    Failed to route to Via1")
            return [], []

        final_path1 = process_path(path1, obstacles1, routing_clearance)
        print(f"    Segment 1: {len(final_path1)} points")

        for i in range(1, len(final_path1)):
            all_tracks.append({
                'start': final_path1[i-1],
                'end': final_path1[i],
                'width': track_width,
                'layer': primary_layer,
                'net_id': net.net_id
            })

        # Add Via 1
        new_vias.append({
            'x': via1_x,
            'y': via1_y,
            'size': VIA_SIZE,
            'drill': VIA_DRILL,
            'layers': VIA_LAYERS,
            'net_id': net.net_id
        })

        # Segment 2: Via1 to Via2 on alternate layer
        print(f"\n  Routing segment 2: Via1 to Via2 on {alternate_layer}...")
        obstacles2 = build_obstacles(pcb_data, net, all_routed_tracks, alternate_layer)
        path2 = route_path((via1_x, via1_y), (via2_x, via2_y), obstacles2, max_time=5.0)

        if not path2:
            # Direct connection if route fails
            print("    Using direct connection")
            path2 = [(via1_x, via1_y), (via2_x, via2_y)]
            final_path2 = path2
        else:
            final_path2 = process_path(path2, obstacles2, routing_clearance)

        print(f"    Segment 2: {len(final_path2)} points")

        for i in range(1, len(final_path2)):
            all_tracks.append({
                'start': final_path2[i-1],
                'end': final_path2[i],
                'width': track_width,
                'layer': alternate_layer,
                'net_id': net.net_id
            })

        # Add Via 2
        new_vias.append({
            'x': via2_x,
            'y': via2_y,
            'size': VIA_SIZE,
            'drill': VIA_DRILL,
            'layers': VIA_LAYERS,
            'net_id': net.net_id
        })

        # Segment 3: Via2 to Sink on primary layer
        print(f"\n  Routing segment 3: Via2 to Sink on {primary_layer}...")
        # Add the new tracks as obstacles for segment 3
        obstacles3 = build_obstacles(pcb_data, net, all_routed_tracks + all_tracks, primary_layer)
        path3 = route_path((via2_x, via2_y), sink, obstacles3, max_time=5.0)

        if not path3:
            print("    Failed to route from Via2 to sink")
            return [], []

        final_path3 = process_path(path3, obstacles3, routing_clearance)
        print(f"    Segment 3: {len(final_path3)} points")

        for i in range(1, len(final_path3)):
            all_tracks.append({
                'start': final_path3[i-1],
                'end': final_path3[i],
                'width': track_width,
                'layer': primary_layer,
                'net_id': net.net_id
            })

    # Calculate total path length
    total_length = 0
    for track in all_tracks:
        dx = track['end'][0] - track['start'][0]
        dy = track['end'][1] - track['start'][1]
        total_length += math.sqrt(dx**2 + dy**2)
    print(f"\nTotal path length: {total_length:.2f} mm (with {len(new_vias)} vias)")

    return all_tracks, new_vias


def main():
    pcb_path = "../haasoscope_pro_max/haasoscope_pro_max.kicad_pcb"
    output_path = "routed.kicad_pcb"

    # Net patterns - use command line args or default
    if len(sys.argv) > 1:
        net_patterns = sys.argv[1:]
    else:
        net_patterns = ["Net-(U2A-DATA_0)"]

    print(f"Parsing {pcb_path}...")
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

    print(f"Found {len(nets)} nets to route")

    all_tracks = []
    all_vias = []
    success_count = 0
    fail_count = 0

    for net in nets:
        tracks, vias = route_net(pcb_data, net, all_tracks, all_vias)
        if tracks:
            all_tracks.extend(tracks)
            all_vias.extend(vias)
            success_count += 1
        else:
            fail_count += 1

    if all_tracks:
        print(f"\n{'='*50}")
        print(f"ROUTING SUMMARY")
        print(f"{'='*50}")
        print(f"Successfully routed: {success_count}")
        print(f"Failed: {fail_count}")
        print(f"Total track segments: {len(all_tracks)}")
        print(f"Total vias: {len(all_vias)}")

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
    else:
        print("No routes completed!")


if __name__ == '__main__':
    main()
