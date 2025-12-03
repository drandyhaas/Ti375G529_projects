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


def build_obstacles(pcb_data, net, all_routed_tracks, layer="F.Cu", all_vias=None):
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

    # Add session vias as obstacles (except this net's vias)
    if all_vias:
        for via in all_vias:
            if via['net_id'] != net.net_id:
                obstacles.append(Obstacle(via['x'], via['y'], via['size'], via['size'], 0, via['size']/2))

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


def line_segment_intersection(p1, p2, p3, p4):
    """
    Find intersection point of line segments p1-p2 and p3-p4.
    Returns (x, y, t) where t is parameter along p1-p2, or None if no intersection.
    """
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4

    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-10:
        return None  # Parallel lines

    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom

    if 0 <= t <= 1 and 0 <= u <= 1:
        ix = x1 + t * (x2 - x1)
        iy = y1 + t * (y2 - y1)
        return (ix, iy, t)
    return None


def point_to_segment_distance(px, py, x1, y1, x2, y2):
    """Calculate minimum distance from point (px, py) to line segment (x1,y1)-(x2,y2)."""
    dx, dy = x2 - x1, y2 - y1
    if dx == 0 and dy == 0:
        return math.sqrt((px - x1)**2 + (py - y1)**2)
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
    nearest_x = x1 + t * dx
    nearest_y = y1 + t * dy
    return math.sqrt((px - nearest_x)**2 + (py - nearest_y)**2)


def find_crossing_point(source, sink, blocking_tracks, layer):
    """
    Find where the route would need to cross a blocking track.
    Returns (crossing_point, blocking_track) or (None, None).

    Works for any route orientation (horizontal, vertical, diagonal).
    Uses actual line-segment intersection when possible, otherwise finds
    the closest point on blocking tracks to the route line.
    """
    route_dx = sink[0] - source[0]
    route_dy = sink[1] - source[1]
    route_len = math.sqrt(route_dx**2 + route_dy**2)

    if route_len < 0.001:
        return None, None

    # Define a corridor around the route - tracks must be within this to be relevant
    corridor_margin = 1.0  # 1mm margin perpendicular to route

    best_track = None
    best_cross_point = None
    best_t = float('inf')  # Parameter along route (0=source, 1=sink), prefer earlier crossings

    for track in blocking_tracks:
        if track['layer'] != layer:
            continue

        track_start = track['start']
        track_end = track['end']

        # Check bounding box overlap first (quick rejection)
        route_min_x = min(source[0], sink[0]) - corridor_margin
        route_max_x = max(source[0], sink[0]) + corridor_margin
        route_min_y = min(source[1], sink[1]) - corridor_margin
        route_max_y = max(source[1], sink[1]) + corridor_margin

        track_min_x = min(track_start[0], track_end[0])
        track_max_x = max(track_start[0], track_end[0])
        track_min_y = min(track_start[1], track_end[1])
        track_max_y = max(track_start[1], track_end[1])

        if track_max_x < route_min_x or track_min_x > route_max_x:
            continue
        if track_max_y < route_min_y or track_min_y > route_max_y:
            continue

        # Try to find actual intersection
        intersection = line_segment_intersection(source, sink, track_start, track_end)

        if intersection:
            ix, iy, t = intersection
            if t < best_t:
                best_t = t
                best_track = track
                best_cross_point = (ix, iy)
        else:
            # No direct intersection - find closest point on track to route line
            # This handles tracks that are parallel but blocking
            mid_route = ((source[0] + sink[0]) / 2, (source[1] + sink[1]) / 2)

            # Distance from route midpoint to track segment
            dist = point_to_segment_distance(mid_route[0], mid_route[1],
                                            track_start[0], track_start[1],
                                            track_end[0], track_end[1])

            # Only consider if track is close enough to be blocking
            if dist < corridor_margin:
                # Find closest point on track to route line
                track_dx = track_end[0] - track_start[0]
                track_dy = track_end[1] - track_start[1]
                track_len_sq = track_dx**2 + track_dy**2

                if track_len_sq > 0.0001:
                    # Project route midpoint onto track
                    t_track = ((mid_route[0] - track_start[0]) * track_dx +
                              (mid_route[1] - track_start[1]) * track_dy) / track_len_sq
                    t_track = max(0, min(1, t_track))
                    cross_x = track_start[0] + t_track * track_dx
                    cross_y = track_start[1] + t_track * track_dy
                else:
                    cross_x = (track_start[0] + track_end[0]) / 2
                    cross_y = (track_start[1] + track_end[1]) / 2

                # Calculate t along route for this crossing point
                if route_len > 0.001:
                    t_route = ((cross_x - source[0]) * route_dx +
                              (cross_y - source[1]) * route_dy) / (route_len * route_len)
                else:
                    t_route = 0.5

                if 0 <= t_route <= 1 and t_route < best_t:
                    best_t = t_route
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


def check_via_collision(x, y, via_size, pcb_data, all_routed_tracks, net_id, clearance=0.1):
    """
    Check if a via at (x, y) would collide with existing tracks on any layer.
    Returns True if collision detected, False if position is clear.
    """
    via_radius = via_size / 2 + clearance

    # Check against existing PCB tracks (all layers - vias go through all)
    for track in pcb_data.segments:
        if track.net_id == net_id:
            continue  # Don't check our own net's tracks
        # Point-to-segment distance
        start_x, start_y = track.start_x, track.start_y
        end_x, end_y = track.end_x, track.end_y

        dx = end_x - start_x
        dy = end_y - start_y
        seg_len_sq = dx*dx + dy*dy

        if seg_len_sq < 0.0001:
            # Track is essentially a point
            dist = math.sqrt((x - start_x)**2 + (y - start_y)**2)
        else:
            # Project point onto line segment
            t = max(0, min(1, ((x - start_x)*dx + (y - start_y)*dy) / seg_len_sq))
            proj_x = start_x + t * dx
            proj_y = start_y + t * dy
            dist = math.sqrt((x - proj_x)**2 + (y - proj_y)**2)

        min_dist = via_radius + track.width / 2
        if dist < min_dist:
            return True

    # Check against session tracks (all layers)
    for track in all_routed_tracks:
        if track['net_id'] == net_id:
            continue
        start_x, start_y = track['start']
        end_x, end_y = track['end']

        dx = end_x - start_x
        dy = end_y - start_y
        seg_len_sq = dx*dx + dy*dy

        if seg_len_sq < 0.0001:
            dist = math.sqrt((x - start_x)**2 + (y - start_y)**2)
        else:
            t = max(0, min(1, ((x - start_x)*dx + (y - start_y)*dy) / seg_len_sq))
            proj_x = start_x + t * dx
            proj_y = start_y + t * dy
            dist = math.sqrt((x - proj_x)**2 + (y - proj_y)**2)

        min_dist = via_radius + track['width'] / 2
        if dist < min_dist:
            return True

    # Check against existing vias
    for via in pcb_data.vias:
        if via.net_id == net_id:
            continue
        dist = math.sqrt((x - via.x)**2 + (y - via.y)**2)
        min_dist = via_radius + via.size / 2
        if dist < min_dist:
            return True

    # Check against pads (vias go through all layers so must avoid all pads)
    # Use bounding box check first for speed
    max_pad_radius = 0.5  # Approximate max pad radius in mm
    search_radius = via_radius + max_pad_radius + clearance
    for pad_net_id, pads in pcb_data.pads_by_net.items():
        if pad_net_id == net_id:
            continue  # Don't check our own net's pads
        for pad in pads:
            pad_x = pad.global_x
            pad_y = pad.global_y
            # Quick bounding box check
            if abs(x - pad_x) > search_radius or abs(y - pad_y) > search_radius:
                continue
            pad_radius = max(pad.size_x, pad.size_y) / 2
            dist = math.sqrt((x - pad_x)**2 + (y - pad_y)**2)
            min_dist = via_radius + pad_radius
            if dist < min_dist:
                return True

    return False


def find_valid_via_position(initial_x, initial_y, via_size, pcb_data, all_routed_tracks,
                            net_id, source, sink, clearance=0.1, max_offset=2.0):
    """
    Find a valid via position near (initial_x, initial_y) that doesn't collide with tracks.
    Searches in a spiral pattern, preferring positions along the source-sink axis.

    Returns (x, y) of valid position, or None if no valid position found.
    """
    # First check if initial position is valid
    if not check_via_collision(initial_x, initial_y, via_size, pcb_data,
                               all_routed_tracks, net_id, clearance):
        return initial_x, initial_y

    # Calculate direction along route
    route_dx = sink[0] - source[0]
    route_dy = sink[1] - source[1]
    route_len = math.sqrt(route_dx**2 + route_dy**2)
    if route_len > 0:
        route_dx /= route_len
        route_dy /= route_len
    else:
        route_dx, route_dy = 1, 0

    # Perpendicular direction
    perp_dx = -route_dy
    perp_dy = route_dx

    # Search in increments along and perpendicular to route
    step = 0.1  # 0.1mm steps
    for offset in [i * step for i in range(1, int(max_offset / step) + 1)]:
        # Try positions along route axis first (toward sink, then toward source)
        for along in [offset, -offset]:
            test_x = initial_x + along * route_dx
            test_y = initial_y + along * route_dy
            if not check_via_collision(test_x, test_y, via_size, pcb_data,
                                       all_routed_tracks, net_id, clearance):
                return test_x, test_y

        # Try perpendicular positions
        for perp in [offset, -offset]:
            test_x = initial_x + perp * perp_dx
            test_y = initial_y + perp * perp_dy
            if not check_via_collision(test_x, test_y, via_size, pcb_data,
                                       all_routed_tracks, net_id, clearance):
                return test_x, test_y

        # Try diagonal positions
        for along in [offset, -offset]:
            for perp in [offset, -offset]:
                test_x = initial_x + along * route_dx + perp * perp_dx
                test_y = initial_y + along * route_dy + perp * perp_dy
                if not check_via_collision(test_x, test_y, via_size, pcb_data,
                                           all_routed_tracks, net_id, clearance):
                    return test_x, test_y

    return None  # No valid position found


def find_clear_channel(source, sink, cross_point, pcb_data, all_routed_tracks, net_id, layer,
                       search_range=2.0, track_width=0.1, clearance=0.05):
    """
    Find clear channel positions perpendicular to the route direction.

    Works for any route orientation (horizontal, vertical, diagonal).
    Searches perpendicular to the route direction for clear paths.

    Args:
        source, sink: Route endpoints
        cross_point: Point where we need to place via (near blocking track)
        pcb_data: PCB data with existing segments
        all_routed_tracks: Session tracks
        net_id: Net to exclude from collision checks
        layer: Layer to check
        search_range: How far perpendicular to search (mm)
        track_width: Width of tracks
        clearance: Required clearance

    Returns:
        List of ((x, y), gap_size) tuples for clear channel positions,
        sorted by proximity to route line
    """
    min_spacing = track_width + clearance * 2

    # Get route direction
    route_dx = sink[0] - source[0]
    route_dy = sink[1] - source[1]
    route_len = math.sqrt(route_dx**2 + route_dy**2)

    if route_len < 0.001:
        return []

    # Normalize route direction
    route_dx /= route_len
    route_dy /= route_len

    # Perpendicular direction (rotate 90 degrees)
    perp_dx = -route_dy
    perp_dy = route_dx

    # Search area bounds along route direction (from cross_point toward sink)
    search_along_min = 0
    search_along_max = min(route_len * 0.5, 5.0)  # Up to 5mm along route

    # Collect blocking positions projected onto perpendicular axis
    blocking_perp_values = []

    # Check PCB segments
    for seg in pcb_data.segments:
        if seg.net_id == net_id or seg.layer != layer:
            continue

        # Check if track is in our search area
        for px, py in [(seg.start_x, seg.start_y), (seg.end_x, seg.end_y)]:
            # Vector from cross_point to track point
            dx = px - cross_point[0]
            dy = py - cross_point[1]

            # Project onto route direction
            along = dx * route_dx + dy * route_dy
            if along < search_along_min - 1 or along > search_along_max + 1:
                continue

            # Project onto perpendicular direction
            perp = dx * perp_dx + dy * perp_dy
            if -search_range - 1 <= perp <= search_range + 1:
                blocking_perp_values.append(perp)

    # Check session tracks
    for track in all_routed_tracks:
        if track['net_id'] == net_id or track['layer'] != layer:
            continue

        for px, py in [track['start'], track['end']]:
            dx = px - cross_point[0]
            dy = py - cross_point[1]

            along = dx * route_dx + dy * route_dy
            if along < search_along_min - 1 or along > search_along_max + 1:
                continue

            perp = dx * perp_dx + dy * perp_dy
            if -search_range - 1 <= perp <= search_range + 1:
                blocking_perp_values.append(perp)

    if not blocking_perp_values:
        # No blocking tracks, can route directly
        return [((cross_point[0] + route_dx * 0.5, cross_point[1] + route_dy * 0.5), search_range * 2)]

    # Sort and find gaps in perpendicular direction
    blocking_perp_values = sorted(set(blocking_perp_values))

    channels = []

    # Check gap before first blocking track
    if blocking_perp_values[0] > -search_range + min_spacing:
        gap_perp = (-search_range + blocking_perp_values[0]) / 2
        gap_size = blocking_perp_values[0] - (-search_range)
        if gap_size > min_spacing:
            # Convert back to (x, y) coordinates
            via_x = cross_point[0] + route_dx * 0.5 + perp_dx * gap_perp
            via_y = cross_point[1] + route_dy * 0.5 + perp_dy * gap_perp
            channels.append(((via_x, via_y), gap_size))

    # Check gaps between blocking tracks
    for i in range(len(blocking_perp_values) - 1):
        gap = blocking_perp_values[i + 1] - blocking_perp_values[i]
        if gap > min_spacing:
            gap_perp = (blocking_perp_values[i] + blocking_perp_values[i + 1]) / 2
            if -search_range <= gap_perp <= search_range:
                via_x = cross_point[0] + route_dx * 0.5 + perp_dx * gap_perp
                via_y = cross_point[1] + route_dy * 0.5 + perp_dy * gap_perp
                channels.append(((via_x, via_y), gap))

    # Check gap after last blocking track
    if blocking_perp_values[-1] < search_range - min_spacing:
        gap_perp = (blocking_perp_values[-1] + search_range) / 2
        gap_size = search_range - blocking_perp_values[-1]
        if gap_size > min_spacing:
            via_x = cross_point[0] + route_dx * 0.5 + perp_dx * gap_perp
            via_y = cross_point[1] + route_dy * 0.5 + perp_dy * gap_perp
            channels.append(((via_x, via_y), gap_size))

    # Sort by proximity to route line (prefer positions close to direct path)
    channels.sort(key=lambda c: abs(
        (c[0][0] - cross_point[0]) * perp_dx + (c[0][1] - cross_point[1]) * perp_dy
    ))

    return channels


# Keep the old function as an alias for backward compatibility
def find_clear_y_channel(x_start, x_end, pcb_data, all_routed_tracks, net_id, layer,
                         y_min, y_max, track_width=0.1, clearance=0.05):
    """
    Legacy function - find Y values where there's a clear horizontal channel.
    For new code, use find_clear_channel() which handles all orientations.
    """
    min_spacing = track_width + clearance * 2

    blocking_y_values = []

    for seg in pcb_data.segments:
        if seg.net_id == net_id or seg.layer != layer:
            continue
        seg_x_min = min(seg.start_x, seg.end_x)
        seg_x_max = max(seg.start_x, seg.end_x)
        if seg_x_max < x_start or seg_x_min > x_end:
            continue
        for y in [seg.start_y, seg.end_y]:
            if y_min - 1 <= y <= y_max + 1:
                blocking_y_values.append(y)

    for track in all_routed_tracks:
        if track['net_id'] == net_id or track['layer'] != layer:
            continue
        seg_x_min = min(track['start'][0], track['end'][0])
        seg_x_max = max(track['start'][0], track['end'][0])
        if seg_x_max < x_start or seg_x_min > x_end:
            continue
        for y in [track['start'][1], track['end'][1]]:
            if y_min - 1 <= y <= y_max + 1:
                blocking_y_values.append(y)

    if not blocking_y_values:
        return [((y_min + y_max) / 2, y_max - y_min)]

    blocking_y_values = sorted(set(blocking_y_values))

    channels = []
    y_mid = (y_min + y_max) / 2

    if blocking_y_values[0] > y_min + min_spacing:
        gap_y = (y_min + blocking_y_values[0]) / 2
        gap_size = blocking_y_values[0] - y_min
        if y_min <= gap_y <= y_max:
            channels.append((gap_y, gap_size))

    for i in range(len(blocking_y_values) - 1):
        gap = blocking_y_values[i + 1] - blocking_y_values[i]
        if gap > min_spacing:
            gap_y = (blocking_y_values[i] + blocking_y_values[i + 1]) / 2
            if y_min <= gap_y <= y_max:
                channels.append((gap_y, gap))

    if blocking_y_values[-1] < y_max - min_spacing:
        gap_y = (blocking_y_values[-1] + y_max) / 2
        gap_size = y_max - blocking_y_values[-1]
        if y_min <= gap_y <= y_max:
            channels.append((gap_y, gap_size))

    channels.sort(key=lambda c: abs(c[0] - y_mid))

    return channels


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
    obstacles = build_obstacles(pcb_data, net, all_routed_tracks, primary_layer, all_vias)
    print(f"  Total obstacles on {primary_layer}: {len(obstacles)}")

    print(f"\nRouting on {primary_layer}...")
    path = route_path(source, sink, obstacles, max_time=0.3)

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

    # Primary layer failed - try other layers only if BOTH source and sink have vias
    print(f"\nPrimary layer routing failed.")

    # If BOTH source and sink have vias, we can route entirely on an alternate layer
    # Both ends have layer transitions available
    if source_via and sink_via and alternate_layers:
        source_via_pos = (source_via.x, source_via.y)
        sink_via_pos = (sink_via.x, sink_via.y)
        for alt_layer in alternate_layers:
            print(f"  Trying route via-to-via on {alt_layer}...")
            alt_obstacles = build_obstacles(pcb_data, net, all_routed_tracks, alt_layer, all_vias)
            # Route from source via to sink via on alternate layer
            alt_path = route_path(source_via_pos, sink_via_pos, alt_obstacles, max_time=0.25)
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
                print(f"  Path length: {total_length:.2f} mm (using existing vias)")
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

    # Find clear channels perpendicular to route direction
    # This works for horizontal, vertical, and diagonal routes
    clear_channels = find_clear_channel(
        source, sink, cross_point, pcb_data, all_routed_tracks,
        net.net_id, primary_layer, search_range=2.0
    )

    if clear_channels:
        # Use the best channel position for via placement
        best_pos, gap_size = clear_channels[0]
        print(f"  Found clear channel at ({best_pos[0]:.2f}, {best_pos[1]:.2f}) (gap={gap_size:.2f}mm)")
        # Use the clear channel position for via2
        via2_x, via2_y = best_pos
    else:
        print(f"  No clear channel found on {primary_layer}")

    all_tracks = []
    new_vias = []

    # Check if source already has a via - can start on alternate layer
    if source_via and alternate_layers:
        # Build list of via positions to try
        # Prioritize positions closer to sink for easier segment 2 routing
        via_positions_to_try = []

        # Add positions along route toward sink (50%, 75% of the way)
        route_dx = sink[0] - cross_point[0]
        route_dy = sink[1] - cross_point[1]
        for fraction in [0.75, 0.5]:  # Closer to sink first
            pos_x = cross_point[0] + route_dx * fraction
            pos_y = cross_point[1] + route_dy * fraction
            via_positions_to_try.append((pos_x, pos_y))

        # Add clear channel positions (limit to first 2)
        if clear_channels:
            for pos, gap_size in clear_channels[:2]:
                # Avoid duplicating similar positions
                is_duplicate = False
                for existing_pos in via_positions_to_try:
                    if abs(existing_pos[0] - pos[0]) < 0.5 and abs(existing_pos[1] - pos[1]) < 0.5:
                        is_duplicate = True
                        break
                if not is_duplicate:
                    via_positions_to_try.append(pos)

        if not via_positions_to_try:
            via_positions_to_try.append((via2_x, via2_y))

        # Limit total positions to try (each position tries 1 layer with 2 layers = 2 attempts per position)
        via_positions_to_try = via_positions_to_try[:2]

        route_success = False
        # Try each via position, then each alternate layer for that position
        for via_pos_idx, try_via_pos in enumerate(via_positions_to_try):
            if route_success:
                break

            for alternate_layer in alternate_layers:
                print(f"  Trying via position {via_pos_idx+1}/{len(via_positions_to_try)} at ({try_via_pos[0]:.2f}, {try_via_pos[1]:.2f}) on {alternate_layer}")

                # Find a valid position that doesn't collide with existing tracks
                valid_pos = find_valid_via_position(try_via_pos[0], try_via_pos[1], VIA_SIZE, pcb_data,
                                                    all_routed_tracks, net.net_id, source, sink)
                if valid_pos is None:
                    print(f"    Could not find valid via position near ({try_via_pos[0]:.2f}, {try_via_pos[1]:.2f})")
                    continue

                new_via_x, new_via_y = valid_pos
                if abs(new_via_x - try_via_pos[0]) > 0.01 or abs(new_via_y - try_via_pos[1]) > 0.01:
                    print(f"    Via position adjusted to ({new_via_x:.2f}, {new_via_y:.2f})")

                # Segment 1: Source via to new via on alternate layer
                print(f"\n  Routing segment 1: Source via to new via on {alternate_layer}...")
                obstacles1 = build_obstacles(pcb_data, net, all_routed_tracks, alternate_layer, all_vias)
                source_via_pos = (source_via.x, source_via.y)
                path1 = route_path(source_via_pos, (new_via_x, new_via_y), obstacles1, max_time=0.25)

                if not path1:
                    print(f"    Failed to route on {alternate_layer}, trying next...")
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
                obstacles2 = build_obstacles(pcb_data, net, all_routed_tracks + temp_tracks, primary_layer, all_vias)
                path2 = route_path((new_via_x, new_via_y), sink, obstacles2, max_time=0.25)

                if not path2:
                    print(f"    Failed to route to sink on {primary_layer}, trying next via position...")
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
            print("  All via positions and alternate layers failed, falling back to two-via approach")
            source_via = None  # Force two-via approach

    # Standard two-via approach if no source via or fallback needed
    if not source_via and alternate_layers:
        # Validate via positions - find collision-free positions
        valid_pos1 = find_valid_via_position(via1_x, via1_y, VIA_SIZE, pcb_data,
                                             all_routed_tracks, net.net_id, source, sink)
        valid_pos2 = find_valid_via_position(via2_x, via2_y, VIA_SIZE, pcb_data,
                                             all_routed_tracks, net.net_id, source, sink)

        if valid_pos1 is None or valid_pos2 is None:
            print(f"  Could not find valid via positions for two-via approach")
            return [], []

        if abs(valid_pos1[0] - via1_x) > 0.01 or abs(valid_pos1[1] - via1_y) > 0.01:
            print(f"  Via 1 adjusted from ({via1_x:.2f}, {via1_y:.2f}) to ({valid_pos1[0]:.2f}, {valid_pos1[1]:.2f})")
        if abs(valid_pos2[0] - via2_x) > 0.01 or abs(valid_pos2[1] - via2_y) > 0.01:
            print(f"  Via 2 adjusted from ({via2_x:.2f}, {via2_y:.2f}) to ({valid_pos2[0]:.2f}, {valid_pos2[1]:.2f})")

        via1_x, via1_y = valid_pos1
        via2_x, via2_y = valid_pos2

        print(f"  Via 1 at ({via1_x:.2f}, {via1_y:.2f})")
        print(f"  Via 2 at ({via2_x:.2f}, {via2_y:.2f})")

        # Route segment 1 first (on primary layer) - same for all attempts
        print(f"\n  Routing segment 1: Source to Via1 on {primary_layer}...")
        obstacles1 = build_obstacles(pcb_data, net, all_routed_tracks, primary_layer, all_vias)
        path1 = route_path(source, (via1_x, via1_y), obstacles1, max_time=0.25)

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
            obstacles2 = build_obstacles(pcb_data, net, all_routed_tracks, alternate_layer, all_vias)
            path2 = route_path((via1_x, via1_y), (via2_x, via2_y), obstacles2, max_time=0.25)

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
            obstacles3 = build_obstacles(pcb_data, net, all_routed_tracks + seg1_tracks + seg2_tracks, primary_layer, all_vias)
            path3 = route_path((via2_x, via2_y), sink, obstacles3, max_time=0.25)

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
