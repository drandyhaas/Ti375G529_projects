#!/usr/bin/env python3
"""
Test the simple router on actual PCB data and output to KiCad.
"""

import sys
import math
from pathlib import Path
from simple_router import route_single_net, route_bidirectional, simplify_path, simplify_path_safe, create_svg_visualization, Obstacle, optimize_simplified_path, snap_to_45_degrees, verify_path, enforce_45_degree_path
from kicad_parser import parse_kicad_pcb, get_nets_to_route
from kicad_writer import add_tracks_to_pcb


def main():
    pcb_path = "../haasoscope_pro_max/haasoscope_pro_max.kicad_pcb"
    output_path = "routed.kicad_pcb"

    # Net pattern - use command line arg or default
    net_pattern = sys.argv[1] if len(sys.argv) > 1 else "Net-(U2A-DATA_1)"

    print(f"Parsing {pcb_path}...")
    pcb_data = parse_kicad_pcb(pcb_path)

    # Get nets to route
    nets = get_nets_to_route(pcb_data, net_patterns=[net_pattern])

    if not nets:
        print(f"No nets found matching '{net_pattern}'!")
        return

    net = nets[0]
    print(f"\nRouting net: {net.name}")
    print(f"  Pads: {len(net.pads)}")

    # Get source and sink positions
    first_pad = net.pads[0]
    second_pad = net.pads[1]

    source_x = getattr(first_pad, 'global_x', 0)
    source_y = getattr(first_pad, 'global_y', 0)
    sink_x = getattr(second_pad, 'global_x', 0)
    sink_y = getattr(second_pad, 'global_y', 0)

    print(f"  Source: ({source_x:.2f}, {source_y:.2f}) - {first_pad.component_ref}.{first_pad.pad_number}")
    print(f"  Sink: ({sink_x:.2f}, {sink_y:.2f}) - {second_pad.component_ref}.{second_pad.pad_number}")

    # Build obstacles from ALL pads except this net's pads
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
                # For circles, use a square that inscribes the circle
                corner_radius = min(width, height) / 2
            elif shape == 'roundrect':
                # Rounded rectangle - estimate 25% corner radius
                corner_radius = min(width, height) * 0.25
            else:
                # Rectangle - no corner radius
                corner_radius = 0

            obstacles.append(Obstacle(px, py, width, height, rotation, corner_radius))

    # Add via obstacles (except this net's vias) - vias are circular
    for via in pcb_data.vias:
        if via.net_id != net.net_id:
            # Vias are circular, so use equal width/height
            obstacles.append(Obstacle(via.x, via.y, via.size, via.size, 0, via.size/2))

    # Add existing track segments as obstacles
    for track in pcb_data.segments:
        if track.net_id != net.net_id:
            # Add rectangular obstacles along the track segment
            start_x, start_y = track.start_x, track.start_y
            end_x, end_y = track.end_x, track.end_y
            track_width = track.width

            # Calculate track angle for rotation
            dx = end_x - start_x
            dy = end_y - start_y
            length = (dx*dx + dy*dy)**0.5
            if length > 0:
                # Track angle in degrees
                track_angle = math.degrees(math.atan2(dy, dx))

                # Add rectangular obstacles along the segment
                num_points = max(2, int(length / 0.2))  # Point every 0.2mm
                for j in range(num_points + 1):
                    t = j / num_points
                    px = start_x + t * dx
                    py = start_y + t * dy
                    # Rectangle along track direction
                    obstacles.append(Obstacle(px, py, 0.25, track_width, track_angle, track_width/2))

    print(f"  Total obstacles: {len(obstacles)}")

    # Calculate bounds
    all_x = [source_x, sink_x] + [o.x for o in obstacles]
    all_y = [source_y, sink_y] + [o.y for o in obstacles]
    margin = 2.0
    bounds = (min(all_x) - margin, min(all_y) - margin,
              max(all_x) + margin, max(all_y) + margin)

    print(f"  Bounds: ({bounds[0]:.1f}, {bounds[1]:.1f}) to ({bounds[2]:.1f}, {bounds[3]:.1f})")

    # Route!
    clearance = 0.05  # Very tight clearance for BGA escape
    step_size = 0.1   # Small steps for precision
    track_width = 0.1

    print(f"\nRouting with step_size={step_size}, clearance={clearance}...")
    print("Using bidirectional routing (escape from both ends, meet in middle)...")
    path = route_bidirectional(
        source=(source_x, source_y),
        sink=(sink_x, sink_y),
        obstacles=obstacles,
        step_size=step_size,
        clearance=clearance,
        max_time=10.0  # More time for complex routes
    )

    if path:
        print(f"Success! Path has {len(path)} points")

        # Step 1: Simplify using collision-aware simplification
        print("Simplifying path (collision-aware)...")
        simplified = simplify_path_safe(path, obstacles, clearance, tolerance=0.1)
        print(f"Simplified to {len(simplified)} points")

        # Step 2: Enforce exact 45-degree angles
        print("Enforcing 45-degree angles...")
        final_path = enforce_45_degree_path(simplified, obstacles, clearance)
        print(f"After 45-deg enforcement: {len(final_path)} points")

        # Calculate path length
        total_length = 0
        for i in range(1, len(final_path)):
            dx = final_path[i][0] - final_path[i-1][0]
            dy = final_path[i][1] - final_path[i-1][1]
            total_length += (dx*dx + dy*dy)**0.5
        print(f"Path length: {total_length:.2f} mm")

        # Use final_path instead of simplified
        simplified = final_path

        # Create visualization
        svg = create_svg_visualization(simplified, obstacles,
                                       (source_x, source_y), (sink_x, sink_y),
                                       bounds, clearance)

        with open("route.svg", 'w') as f:
            f.write(svg)
        print(f"Saved visualization to route.svg")

        # Write report
        with open("route.txt", 'w') as f:
            f.write(f"Net: {net.name}\n")
            f.write(f"Source: ({source_x:.2f}, {source_y:.2f}) - {first_pad.component_ref}.{first_pad.pad_number}\n")
            f.write(f"Sink: ({sink_x:.2f}, {sink_y:.2f}) - {second_pad.component_ref}.{second_pad.pad_number}\n")
            f.write(f"Path points: {len(simplified)}\n")
            f.write(f"Path length: {total_length:.2f} mm\n")
            f.write(f"Obstacles: {len(obstacles)}\n")
        print(f"Saved report to route.txt")

        # Output to KiCad
        # Convert path to track segments
        tracks = []
        layer = "F.Cu"  # Route on front copper
        for i in range(1, len(simplified)):
            tracks.append({
                'start': simplified[i-1],
                'end': simplified[i],
                'width': track_width,
                'layer': layer,
                'net_id': net.net_id
            })

        print(f"\nWriting {len(tracks)} track segments to {output_path}...")
        add_tracks_to_pcb(pcb_path, output_path, tracks)
        print(f"Saved routed PCB to {output_path}")

    else:
        print("Failed to find path!")

        # Create visualization anyway to see obstacles
        svg = create_svg_visualization([], obstacles,
                                       (source_x, source_y), (sink_x, sink_y),
                                       bounds, clearance)
        with open("route.svg", 'w') as f:
            f.write(svg)
        print("Saved obstacle visualization to route.svg")


if __name__ == '__main__':
    main()
