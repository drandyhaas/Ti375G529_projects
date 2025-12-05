"""
BGA Fanout Strategy - Creates escape routing for BGA packages.

The BGA fanout creates:
1. 45-degree stubs from each pad to a routing channel
2. Horizontal or vertical channel segments to exit the BGA boundary
3. Smart layer assignment to avoid collisions on same channel

Key features:
- Generic: works with any BGA package
- Collision-free: tracks on same channel use different layers
- Assumes pads have vias connecting all layers (so any layer can be used)
- Validates no overlapping segments are created
"""

import math
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional, Set
from collections import defaultdict
import fnmatch

from kicad_parser import parse_kicad_pcb, Pad, Footprint, PCBData, find_components_by_type
from kicad_writer import add_tracks_and_vias_to_pcb


@dataclass
class BGAGrid:
    """Represents the BGA ball grid structure."""
    pitch_x: float
    pitch_y: float
    rows: List[float]  # Sorted Y positions
    cols: List[float]  # Sorted X positions
    center_x: float
    center_y: float
    min_x: float
    max_x: float
    min_y: float
    max_y: float


@dataclass
class Channel:
    """A routing channel between ball rows/columns."""
    orientation: str  # 'horizontal' or 'vertical'
    position: float   # Y for horizontal, X for vertical
    index: int


@dataclass
class FanoutRoute:
    """Complete fanout: pad -> 45° stub -> channel -> exit -> jog."""
    pad: Pad
    pad_pos: Tuple[float, float]
    stub_end: Tuple[float, float]  # Where stub meets channel (or exit for edge pads)
    exit_pos: Tuple[float, float]  # Where route exits BGA
    jog_end: Tuple[float, float] = None  # End of 45° jog after exit
    channel: Optional[Channel] = None  # None for edge pads with direct escape
    escape_dir: str = ''  # 'left', 'right', 'up', 'down'
    is_edge: bool = False  # True for outer row/column pads
    layer: str = "F.Cu"

    @property
    def net_id(self) -> int:
        return self.pad.net_id


def analyze_bga_grid(footprint: Footprint) -> Optional[BGAGrid]:
    """Analyze a footprint to extract BGA grid parameters."""
    pads = footprint.pads
    if len(pads) < 4:
        return None

    x_positions = sorted(set(p.global_x for p in pads))
    y_positions = sorted(set(p.global_y for p in pads))

    if len(x_positions) < 2 or len(y_positions) < 2:
        return None

    x_diffs = [x_positions[i+1] - x_positions[i] for i in range(len(x_positions)-1)]
    y_diffs = [y_positions[i+1] - y_positions[i] for i in range(len(y_positions)-1)]

    def get_dominant_pitch(diffs):
        if not diffs:
            return None
        rounded = [round(d, 2) for d in diffs]
        counts = defaultdict(int)
        for d in rounded:
            counts[d] += 1
        if not counts:
            return None
        dominant = max(counts.keys(), key=lambda k: counts[k])
        if counts[dominant] < len(diffs) * 0.5:
            return None
        return dominant

    pitch_x = get_dominant_pitch(x_diffs)
    pitch_y = get_dominant_pitch(y_diffs)

    if pitch_x is None or pitch_y is None:
        return None

    return BGAGrid(
        pitch_x=pitch_x,
        pitch_y=pitch_y,
        rows=y_positions,
        cols=x_positions,
        center_x=(min(x_positions) + max(x_positions)) / 2,
        center_y=(min(y_positions) + max(y_positions)) / 2,
        min_x=min(x_positions) - pitch_x / 2,
        max_x=max(x_positions) + pitch_x / 2,
        min_y=min(y_positions) - pitch_y / 2,
        max_y=max(y_positions) + pitch_y / 2
    )


def calculate_channels(grid: BGAGrid) -> List[Channel]:
    """Calculate routing channels between ball rows and columns."""
    channels = []
    idx = 0

    for i in range(len(grid.rows) - 1):
        y_pos = (grid.rows[i] + grid.rows[i + 1]) / 2
        channels.append(Channel(orientation='horizontal', position=y_pos, index=idx))
        idx += 1

    for i in range(len(grid.cols) - 1):
        x_pos = (grid.cols[i] + grid.cols[i + 1]) / 2
        channels.append(Channel(orientation='vertical', position=x_pos, index=idx))
        idx += 1

    return channels


def is_edge_pad(pad_x: float, pad_y: float, grid: BGAGrid, tolerance: float = 0.01) -> Tuple[bool, str]:
    """
    Check if a pad is on the outer edge of the BGA.
    Returns (is_edge, escape_direction).

    Edge pads are on the outermost row or column and can escape directly
    without needing a 45° stub.
    """
    on_left = abs(pad_x - grid.cols[0]) < tolerance
    on_right = abs(pad_x - grid.cols[-1]) < tolerance
    on_top = abs(pad_y - grid.rows[0]) < tolerance
    on_bottom = abs(pad_y - grid.rows[-1]) < tolerance

    # Determine escape direction based on which edge
    if on_right:
        return True, 'right'
    elif on_left:
        return True, 'left'
    elif on_bottom:
        return True, 'down'
    elif on_top:
        return True, 'up'

    return False, ''


def find_escape_channel(pad_x: float, pad_y: float,
                        grid: BGAGrid,
                        channels: List[Channel]) -> Tuple[Optional[Channel], str]:
    """
    Find the best channel for a pad to escape through.
    Returns (channel, direction). Channel is None for edge pads.
    """
    # Check if this is an edge pad first
    is_edge, edge_dir = is_edge_pad(pad_x, pad_y, grid)
    if is_edge:
        return None, edge_dir

    dist_left = pad_x - grid.min_x
    dist_right = grid.max_x - pad_x
    dist_up = pad_y - grid.min_y
    dist_down = grid.max_y - pad_y

    min_dist = min(dist_left, dist_right, dist_up, dist_down)

    if min_dist == dist_right:
        escape_dir = 'right'
        h_channels = [c for c in channels if c.orientation == 'horizontal']
        best = min(h_channels, key=lambda c: abs(c.position - pad_y))
        return best, escape_dir
    elif min_dist == dist_left:
        escape_dir = 'left'
        h_channels = [c for c in channels if c.orientation == 'horizontal']
        best = min(h_channels, key=lambda c: abs(c.position - pad_y))
        return best, escape_dir
    elif min_dist == dist_down:
        escape_dir = 'down'
        v_channels = [c for c in channels if c.orientation == 'vertical']
        best = min(v_channels, key=lambda c: abs(c.position - pad_x))
        return best, escape_dir
    else:
        escape_dir = 'up'
        v_channels = [c for c in channels if c.orientation == 'vertical']
        best = min(v_channels, key=lambda c: abs(c.position - pad_x))
        return best, escape_dir


def create_45_stub(pad_x: float, pad_y: float,
                   channel: Channel,
                   escape_dir: str) -> Tuple[float, float]:
    """Create 45-degree stub from pad to channel."""
    if channel.orientation == 'horizontal':
        dy = channel.position - pad_y
        if escape_dir == 'right':
            dx = abs(dy)
        else:
            dx = -abs(dy)
        return (pad_x + dx, channel.position)
    else:
        dx = channel.position - pad_x
        if escape_dir == 'down':
            dy = abs(dx)
        else:
            dy = -abs(dx)
        return (channel.position, pad_y + dy)


def calculate_exit_point(stub_end: Tuple[float, float],
                         channel: Channel,
                         escape_dir: str,
                         grid: BGAGrid,
                         margin: float = 0.5) -> Tuple[float, float]:
    """Calculate where the route exits the BGA boundary."""
    if channel.orientation == 'horizontal':
        if escape_dir == 'right':
            return (grid.max_x + margin, channel.position)
        else:
            return (grid.min_x - margin, channel.position)
    else:
        if escape_dir == 'down':
            return (channel.position, grid.max_y + margin)
        else:
            return (channel.position, grid.min_y - margin)


def calculate_jog_end(exit_pos: Tuple[float, float],
                      escape_dir: str,
                      layer: str,
                      layers: List[str],
                      jog_length: float) -> Tuple[float, float]:
    """
    Calculate the end position of the 45° jog at the exit.

    Jog direction depends on layer:
    - Top layer (F.Cu): 45° to the left (from perspective walking towards BGA edge)
    - Bottom layer (B.Cu): 45° to the right
    - Middle layers: linear interpolation

    Args:
        exit_pos: Starting point of jog
        escape_dir: Direction of escape ('left', 'right', 'up', 'down')
        layer: Current layer
        layers: List of all available layers
        jog_length: Length of the jog (distance from BGA edge to first pad row/col)

    Returns:
        End position of the jog
    """
    # Calculate layer position: 0 = top (left jog), 1 = bottom (right jog)
    try:
        layer_idx = layers.index(layer)
    except ValueError:
        layer_idx = 0

    num_layers = len(layers)
    if num_layers <= 1:
        layer_factor = 0.0  # Default to left jog
    else:
        layer_factor = layer_idx / (num_layers - 1)  # 0 to 1

    # Jog angle: -1 = left, +1 = right (from perspective of walking towards edge)
    # layer_factor 0 (top) -> -1 (left)
    # layer_factor 1 (bottom) -> +1 (right)
    jog_direction = 2 * layer_factor - 1  # Maps 0->-1, 1->+1

    # Calculate jog components based on escape direction
    # At 45°, both components equal jog_length / sqrt(2)
    diag = jog_length / math.sqrt(2)

    ex, ey = exit_pos

    if escape_dir == 'right':
        # Walking right, left is up (-Y), right is down (+Y)
        return (ex + diag, ey + jog_direction * diag)
    elif escape_dir == 'left':
        # Walking left, left is down (+Y), right is up (-Y)
        return (ex - diag, ey - jog_direction * diag)
    elif escape_dir == 'down':
        # Walking down, left is right (+X), right is left (-X)
        return (ex - jog_direction * diag, ey + diag)
    else:  # up
        # Walking up, left is left (-X), right is right (+X)
        return (ex + jog_direction * diag, ey - diag)


def segments_overlap_on_channel(route1: FanoutRoute, route2: FanoutRoute,
                                min_spacing: float) -> bool:
    """
    Check if two routes on the same channel have overlapping channel segments.

    For horizontal channels going right: segments from stub_end.x to exit.x
    The segments overlap if one starts before the other ends.
    """
    if route1.channel.orientation == 'horizontal':
        # Check if the horizontal channel segments overlap
        # Going right: segment is from stub_end.x to exit_pos.x
        # Going left: segment is from exit_pos.x to stub_end.x
        if route1.escape_dir == 'right':
            # Both going right - segment is stub_end.x to exit.x
            # They overlap since they share the same exit point
            # Route closer to exit (higher x) has segment that overlaps with further one
            x1_start, x1_end = route1.stub_end[0], route1.exit_pos[0]
            x2_start, x2_end = route2.stub_end[0], route2.exit_pos[0]
        else:
            # Both going left
            x1_start, x1_end = route1.exit_pos[0], route1.stub_end[0]
            x2_start, x2_end = route2.exit_pos[0], route2.stub_end[0]

        # Segments overlap if max(starts) < min(ends) + spacing
        if max(x1_start, x2_start) < min(x1_end, x2_end) + min_spacing:
            return True
    else:
        # Vertical channel
        if route1.escape_dir == 'down':
            y1_start, y1_end = route1.stub_end[1], route1.exit_pos[1]
            y2_start, y2_end = route2.stub_end[1], route2.exit_pos[1]
        else:
            y1_start, y1_end = route1.exit_pos[1], route1.stub_end[1]
            y2_start, y2_end = route2.exit_pos[1], route2.stub_end[1]

        if max(y1_start, y2_start) < min(y1_end, y2_end) + min_spacing:
            return True

    return False


def assign_layers_smart(routes: List[FanoutRoute],
                        available_layers: List[str],
                        track_width: float,
                        clearance: float) -> None:
    """
    Assign layers to routes to avoid all collisions.

    Strategy:
    - Edge pads stay on first layer (F.Cu) with direct H/V escape
    - Inner pads grouped by channel AND escape direction
    - Routes with overlapping channel segments must use different layers
    """
    min_spacing = track_width + clearance

    # Edge routes stay on first layer
    for route in routes:
        if route.is_edge:
            route.layer = available_layers[0]

    # Group inner routes by (channel_index, escape_direction)
    by_channel_dir: Dict[Tuple[int, str], List[FanoutRoute]] = defaultdict(list)
    for route in routes:
        if route.is_edge:
            continue  # Skip edge pads
        key = (route.channel.index, route.escape_dir)
        by_channel_dir[key].append(route)

    # For each group, assign layers to avoid overlapping channel segments
    for (channel_idx, escape_dir), group_routes in by_channel_dir.items():
        if not group_routes:
            continue

        # Sort routes by position along the channel segment
        # This puts routes in order from inner to outer (or vice versa)
        if group_routes[0].channel.orientation == 'horizontal':
            if escape_dir == 'right':
                # Sort by stub_end X descending - routes closer to exit first
                group_routes.sort(key=lambda r: r.stub_end[0], reverse=True)
            else:
                group_routes.sort(key=lambda r: r.stub_end[0])
        else:
            if escape_dir == 'down':
                group_routes.sort(key=lambda r: r.stub_end[1], reverse=True)
            else:
                group_routes.sort(key=lambda r: r.stub_end[1])

        # Greedy layer assignment
        # For each route, find a layer where it doesn't conflict with already-assigned routes
        for route in group_routes:
            assigned = False

            for layer in available_layers:
                conflict = False
                # Check against all previously assigned routes on this layer in this group
                for other in group_routes:
                    if other is route:
                        break  # Only check routes before this one
                    if other.layer != layer:
                        continue
                    # Check if channel segments overlap
                    if segments_overlap_on_channel(route, other, min_spacing):
                        conflict = True
                        break

                if not conflict:
                    route.layer = layer
                    assigned = True
                    break

            if not assigned:
                # Couldn't find a conflict-free layer - use first layer and warn
                route.layer = available_layers[0]
                print(f"  Warning: Could not find collision-free layer for route at {route.pad_pos}")


def check_segment_collision(seg1_start: Tuple[float, float], seg1_end: Tuple[float, float],
                            seg2_start: Tuple[float, float], seg2_end: Tuple[float, float],
                            clearance: float) -> bool:
    """Check if two segments are too close."""
    # Horizontal segments
    if abs(seg1_start[1] - seg1_end[1]) < 0.001 and abs(seg2_start[1] - seg2_end[1]) < 0.001:
        if abs(seg1_start[1] - seg2_start[1]) < clearance:
            x1_min, x1_max = min(seg1_start[0], seg1_end[0]), max(seg1_start[0], seg1_end[0])
            x2_min, x2_max = min(seg2_start[0], seg2_end[0]), max(seg2_start[0], seg2_end[0])
            if x1_max > x2_min - clearance and x2_max > x1_min - clearance:
                return True

    # Vertical segments
    if abs(seg1_start[0] - seg1_end[0]) < 0.001 and abs(seg2_start[0] - seg2_end[0]) < 0.001:
        if abs(seg1_start[0] - seg2_start[0]) < clearance:
            y1_min, y1_max = min(seg1_start[1], seg1_end[1]), max(seg1_start[1], seg1_end[1])
            y2_min, y2_max = min(seg2_start[1], seg2_end[1]), max(seg2_start[1], seg2_end[1])
            if y1_max > y2_min - clearance and y2_max > y1_min - clearance:
                return True

    # Check endpoint proximity
    for p1 in [seg1_start, seg1_end]:
        for p2 in [seg2_start, seg2_end]:
            dist = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
            if dist < clearance:
                return True

    return False


def generate_bga_fanout(footprint: Footprint,
                        pcb_data: PCBData,
                        net_filter: Optional[List[str]] = None,
                        layers: List[str] = None,
                        track_width: float = 0.1,
                        clearance: float = 0.1,
                        exit_margin: float = 0.5) -> Tuple[List[Dict], List[Dict]]:
    """
    Generate BGA fanout tracks for a footprint.

    Creates:
    1. 45-degree stubs from pads to channels
    2. Channel segments extending to BGA boundary exit

    Args:
        footprint: The BGA footprint
        pcb_data: Full PCB data
        net_filter: Optional list of net patterns to include
        layers: Available routing layers (all connected via pad vias)
        track_width: Width of fanout tracks
        clearance: Minimum clearance between tracks
        exit_margin: How far past BGA boundary to extend

    Returns:
        (tracks, vias) - tracks are the segments, vias is empty (pad vias assumed)
    """
    if layers is None:
        layers = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]

    grid = analyze_bga_grid(footprint)
    if grid is None:
        print(f"Warning: {footprint.reference} doesn't appear to be a BGA")
        return [], []

    print(f"BGA Grid Analysis for {footprint.reference}:")
    print(f"  Pitch: {grid.pitch_x:.2f} x {grid.pitch_y:.2f} mm")
    print(f"  Grid: {len(grid.rows)} rows x {len(grid.cols)} columns")
    print(f"  Center: ({grid.center_x:.2f}, {grid.center_y:.2f})")
    print(f"  Boundary: X[{grid.min_x:.2f}, {grid.max_x:.2f}], Y[{grid.min_y:.2f}, {grid.max_y:.2f}]")

    channels = calculate_channels(grid)
    h_count = len([c for c in channels if c.orientation == 'horizontal'])
    v_count = len([c for c in channels if c.orientation == 'vertical'])
    print(f"  Channels: {h_count} horizontal, {v_count} vertical")
    print(f"  Available layers: {layers}")

    # Build routes
    routes: List[FanoutRoute] = []

    for pad in footprint.pads:
        if not pad.net_name or pad.net_id == 0:
            continue

        if net_filter:
            matched = any(fnmatch.fnmatch(pad.net_name, pattern) for pattern in net_filter)
            if not matched:
                continue

        channel, escape_dir = find_escape_channel(pad.global_x, pad.global_y, grid, channels)

        # Check if this is an edge pad (channel will be None)
        is_edge = channel is None

        if is_edge:
            # Edge pads: direct horizontal/vertical escape to boundary
            pad_pos = (pad.global_x, pad.global_y)
            if escape_dir == 'right':
                exit_pos = (grid.max_x + exit_margin, pad.global_y)
            elif escape_dir == 'left':
                exit_pos = (grid.min_x - exit_margin, pad.global_y)
            elif escape_dir == 'down':
                exit_pos = (pad.global_x, grid.max_y + exit_margin)
            else:  # up
                exit_pos = (pad.global_x, grid.min_y - exit_margin)
            stub_end = exit_pos  # For edge pads, stub goes directly to exit
        else:
            # Inner pads: 45° stub to channel, then channel to exit
            stub_end = create_45_stub(pad.global_x, pad.global_y, channel, escape_dir)
            exit_pos = calculate_exit_point(stub_end, channel, escape_dir, grid, exit_margin)

        route = FanoutRoute(
            pad=pad,
            pad_pos=(pad.global_x, pad.global_y),
            stub_end=stub_end,
            exit_pos=exit_pos,
            channel=channel,
            escape_dir=escape_dir,
            is_edge=is_edge,
            layer=layers[0]  # Edge pads will stay on first layer (F.Cu)
        )
        routes.append(route)

    print(f"  Found {len(routes)} pads to fanout")

    if not routes:
        return [], []

    # Smart layer assignment
    assign_layers_smart(routes, layers, track_width, clearance)

    # Calculate jog length = distance from BGA edge to first pad row/col
    # This is half the pitch (since edge is pitch/2 from first pad)
    jog_length = min(grid.pitch_x, grid.pitch_y) / 2
    print(f"  Jog length: {jog_length:.2f} mm")

    # Calculate jog_end for each route based on layer
    for route in routes:
        route.jog_end = calculate_jog_end(
            route.exit_pos,
            route.escape_dir,
            route.layer,
            layers,
            jog_length
        )

    # Generate tracks
    tracks = []
    edge_count = 0
    inner_count = 0

    for route in routes:
        if route.is_edge:
            # Edge pad: direct segment to exit + jog
            tracks.append({
                'start': route.pad_pos,
                'end': route.exit_pos,
                'width': track_width,
                'layer': route.layer,
                'net_id': route.net_id
            })
            # Add jog segment
            if route.jog_end:
                tracks.append({
                    'start': route.exit_pos,
                    'end': route.jog_end,
                    'width': track_width,
                    'layer': route.layer,
                    'net_id': route.net_id
                })
            edge_count += 1
        else:
            # Inner pad: 45° stub + channel segment + jog
            # Skip zero-length stubs
            dx = abs(route.stub_end[0] - route.pad_pos[0])
            dy = abs(route.stub_end[1] - route.pad_pos[1])
            if dx < 0.001 and dy < 0.001:
                continue

            # 45-degree stub: pad -> channel
            tracks.append({
                'start': route.pad_pos,
                'end': route.stub_end,
                'width': track_width,
                'layer': route.layer,
                'net_id': route.net_id
            })

            # Channel segment: stub_end -> exit
            tracks.append({
                'start': route.stub_end,
                'end': route.exit_pos,
                'width': track_width,
                'layer': route.layer,
                'net_id': route.net_id
            })

            # Jog segment: exit -> jog_end
            if route.jog_end:
                tracks.append({
                    'start': route.exit_pos,
                    'end': route.jog_end,
                    'width': track_width,
                    'layer': route.layer,
                    'net_id': route.net_id
                })
            inner_count += 1

    print(f"  Generated {len(tracks)} track segments")
    print(f"    Edge pads: {edge_count} (direct H/V on {layers[0]})")
    print(f"    Inner pads: {inner_count} (45° stub + channel)")

    # Validate no collisions
    min_spacing = track_width + clearance
    collision_count = 0
    collision_pairs = []

    for i, t1 in enumerate(tracks):
        for j, t2 in enumerate(tracks[i+1:], i+1):
            if t1['layer'] != t2['layer']:
                continue
            if t1['net_id'] == t2['net_id']:
                continue
            if check_segment_collision(t1['start'], t1['end'],
                                       t2['start'], t2['end'],
                                       min_spacing):
                collision_count += 1
                if len(collision_pairs) < 5:
                    collision_pairs.append((t1, t2))

    if collision_count > 0:
        print(f"  WARNING: {collision_count} potential collisions detected!")
        for t1, t2 in collision_pairs:
            print(f"    {t1['layer']} net{t1['net_id']}: {t1['start']}->{t1['end']}")
            print(f"    {t2['layer']} net{t2['net_id']}: {t2['start']}->{t2['end']}")
    else:
        print(f"  Validated: No collisions")

    # Stats by layer
    layer_counts = defaultdict(int)
    for route in routes:
        layer_counts[route.layer] += 1
    for layer, count in sorted(layer_counts.items()):
        print(f"    {layer}: {count} routes")

    return tracks, []


def main():
    """Run BGA fanout generation."""
    import argparse

    parser = argparse.ArgumentParser(description='Generate BGA fanout routing')
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--output', '-o', default='fanout_test.kicad_pcb',
                        help='Output PCB file')
    parser.add_argument('--component', '-c', default=None,
                        help='Component reference (auto-detected if not specified)')
    parser.add_argument('--layers', '-l', nargs='+', default=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
                        help='Routing layers')
    parser.add_argument('--width', '-w', type=float, default=0.1,
                        help='Track width in mm')
    parser.add_argument('--clearance', type=float, default=0.1,
                        help='Track clearance in mm')
    parser.add_argument('--nets', '-n', nargs='*',
                        help='Net patterns to include')
    parser.add_argument('--exit-margin', type=float, default=0.5,
                        help='Distance past BGA boundary')

    args = parser.parse_args()

    print(f"Parsing {args.pcb}...")
    pcb_data = parse_kicad_pcb(args.pcb)

    # Auto-detect BGA component if not specified
    if args.component is None:
        bga_components = find_components_by_type(pcb_data, 'BGA')
        if bga_components:
            args.component = bga_components[0].reference
            print(f"Auto-detected BGA component: {args.component}")
            if len(bga_components) > 1:
                print(f"  (Other BGAs found: {[fp.reference for fp in bga_components[1:]]})")
        else:
            print("Error: No BGA components found in PCB")
            print(f"Available components: {list(pcb_data.footprints.keys())[:20]}...")
            return 1

    if args.component not in pcb_data.footprints:
        print(f"Error: Component {args.component} not found")
        print(f"Available: {list(pcb_data.footprints.keys())[:20]}...")
        return 1

    footprint = pcb_data.footprints[args.component]
    print(f"\nFound {args.component}: {footprint.footprint_name}")
    print(f"  Position: ({footprint.x:.2f}, {footprint.y:.2f})")
    print(f"  Rotation: {footprint.rotation}deg")
    print(f"  Pads: {len(footprint.pads)}")

    tracks, vias = generate_bga_fanout(
        footprint,
        pcb_data,
        net_filter=args.nets,
        layers=args.layers,
        track_width=args.width,
        clearance=args.clearance,
        exit_margin=args.exit_margin
    )

    if tracks:
        print(f"\nWriting {len(tracks)} tracks to {args.output}...")
        add_tracks_and_vias_to_pcb(args.pcb, args.output, tracks, vias)
        print("Done!")
    else:
        print("\nNo fanout tracks generated")

    return 0


if __name__ == '__main__':
    exit(main())
