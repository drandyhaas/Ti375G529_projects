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
- Differential pair support: P/N pairs routed together on same layer
"""

import math
import re
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional, Set
from collections import defaultdict
import fnmatch

from kicad_parser import parse_kicad_pcb, Pad, Footprint, PCBData, find_components_by_type
from kicad_writer import add_tracks_and_vias_to_pcb


@dataclass
class DiffPair:
    """A differential pair of pads (P and N)."""
    base_name: str  # Common name without _P/_N suffix
    p_pad: Optional[Pad] = None
    n_pad: Optional[Pad] = None

    @property
    def is_complete(self) -> bool:
        return self.p_pad is not None and self.n_pad is not None


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
    jog_extension: Tuple[float, float] = None  # Extension point for outside track of diff pair
    channel_point: Tuple[float, float] = None  # First channel point for half-edge inner pads (45° entry)
    channel_point2: Tuple[float, float] = None  # Second channel point (after horizontal segment)
    channel: Optional[Channel] = None  # None for edge pads with direct escape
    escape_dir: str = ''  # 'left', 'right', 'up', 'down'
    is_edge: bool = False  # True for outer row/column pads
    layer: str = "F.Cu"
    pair_id: Optional[str] = None  # Differential pair base name if part of a pair
    is_p: bool = True  # True for P, False for N in differential pair

    @property
    def net_id(self) -> int:
        return self.pad.net_id


def extract_diff_pair_base(net_name: str) -> Optional[Tuple[str, bool]]:
    """
    Extract differential pair base name and polarity from net name.

    Looks for common differential pair naming conventions:
    - name_P / name_N
    - nameP / nameN
    - name+ / name-

    Returns (base_name, is_positive) or None if not a diff pair.
    """
    if not net_name:
        return None

    # Try _P/_N suffix (most common for LVDS)
    if net_name.endswith('_P'):
        return (net_name[:-2], True)
    if net_name.endswith('_N'):
        return (net_name[:-2], False)

    # Try P/N suffix without underscore
    if net_name.endswith('P') and len(net_name) > 1:
        # Check it's not just ending in P as part of name
        if net_name[-2] in '0123456789_':
            return (net_name[:-1], True)
    if net_name.endswith('N') and len(net_name) > 1:
        if net_name[-2] in '0123456789_':
            return (net_name[:-1], False)

    # Try +/- suffix
    if net_name.endswith('+'):
        return (net_name[:-1], True)
    if net_name.endswith('-'):
        return (net_name[:-1], False)

    return None


def find_differential_pairs(footprint: Footprint,
                           diff_pair_patterns: List[str]) -> Dict[str, DiffPair]:
    """
    Find all differential pairs in a footprint matching the given patterns.

    Args:
        footprint: The footprint to search
        diff_pair_patterns: Glob patterns for nets to treat as diff pairs

    Returns:
        Dict mapping base_name to DiffPair
    """
    pairs: Dict[str, DiffPair] = {}

    for pad in footprint.pads:
        if not pad.net_name or pad.net_id == 0:
            continue

        # Check if this net matches any diff pair pattern
        matched = any(fnmatch.fnmatch(pad.net_name, pattern)
                     for pattern in diff_pair_patterns)
        if not matched:
            continue

        # Try to extract diff pair info
        result = extract_diff_pair_base(pad.net_name)
        if result is None:
            continue

        base_name, is_p = result

        if base_name not in pairs:
            pairs[base_name] = DiffPair(base_name=base_name)

        if is_p:
            pairs[base_name].p_pad = pad
        else:
            pairs[base_name].n_pad = pad

    # Filter to only complete pairs
    complete_pairs = {k: v for k, v in pairs.items() if v.is_complete}

    return complete_pairs


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


def get_pair_escape_options(p_pad_x: float, p_pad_y: float,
                             n_pad_x: float, n_pad_y: float,
                             grid: BGAGrid,
                             channels: List[Channel]) -> List[Tuple[Optional[Channel], str]]:
    """
    Get all valid escape options for a differential pair, ordered by preference.

    Returns list of (channel, direction) tuples, best options first.
    Returns empty list for edge/half-edge pairs (they have fixed escape).
    """
    center_x = (p_pad_x + n_pad_x) / 2
    center_y = (p_pad_y + n_pad_y) / 2

    # Check if this is an edge pair - no options, fixed escape
    is_edge_p, _ = is_edge_pad(p_pad_x, p_pad_y, grid)
    is_edge_n, _ = is_edge_pad(n_pad_x, n_pad_y, grid)
    if is_edge_p or is_edge_n:
        return []  # Edge pairs have fixed escape direction

    # Calculate distances to each edge
    dist_left = center_x - grid.min_x
    dist_right = grid.max_x - center_x
    dist_up = center_y - grid.min_y
    dist_down = grid.max_y - center_y

    min_pad_y = min(p_pad_y, n_pad_y)
    max_pad_y = max(p_pad_y, n_pad_y)
    min_pad_x = min(p_pad_x, n_pad_x)
    max_pad_x = max(p_pad_x, n_pad_x)

    options = []

    # Horizontal escape options (left/right)
    h_channels = [c for c in channels if c.orientation == 'horizontal']
    channels_above = [c for c in h_channels if c.position < min_pad_y]
    channels_below = [c for c in h_channels if c.position > max_pad_y]

    # Best horizontal channel
    if dist_up <= dist_down and channels_above:
        h_channel = max(channels_above, key=lambda c: c.position)
    elif channels_below:
        h_channel = min(channels_below, key=lambda c: c.position)
    elif channels_above:
        h_channel = max(channels_above, key=lambda c: c.position)
    else:
        h_channel = min(h_channels, key=lambda c: abs(c.position - center_y)) if h_channels else None

    if h_channel:
        h_dir = 'left' if dist_left <= dist_right else 'right'
        h_dist = min(dist_left, dist_right)
        options.append((h_channel, h_dir, h_dist))

    # Vertical escape options (up/down)
    v_channels = [c for c in channels if c.orientation == 'vertical']
    channels_left = [c for c in v_channels if c.position < min_pad_x]
    channels_right = [c for c in v_channels if c.position > max_pad_x]

    # Best vertical channel
    if dist_left <= dist_right and channels_left:
        v_channel = max(channels_left, key=lambda c: c.position)
    elif channels_right:
        v_channel = min(channels_right, key=lambda c: c.position)
    elif channels_left:
        v_channel = max(channels_left, key=lambda c: c.position)
    else:
        v_channel = min(v_channels, key=lambda c: abs(c.position - center_x)) if v_channels else None

    if v_channel:
        v_dir = 'up' if dist_up <= dist_down else 'down'
        v_dist = min(dist_up, dist_down)
        options.append((v_channel, v_dir, v_dist))

    # Sort by distance (closest edge first)
    options.sort(key=lambda x: x[2])

    # Return without the distance
    return [(ch, d) for ch, d, _ in options]


def find_diff_pair_escape(p_pad_x: float, p_pad_y: float,
                          n_pad_x: float, n_pad_y: float,
                          grid: BGAGrid,
                          channels: List[Channel],
                          preferred_orientation: str = 'auto') -> Tuple[Optional[Channel], str]:
    """
    Find the best escape channel for a differential pair.

    Args:
        preferred_orientation: 'horizontal', 'vertical', or 'auto'
            - 'horizontal': prefer left/right escape
            - 'vertical': prefer up/down escape
            - 'auto': choose based on closest edge

    Returns (channel, direction). Channel is None for edge pads.
    """
    # Calculate pair center and orientation
    center_x = (p_pad_x + n_pad_x) / 2
    center_y = (p_pad_y + n_pad_y) / 2

    # Check if this is an edge pair
    is_edge_p, edge_dir_p = is_edge_pad(p_pad_x, p_pad_y, grid)
    is_edge_n, edge_dir_n = is_edge_pad(n_pad_x, n_pad_y, grid)

    if is_edge_p and is_edge_n:
        # Both pads on edge - use the common edge direction
        return None, edge_dir_p

    # Check for "half-edge" pair: one pad on edge, one inner
    if is_edge_p or is_edge_n:
        edge_dir = edge_dir_p if is_edge_p else edge_dir_n
        return None, f'half_edge_{edge_dir}'

    # Get all escape options
    options = get_pair_escape_options(p_pad_x, p_pad_y, n_pad_x, n_pad_y, grid, channels)

    if not options:
        # Fallback - shouldn't happen for inner pairs
        return None, 'right'

    if preferred_orientation == 'auto':
        # Use first (best) option
        return options[0]
    elif preferred_orientation == 'horizontal':
        # Prefer horizontal (left/right)
        for ch, d in options:
            if d in ['left', 'right']:
                return ch, d
        return options[0]  # Fallback
    elif preferred_orientation == 'vertical':
        # Prefer vertical (up/down)
        for ch, d in options:
            if d in ['up', 'down']:
                return ch, d
        return options[0]  # Fallback
    else:
        return options[0]


def assign_pair_escapes(diff_pairs: Dict[str, DiffPair],
                        grid: BGAGrid,
                        channels: List[Channel],
                        layers: List[str],
                        primary_orientation: str = 'horizontal',
                        track_width: float = 0.1,
                        clearance: float = 0.1) -> Dict[str, Tuple[Optional[Channel], str]]:
    """
    Assign escape directions to all differential pairs, avoiding overlaps.

    Strategy:
    1. Sort pairs by distance to nearest edge (closest first - they have fewer options)
    2. Try primary orientation first for each pair
    3. If channel/layer is occupied, try secondary orientation
    4. Track channel occupancy per layer
    5. After assignment, try to balance if one direction is overpopulated

    Args:
        diff_pairs: Dictionary of pair_id -> DiffPair
        grid: BGA grid info
        channels: Available channels
        layers: Available routing layers
        primary_orientation: 'horizontal' or 'vertical'
        track_width: Track width for spacing calculation
        clearance: Clearance for spacing calculation

    Returns:
        Dictionary of pair_id -> (channel, escape_direction)
    """
    # Track which exit positions are used per layer
    # Key: (layer, exit_x or exit_y rounded to 0.1mm)
    # For horizontal escape: track exit Y positions
    # For vertical escape: track exit X positions
    used_exits: Dict[Tuple[str, str, float], Set[str]] = defaultdict(set)  # (layer, 'h'/'v', pos) -> set of pair_ids

    pair_spacing = track_width * 2 + clearance  # Space needed for a diff pair

    assignments: Dict[str, Tuple[Optional[Channel], str]] = {}

    # Collect pair info with distances
    pair_info = []
    for pair_id, pair in diff_pairs.items():
        if not pair.is_complete:
            continue
        p_pad = pair.p_pad
        n_pad = pair.n_pad
        center_x = (p_pad.global_x + n_pad.global_x) / 2
        center_y = (p_pad.global_y + n_pad.global_y) / 2

        # Check if edge pair
        is_edge_p, edge_dir_p = is_edge_pad(p_pad.global_x, p_pad.global_y, grid)
        is_edge_n, edge_dir_n = is_edge_pad(n_pad.global_x, n_pad.global_y, grid)

        if is_edge_p and is_edge_n:
            # Both on edge - fixed assignment
            assignments[pair_id] = (None, edge_dir_p)
            continue
        elif is_edge_p or is_edge_n:
            # Half-edge - fixed assignment
            edge_dir = edge_dir_p if is_edge_p else edge_dir_n
            assignments[pair_id] = (None, f'half_edge_{edge_dir}')
            continue

        # Calculate distance to nearest edge
        dist_left = center_x - grid.min_x
        dist_right = grid.max_x - center_x
        dist_up = center_y - grid.min_y
        dist_down = grid.max_y - center_y
        min_dist = min(dist_left, dist_right, dist_up, dist_down)

        pair_info.append((pair_id, pair, min_dist, center_x, center_y))

    # Sort by distance to edge (closest first - they have fewer routing options)
    pair_info.sort(key=lambda x: x[2])

    secondary_orientation = 'vertical' if primary_orientation == 'horizontal' else 'horizontal'

    def try_assign(pair_id: str, pair: DiffPair, orientation: str) -> bool:
        """Try to assign a pair to the given orientation. Returns True if successful."""
        p_pad = pair.p_pad
        n_pad = pair.n_pad
        center_x = (p_pad.global_x + n_pad.global_x) / 2
        center_y = (p_pad.global_y + n_pad.global_y) / 2

        channel, escape_dir = find_diff_pair_escape(
            p_pad.global_x, p_pad.global_y,
            n_pad.global_x, n_pad.global_y,
            grid, channels, orientation
        )

        if channel is None:
            return False

        # Check if this conflicts with existing assignments
        # For cross-escape (convergence), check the exit position
        pads_horizontal = abs(p_pad.global_x - n_pad.global_x) > abs(p_pad.global_y - n_pad.global_y)
        is_cross = (pads_horizontal and escape_dir in ['up', 'down']) or \
                   (not pads_horizontal and escape_dir in ['left', 'right'])

        if is_cross:
            # Convergence routing - exits at center +/- half_pair_spacing
            if escape_dir in ['up', 'down']:
                # Vertical exit - check X positions
                exit_pos = round(center_x, 1)
                pos_key = 'v'
            else:
                # Horizontal exit - check Y positions
                exit_pos = round(center_y, 1)
                pos_key = 'h'
        else:
            # Channel routing - exits at channel position +/- offset
            if escape_dir in ['left', 'right']:
                exit_pos = round(channel.position, 1)
                pos_key = 'h'
            else:
                exit_pos = round(channel.position, 1)
                pos_key = 'v'

        # Check all layers for conflicts
        # A position is available if there's at least one free layer
        conflicts_per_layer = {}
        for layer in layers:
            key = (layer, pos_key, exit_pos)
            nearby_pairs = used_exits[key]
            # Also check adjacent positions for spacing
            for delta in [-pair_spacing, 0, pair_spacing]:
                adj_key = (layer, pos_key, round(exit_pos + delta, 1))
                nearby_pairs = nearby_pairs.union(used_exits[adj_key])
            conflicts_per_layer[layer] = len(nearby_pairs)

        # Find layer with fewest conflicts
        best_layer = min(layers, key=lambda l: conflicts_per_layer[l])
        if conflicts_per_layer[best_layer] >= 1:
            # All layers have conflicts at this position
            return False

        # Assign this pair
        assignments[pair_id] = (channel, escape_dir)
        key = (best_layer, pos_key, exit_pos)
        used_exits[key].add(pair_id)
        return True

    # First pass: try primary orientation
    unassigned = []
    for pair_id, pair, min_dist, cx, cy in pair_info:
        if not try_assign(pair_id, pair, primary_orientation):
            unassigned.append((pair_id, pair, min_dist, cx, cy))

    # Second pass: try secondary orientation for unassigned
    still_unassigned = []
    for pair_id, pair, min_dist, cx, cy in unassigned:
        if not try_assign(pair_id, pair, secondary_orientation):
            still_unassigned.append((pair_id, pair, min_dist, cx, cy))

    # Third pass: force assign remaining (they'll have collisions)
    for pair_id, pair, min_dist, cx, cy in still_unassigned:
        # Use auto mode as fallback
        channel, escape_dir = find_diff_pair_escape(
            pair.p_pad.global_x, pair.p_pad.global_y,
            pair.n_pad.global_x, pair.n_pad.global_y,
            grid, channels, 'auto'
        )
        assignments[pair_id] = (channel, escape_dir)

    # Count direction distribution
    dir_counts = defaultdict(int)
    for ch, d in assignments.values():
        if d.startswith('half_edge_'):
            d = d.replace('half_edge_', '')
        dir_counts[d] += 1

    h_count = dir_counts['left'] + dir_counts['right']
    v_count = dir_counts['up'] + dir_counts['down']

    print(f"    Initial assignment: horizontal={h_count}, vertical={v_count}")

    # Try to balance if very uneven (>2:1 ratio)
    if h_count > 0 and v_count > 0:
        ratio = max(h_count, v_count) / min(h_count, v_count)
        if ratio > 2.0:
            # Find pairs to switch from overpopulated to underpopulated direction
            overpopulated = 'horizontal' if h_count > v_count else 'vertical'
            underpopulated = 'vertical' if h_count > v_count else 'horizontal'

            # Find switchable pairs (farthest from their current exit edge)
            switchable = []
            for pair_id, pair, min_dist, cx, cy in pair_info:
                if pair_id not in assignments:
                    continue
                ch, d = assignments[pair_id]
                if d.startswith('half_edge_'):
                    continue  # Can't switch edge pairs
                if ch is None:
                    continue

                current_is_horiz = d in ['left', 'right']
                if (overpopulated == 'horizontal') == current_is_horiz:
                    # This pair is in the overpopulated direction
                    # Calculate distance to current exit edge
                    if d == 'left':
                        exit_dist = cx - grid.min_x
                    elif d == 'right':
                        exit_dist = grid.max_x - cx
                    elif d == 'up':
                        exit_dist = cy - grid.min_y
                    else:  # down
                        exit_dist = grid.max_y - cy
                    switchable.append((pair_id, pair, exit_dist, cx, cy))

            # Sort by exit distance (farthest first - they benefit most from switching)
            switchable.sort(key=lambda x: -x[2])

            # Try to switch some pairs
            target_switch = abs(h_count - v_count) // 2
            switched = 0
            for pair_id, pair, exit_dist, cx, cy in switchable:
                if switched >= target_switch:
                    break
                # Try to assign to underpopulated direction
                old_assignment = assignments[pair_id]
                if try_assign(pair_id, pair, underpopulated):
                    switched += 1
                else:
                    # Restore old assignment
                    assignments[pair_id] = old_assignment

            if switched > 0:
                print(f"    Balanced: switched {switched} pairs to {underpopulated}")

    return assignments


def create_45_stub(pad_x: float, pad_y: float,
                   channel: Channel,
                   escape_dir: str,
                   channel_offset: float = 0.0) -> Tuple[float, float]:
    """
    Create 45-degree stub from pad to channel.

    For differential pairs, the offset is applied AFTER calculating the 45° stub
    based on the channel center. This ensures both P and N travel the same
    horizontal/vertical distance before applying the parallel offset.

    Args:
        pad_x, pad_y: Pad position
        channel: Target channel
        escape_dir: Direction of escape
        channel_offset: Offset from channel center (for diff pairs)

    Returns:
        End position of the 45° stub
    """
    if channel.orientation == 'horizontal':
        # Calculate 45° stub to channel CENTER first
        dy_to_center = channel.position - pad_y
        if escape_dir == 'right':
            dx = abs(dy_to_center)
        else:
            dx = -abs(dy_to_center)

        # Apply offset to final Y position (parallel offset)
        target_y = channel.position + channel_offset
        return (pad_x + dx, target_y)
    else:
        # Calculate 45° stub to channel CENTER first
        dx_to_center = channel.position - pad_x
        if escape_dir == 'down':
            dy = abs(dx_to_center)
        else:
            dy = -abs(dx_to_center)

        # Apply offset to final X position (parallel offset)
        target_x = channel.position + channel_offset
        return (target_x, pad_y + dy)


def calculate_exit_point(stub_end: Tuple[float, float],
                         channel: Channel,
                         escape_dir: str,
                         grid: BGAGrid,
                         margin: float = 0.5,
                         channel_offset: float = 0.0) -> Tuple[float, float]:
    """Calculate where the route exits the BGA boundary."""
    if channel.orientation == 'horizontal':
        exit_y = channel.position + channel_offset
        if escape_dir == 'right':
            return (grid.max_x + margin, exit_y)
        else:
            return (grid.min_x - margin, exit_y)
    else:
        exit_x = channel.position + channel_offset
        if escape_dir == 'down':
            return (exit_x, grid.max_y + margin)
        else:
            return (exit_x, grid.min_y - margin)


def calculate_jog_end(exit_pos: Tuple[float, float],
                      escape_dir: str,
                      layer: str,
                      layers: List[str],
                      jog_length: float,
                      is_diff_pair: bool = False,
                      is_outside_track: bool = False,
                      pair_spacing: float = 0.0) -> Tuple[Tuple[float, float], Optional[Tuple[float, float]]]:
    """
    Calculate the end position of the 45° jog at the exit.

    For differential pairs, the outside track needs to extend further before
    bending to maintain constant spacing through the 45° turn.

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
        is_diff_pair: Whether this is part of a differential pair
        is_outside_track: Whether this is the outside track of the pair (needs extension)
        pair_spacing: Spacing between P and N tracks

    Returns:
        (jog_end, extension_point) - extension_point is the intermediate point for outside tracks
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
    extension_point = None

    # For differential pairs, outside track extends further before bending
    # To maintain constant perpendicular spacing through a 45° turn:
    # extension = pair_spacing * (sqrt(2) - 1) ≈ 0.414 * pair_spacing
    if is_diff_pair and is_outside_track:
        extension = pair_spacing * (math.sqrt(2) - 1)
        if escape_dir == 'right':
            extension_point = (ex + extension, ey)
            ex = ex + extension
        elif escape_dir == 'left':
            extension_point = (ex - extension, ey)
            ex = ex - extension
        elif escape_dir == 'down':
            extension_point = (ex, ey + extension)
            ey = ey + extension
        else:  # up
            extension_point = (ex, ey - extension)
            ey = ey - extension

    if escape_dir == 'right':
        # Walking right, left is up (-Y), right is down (+Y)
        jog_end = (ex + diag, ey + jog_direction * diag)
    elif escape_dir == 'left':
        # Walking left, left is down (+Y), right is up (-Y)
        jog_end = (ex - diag, ey - jog_direction * diag)
    elif escape_dir == 'down':
        # Walking down, left is right (+X), right is left (-X)
        jog_end = (ex - jog_direction * diag, ey + diag)
    else:  # up
        # Walking up, left is left (-X), right is right (+X)
        jog_end = (ex + jog_direction * diag, ey - diag)

    return jog_end, extension_point


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
                        clearance: float,
                        diff_pair_spacing: float = 0.0) -> None:
    """
    Assign layers to routes to avoid all collisions.

    Strategy:
    - Edge pads stay on first layer (F.Cu) with direct H/V escape
    - Differential pairs are assigned to the same layer together
    - Inner pads grouped by channel AND escape direction
    - Routes with overlapping channel segments must use different layers
    """
    min_spacing = track_width + clearance

    # For diff pairs, we need to consider pair spacing when checking collisions
    # Two traces from the same pair use 2*track_width + diff_pair_spacing
    pair_width = 2 * track_width + diff_pair_spacing

    # Edge routes stay on first layer
    for route in routes:
        if route.is_edge:
            route.layer = available_layers[0]

    # Build lookup from pair_id to routes
    pair_routes: Dict[str, List[FanoutRoute]] = defaultdict(list)
    for route in routes:
        if route.pair_id:
            pair_routes[route.pair_id].append(route)

    # Group inner routes by (channel_index, escape_direction)
    by_channel_dir: Dict[Tuple[int, str], List[FanoutRoute]] = defaultdict(list)
    for route in routes:
        if route.is_edge:
            continue  # Skip edge pads
        key = (route.channel.index, route.escape_dir)
        by_channel_dir[key].append(route)

    # Track which pairs have been assigned
    assigned_pairs: Set[str] = set()

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
            # If this route is part of a pair that's already assigned, use that layer
            if route.pair_id and route.pair_id in assigned_pairs:
                # Find the other route in the pair and use its layer
                for other in pair_routes[route.pair_id]:
                    if other is not route and other.layer:
                        route.layer = other.layer
                        break
                continue

            assigned = False

            for layer in available_layers:
                conflict = False
                # Check against all previously assigned routes on this layer in this group
                for other in group_routes:
                    if other is route:
                        break  # Only check routes before this one
                    if other.layer != layer:
                        continue
                    # Skip collision check for same diff pair (they route together)
                    if route.pair_id and other.pair_id == route.pair_id:
                        continue
                    # Check if channel segments overlap
                    # Use pair_width for spacing if either route is a diff pair
                    spacing = min_spacing
                    if route.pair_id or other.pair_id:
                        spacing = pair_width + clearance
                    if segments_overlap_on_channel(route, other, spacing):
                        conflict = True
                        break

                if not conflict:
                    route.layer = layer
                    assigned = True
                    # If this is a diff pair, mark it as assigned and set partner's layer
                    if route.pair_id:
                        assigned_pairs.add(route.pair_id)
                        for partner in pair_routes[route.pair_id]:
                            if partner is not route:
                                partner.layer = layer
                    break

            if not assigned:
                # Couldn't find a conflict-free layer - use first layer and warn
                route.layer = available_layers[0]
                print(f"  Warning: Could not find collision-free layer for route at {route.pad_pos}")
                if route.pair_id:
                    assigned_pairs.add(route.pair_id)
                    for partner in pair_routes[route.pair_id]:
                        if partner is not route:
                            partner.layer = available_layers[0]


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
                        diff_pair_patterns: Optional[List[str]] = None,
                        layers: List[str] = None,
                        track_width: float = 0.1,
                        clearance: float = 0.1,
                        diff_pair_gap: float = 0.1,
                        exit_margin: float = 0.5,
                        primary_escape: str = 'horizontal') -> Tuple[List[Dict], List[Dict]]:
    """
    Generate BGA fanout tracks for a footprint.

    Creates:
    1. 45-degree stubs from pads to channels
    2. Channel segments extending to BGA boundary exit
    3. Differential pairs routed together on same layer

    Args:
        footprint: The BGA footprint
        pcb_data: Full PCB data
        net_filter: Optional list of net patterns to include
        diff_pair_patterns: Glob patterns for differential pair nets (e.g., '*lvds*')
        layers: Available routing layers (all connected via pad vias)
        track_width: Width of fanout tracks
        clearance: Minimum clearance between tracks
        diff_pair_gap: Gap between P and N traces of a differential pair
        exit_margin: How far past BGA boundary to extend
        primary_escape: Primary escape direction preference ('horizontal' or 'vertical')

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

    # Find differential pairs if patterns specified
    diff_pairs: Dict[str, DiffPair] = {}
    pair_escape_assignments: Dict[str, Tuple[Optional[Channel], str]] = {}
    if diff_pair_patterns:
        diff_pairs = find_differential_pairs(footprint, diff_pair_patterns)
        print(f"  Found {len(diff_pairs)} differential pairs")

        # Pre-assign escape directions for all pairs to avoid overlaps
        print(f"  Assigning escape directions (primary: {primary_escape})...")
        pair_escape_assignments = assign_pair_escapes(
            diff_pairs, grid, channels, layers,
            primary_orientation=primary_escape,
            track_width=track_width,
            clearance=clearance
        )

    # Build lookup from net_name to pair info
    net_to_pair: Dict[str, Tuple[str, bool]] = {}  # net_name -> (pair_id, is_p)
    for pair_id, pair in diff_pairs.items():
        if pair.p_pad:
            net_to_pair[pair.p_pad.net_name] = (pair_id, True)
        if pair.n_pad:
            net_to_pair[pair.n_pad.net_name] = (pair_id, False)

    # Calculate half-spacing for differential pairs
    # Each trace is offset from channel center by this amount
    half_pair_spacing = (track_width + diff_pair_gap) / 2

    # Build routes - process differential pairs together
    routes: List[FanoutRoute] = []
    processed_pairs: Set[str] = set()

    for pad in footprint.pads:
        if not pad.net_name or pad.net_id == 0:
            continue

        if net_filter:
            matched = any(fnmatch.fnmatch(pad.net_name, pattern) for pattern in net_filter)
            if not matched:
                continue

        # Check if this pad is part of a differential pair
        pair_id = None
        is_p = True
        if pad.net_name in net_to_pair:
            pair_id, is_p = net_to_pair[pad.net_name]

        # Skip if we already processed this pair
        if pair_id and pair_id in processed_pairs:
            continue

        if pair_id:
            # Process differential pair together
            processed_pairs.add(pair_id)
            pair = diff_pairs[pair_id]
            p_pad = pair.p_pad
            n_pad = pair.n_pad

            # Use pre-assigned escape direction if available, otherwise compute
            if pair_id in pair_escape_assignments:
                channel, escape_dir = pair_escape_assignments[pair_id]
            else:
                channel, escape_dir = find_diff_pair_escape(
                    p_pad.global_x, p_pad.global_y,
                    n_pad.global_x, n_pad.global_y,
                    grid, channels
                )
            is_edge = channel is None

            # Determine if pads are horizontally or vertically adjacent
            pads_horizontal = abs(p_pad.global_x - n_pad.global_x) > abs(p_pad.global_y - n_pad.global_y)

            # Check if escape direction is "cross" to pad orientation
            # Cross case: horizontal pads escaping vertically, or vertical pads escaping horizontally
            # In cross case, pads converge with 45° stubs (like edge pairs)
            is_cross_escape = False
            if channel:
                if pads_horizontal and escape_dir in ['up', 'down']:
                    is_cross_escape = True
                elif not pads_horizontal and escape_dir in ['left', 'right']:
                    is_cross_escape = True

            # Determine which pad is "positive" offset and which is "negative"
            # For horizontal channel (left/right escape): offset is in Y direction
            # For vertical channel (up/down escape): offset is in X direction
            #
            # Key insight: To avoid crossing, the pad that is FURTHER from the channel
            # should get the offset that brings it CLOSER to the channel center,
            # while the pad CLOSER to the channel gets offset AWAY from channel center.
            if channel and channel.orientation == 'horizontal' and not is_cross_escape:
                # Horizontal channel - pads are horizontally adjacent, escaping left/right
                # Traces will be offset in Y (one above, one below channel center)
                #
                # Channel is either above (channel.position < pad_y) or below the pads
                channel_above = channel.position < p_pad.global_y

                if p_pad.global_x < n_pad.global_x:
                    # P is left of N
                    if channel_above:
                        # Channel is above pads - P (left) goes to upper offset, N to lower
                        # This way P's 45° stub going up-left doesn't cross N's stub going up-right
                        p_offset = -half_pair_spacing  # upper (towards channel)
                        n_offset = half_pair_spacing   # lower (away from channel)
                    else:
                        # Channel is below pads - P (left) goes to lower offset, N to upper
                        # This way P's 45° stub going down-left doesn't cross N's stub going down-right
                        p_offset = half_pair_spacing   # lower (towards channel)
                        n_offset = -half_pair_spacing  # upper (away from channel)
                else:
                    # P is right of N (reversed)
                    if channel_above:
                        p_offset = half_pair_spacing
                        n_offset = -half_pair_spacing
                    else:
                        p_offset = -half_pair_spacing
                        n_offset = half_pair_spacing
            elif channel and channel.orientation == 'vertical' and not is_cross_escape:
                # Vertical channel - pads are vertically adjacent, escaping up/down
                # Traces will be offset in X (one left, one right of channel center)
                channel_left = channel.position < p_pad.global_x

                if p_pad.global_y < n_pad.global_y:
                    # P is above N
                    if channel_left:
                        p_offset = -half_pair_spacing
                        n_offset = half_pair_spacing
                    else:
                        p_offset = half_pair_spacing
                        n_offset = -half_pair_spacing
                else:
                    # P is below N
                    if channel_left:
                        p_offset = half_pair_spacing
                        n_offset = -half_pair_spacing
                    else:
                        p_offset = -half_pair_spacing
                        n_offset = half_pair_spacing
            else:
                # Edge pads or cross-escape - no offset needed, they converge with 45° stubs
                p_offset = 0
                n_offset = 0

            # Check for half-edge case
            is_half_edge = escape_dir.startswith('half_edge_')
            if is_half_edge:
                actual_escape_dir = escape_dir.replace('half_edge_', '')
            else:
                actual_escape_dir = escape_dir

            # Create routes for both P and N
            for pad_info, offset, is_p_route in [(p_pad, p_offset, True), (n_pad, n_offset, False)]:
                if is_half_edge:
                    # Half-edge pair: one pad on edge, one inner
                    # Edge pad: goes straight out to BGA edge
                    # Inner pad: 45° up to channel center, then 45° back down to converge
                    #            with edge pad at pair spacing
                    #
                    # The inner pad makes a "tent" shape to go around the via pad

                    is_edge_p_check, _ = is_edge_pad(p_pad.global_x, p_pad.global_y, grid)
                    is_edge_n_check, _ = is_edge_pad(n_pad.global_x, n_pad.global_y, grid)

                    # Identify which pad is edge and which is inner
                    if is_edge_p_check:
                        edge_pad_info = p_pad
                        inner_pad_info = n_pad
                        edge_is_p = True
                    else:
                        edge_pad_info = n_pad
                        inner_pad_info = p_pad
                        edge_is_p = False

                    this_pad_is_edge = (is_p_route and edge_is_p) or (not is_p_route and not edge_is_p)
                    pair_spacing_full = 2 * half_pair_spacing

                    if actual_escape_dir in ['left', 'right']:
                        # Find channel between inner pad and edge pad (horizontally adjacent)
                        h_channels = [c for c in channels if c.orientation == 'horizontal']
                        inner_y = inner_pad_info.global_y
                        edge_y = edge_pad_info.global_y

                        # Channel should be between the two pads OR closest to inner going away from edge
                        # Since they're on same row, find channel above or below
                        channels_above = [c for c in h_channels if c.position < inner_y]
                        channels_below = [c for c in h_channels if c.position > inner_y]

                        # Choose channel direction based on distance to BGA edge
                        dist_to_top = inner_y - grid.min_y
                        dist_to_bottom = grid.max_y - inner_y

                        if dist_to_top <= dist_to_bottom and channels_above:
                            inner_channel = max(channels_above, key=lambda c: c.position)
                            channel_above = True
                        elif channels_below:
                            inner_channel = min(channels_below, key=lambda c: c.position)
                            channel_above = False
                        else:
                            inner_channel = max(channels_above, key=lambda c: c.position)
                            channel_above = True

                        if this_pad_is_edge:
                            # Edge pad: straight out horizontally
                            # stub_end = pad position (no stub needed)
                            stub_end = (edge_pad_info.global_x, edge_pad_info.global_y)
                            if actual_escape_dir == 'right':
                                exit_pos = (grid.max_x + exit_margin, edge_pad_info.global_y)
                            else:
                                exit_pos = (grid.min_x - exit_margin, edge_pad_info.global_y)
                            route_channel = None
                            channel_pt = None
                            channel_pt2 = None
                        else:
                            # Inner pad: 45° up to channel, horizontal in channel (1 pitch),
                            # then 45° back down to converge with edge pad
                            channel_y = inner_channel.position
                            dy_to_channel = channel_y - inner_pad_info.global_y

                            if actual_escape_dir == 'right':
                                # First 45°: pad -> channel entry point
                                channel_pt_x = inner_pad_info.global_x + abs(dy_to_channel)
                                channel_pt = (channel_pt_x, channel_y)

                                # Horizontal segment in channel: 1 pitch toward edge
                                channel_pt2_x = channel_pt_x + grid.pitch_x
                                channel_pt2 = (channel_pt2_x, channel_y)

                                # Target Y at exit = edge pad Y + offset for pair spacing
                                if channel_above:
                                    target_exit_y = edge_pad_info.global_y - pair_spacing_full
                                else:
                                    target_exit_y = edge_pad_info.global_y + pair_spacing_full

                                # Second 45°: from channel_pt2 back toward target_exit_y
                                dy_return = target_exit_y - channel_y
                                stub_end_x = channel_pt2_x + abs(dy_return)
                                stub_end = (stub_end_x, target_exit_y)

                                exit_pos = (grid.max_x + exit_margin, target_exit_y)

                            else:  # left
                                channel_pt_x = inner_pad_info.global_x - abs(dy_to_channel)
                                channel_pt = (channel_pt_x, channel_y)

                                channel_pt2_x = channel_pt_x - grid.pitch_x
                                channel_pt2 = (channel_pt2_x, channel_y)

                                if channel_above:
                                    target_exit_y = edge_pad_info.global_y - pair_spacing_full
                                else:
                                    target_exit_y = edge_pad_info.global_y + pair_spacing_full

                                dy_return = target_exit_y - channel_y
                                stub_end_x = channel_pt2_x - abs(dy_return)
                                stub_end = (stub_end_x, target_exit_y)

                                exit_pos = (grid.min_x - exit_margin, target_exit_y)

                            route_channel = inner_channel

                    else:
                        # Vertical escape - similar logic but X/Y swapped
                        v_channels = [c for c in channels if c.orientation == 'vertical']
                        inner_x = inner_pad_info.global_x
                        edge_x = edge_pad_info.global_x

                        channels_left = [c for c in v_channels if c.position < inner_x]
                        channels_right = [c for c in v_channels if c.position > inner_x]

                        dist_to_left = inner_x - grid.min_x
                        dist_to_right = grid.max_x - inner_x

                        if dist_to_left <= dist_to_right and channels_left:
                            inner_channel = max(channels_left, key=lambda c: c.position)
                            channel_left = True
                        elif channels_right:
                            inner_channel = min(channels_right, key=lambda c: c.position)
                            channel_left = False
                        else:
                            inner_channel = max(channels_left, key=lambda c: c.position)
                            channel_left = True

                        if this_pad_is_edge:
                            stub_end = (edge_pad_info.global_x, edge_pad_info.global_y)
                            if actual_escape_dir == 'down':
                                exit_pos = (edge_pad_info.global_x, grid.max_y + exit_margin)
                            else:
                                exit_pos = (edge_pad_info.global_x, grid.min_y - exit_margin)
                            route_channel = None
                            channel_pt = None
                            channel_pt2 = None
                        else:
                            channel_x = inner_channel.position
                            dx_to_channel = channel_x - inner_pad_info.global_x

                            if actual_escape_dir == 'down':
                                # First 45°: pad -> channel entry point
                                channel_pt_y = inner_pad_info.global_y + abs(dx_to_channel)
                                channel_pt = (channel_x, channel_pt_y)

                                # Vertical segment in channel: 1 pitch toward edge
                                channel_pt2_y = channel_pt_y + grid.pitch_y
                                channel_pt2 = (channel_x, channel_pt2_y)

                                # Target X at exit = edge pad X + offset for pair spacing
                                if channel_left:
                                    target_exit_x = edge_pad_info.global_x - pair_spacing_full
                                else:
                                    target_exit_x = edge_pad_info.global_x + pair_spacing_full

                                # Second 45°: from channel_pt2 back toward target_exit_x
                                dx_return = target_exit_x - channel_x
                                stub_end_y = channel_pt2_y + abs(dx_return)
                                stub_end = (target_exit_x, stub_end_y)

                                exit_pos = (target_exit_x, grid.max_y + exit_margin)

                            else:  # up
                                # First 45°: pad -> channel entry point
                                channel_pt_y = inner_pad_info.global_y - abs(dx_to_channel)
                                channel_pt = (channel_x, channel_pt_y)

                                # Vertical segment in channel: 1 pitch toward edge
                                channel_pt2_y = channel_pt_y - grid.pitch_y
                                channel_pt2 = (channel_x, channel_pt2_y)

                                # Target X at exit = edge pad X + offset for pair spacing
                                if channel_left:
                                    target_exit_x = edge_pad_info.global_x - pair_spacing_full
                                else:
                                    target_exit_x = edge_pad_info.global_x + pair_spacing_full

                                # Second 45°: from channel_pt2 back toward target_exit_x
                                dx_return = target_exit_x - channel_x
                                stub_end_y = channel_pt2_y - abs(dx_return)
                                stub_end = (target_exit_x, stub_end_y)

                                exit_pos = (target_exit_x, grid.min_y - exit_margin)

                            route_channel = inner_channel

                    route = FanoutRoute(
                        pad=pad_info,
                        pad_pos=(pad_info.global_x, pad_info.global_y),
                        stub_end=stub_end,
                        exit_pos=exit_pos,
                        channel_point=channel_pt if not this_pad_is_edge else None,
                        channel_point2=channel_pt2 if not this_pad_is_edge else None,
                        channel=route_channel,
                        escape_dir=actual_escape_dir,
                        is_edge=this_pad_is_edge,
                        layer=layers[0],
                        pair_id=pair_id,
                        is_p=is_p_route
                    )
                    routes.append(route)
                    continue  # Skip the normal edge/inner handling below

                if is_edge or is_cross_escape:
                    # Edge pads or cross-escape: converge with 45° stubs to meet at pair spacing
                    # Cross-escape: horizontal pads escaping vertically, or vertical pads escaping horizontally
                    # Calculate the center point between P and N pads
                    center_x = (p_pad.global_x + n_pad.global_x) / 2
                    center_y = (p_pad.global_y + n_pad.global_y) / 2

                    if pads_horizontal:
                        # Pads are side by side horizontally (like T9 and T10 in screenshot)
                        # They need to converge to pair spacing using 45° stubs
                        # Final X positions: center_x +/- half_pair_spacing

                        # Target X for converged pair
                        if is_p_route:
                            target_x = center_x - half_pair_spacing
                        else:
                            target_x = center_x + half_pair_spacing

                        # Distance each trace needs to move in X (towards center)
                        dx_needed = target_x - pad_info.global_x

                        # At 45°, dy = dx (in absolute terms, direction depends on escape)
                        if escape_dir == 'down':
                            # Going down: Y increases, stub goes at 45° down
                            stub_end_y = pad_info.global_y + abs(dx_needed)
                            stub_end_x = target_x
                        elif escape_dir == 'up':
                            # Going up: Y decreases, stub goes at 45° up
                            stub_end_y = pad_info.global_y - abs(dx_needed)
                            stub_end_x = target_x
                        else:
                            # For left/right edge with horizontal pads, shouldn't happen normally
                            stub_end_x = target_x
                            stub_end_y = pad_info.global_y

                        stub_end = (stub_end_x, stub_end_y)

                        # Exit position continues in escape direction
                        if escape_dir == 'down':
                            exit_pos = (stub_end[0], grid.max_y + exit_margin)
                        elif escape_dir == 'up':
                            exit_pos = (stub_end[0], grid.min_y - exit_margin)
                        elif escape_dir == 'right':
                            exit_pos = (grid.max_x + exit_margin, stub_end[1])
                        else:  # left
                            exit_pos = (grid.min_x - exit_margin, stub_end[1])
                    else:
                        # Pads are vertically adjacent
                        # They need to converge to pair spacing using 45° stubs

                        # Target Y for converged pair
                        if is_p_route:
                            target_y = center_y - half_pair_spacing
                        else:
                            target_y = center_y + half_pair_spacing

                        # Distance each trace needs to move in Y (towards center)
                        dy_needed = target_y - pad_info.global_y

                        # At 45°, dx = dy (in absolute terms)
                        if escape_dir == 'right':
                            stub_end_x = pad_info.global_x + abs(dy_needed)
                            stub_end_y = target_y
                        elif escape_dir == 'left':
                            stub_end_x = pad_info.global_x - abs(dy_needed)
                            stub_end_y = target_y
                        else:
                            stub_end_x = pad_info.global_x
                            stub_end_y = target_y

                        stub_end = (stub_end_x, stub_end_y)

                        if escape_dir == 'right':
                            exit_pos = (grid.max_x + exit_margin, stub_end[1])
                        elif escape_dir == 'left':
                            exit_pos = (grid.min_x - exit_margin, stub_end[1])
                        elif escape_dir == 'down':
                            exit_pos = (stub_end[0], grid.max_y + exit_margin)
                        else:  # up
                            exit_pos = (stub_end[0], grid.min_y - exit_margin)
                else:
                    # Inner pads with aligned escape: 45° stub to channel with offset, then channel to exit
                    stub_end = create_45_stub(pad_info.global_x, pad_info.global_y,
                                             channel, escape_dir, offset)
                    exit_pos = calculate_exit_point(stub_end, channel, escape_dir,
                                                   grid, exit_margin, offset)

                route = FanoutRoute(
                    pad=pad_info,
                    pad_pos=(pad_info.global_x, pad_info.global_y),
                    stub_end=stub_end,
                    exit_pos=exit_pos,
                    channel=channel,
                    escape_dir=escape_dir,
                    is_edge=is_edge,
                    layer=layers[0],
                    pair_id=pair_id,
                    is_p=is_p_route
                )
                routes.append(route)
        else:
            # Single-ended signal (not part of a pair)
            channel, escape_dir = find_escape_channel(pad.global_x, pad.global_y, grid, channels)
            is_edge = channel is None

            if is_edge:
                if escape_dir == 'right':
                    exit_pos = (grid.max_x + exit_margin, pad.global_y)
                elif escape_dir == 'left':
                    exit_pos = (grid.min_x - exit_margin, pad.global_y)
                elif escape_dir == 'down':
                    exit_pos = (pad.global_x, grid.max_y + exit_margin)
                else:  # up
                    exit_pos = (pad.global_x, grid.min_y - exit_margin)
                stub_end = exit_pos
            else:
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
                layer=layers[0],
                pair_id=None,
                is_p=True
            )
            routes.append(route)

    print(f"  Found {len(routes)} pads to fanout")
    paired_count = sum(1 for r in routes if r.pair_id is not None)
    print(f"    {paired_count} are part of differential pairs")

    # Print escape direction distribution
    escape_counts = defaultdict(int)
    for r in routes:
        escape_counts[r.escape_dir] += 1
    print(f"  Escape direction distribution:")
    for direction in ['left', 'right', 'up', 'down']:
        if escape_counts[direction] > 0:
            print(f"    {direction}: {escape_counts[direction]}")

    # Print all net names being fanned out
    net_names = sorted(set(r.pad.net_name for r in routes if r.pad.net_name))
    print(f"  Nets being fanned out:")
    for name in net_names:
        print(f"    {name}")

    if not routes:
        return [], []

    # Smart layer assignment (keeps diff pairs together)
    assign_layers_smart(routes, layers, track_width, clearance, diff_pair_gap)

    # Calculate jog length = distance from BGA edge to first pad row/col
    # This is half the pitch (since edge is pitch/2 from first pad)
    jog_length = min(grid.pitch_x, grid.pitch_y) / 2
    print(f"  Jog length: {jog_length:.2f} mm")

    # Calculate jog_end for each route based on layer
    # For differential pairs, determine which track is on the outside of the bend
    pair_spacing = track_width + diff_pair_gap  # Center-to-center spacing

    for route in routes:
        is_outside = False

        if route.pair_id:
            # Determine jog direction for this layer
            try:
                layer_idx = layers.index(route.layer)
            except ValueError:
                layer_idx = 0
            num_layers = len(layers)
            if num_layers <= 1:
                jog_direction = -1  # left
            else:
                layer_factor = layer_idx / (num_layers - 1)
                jog_direction = 2 * layer_factor - 1  # -1 = left, +1 = right

            # Determine if this route is on the outside of the bend
            # "Outside" means the track that is further from the bend direction
            # For horizontal escape (left/right), the jog is in Y direction
            # For vertical escape (up/down), the jog is in X direction
            #
            # For right escape with negative jog (up/-Y): lower Y is outside
            # For right escape with positive jog (down/+Y): higher Y is outside
            # Similar logic for other directions

            if route.escape_dir in ['left', 'right']:
                # Horizontal escape, jog is in Y direction
                # Check if this route has higher or lower Y than its pair partner
                # Find the partner route
                for other in routes:
                    if other.pair_id == route.pair_id and other is not route:
                        if route.escape_dir == 'right':
                            if jog_direction < 0:  # Jog goes up (-Y)
                                is_outside = route.exit_pos[1] > other.exit_pos[1]
                            else:  # Jog goes down (+Y)
                                is_outside = route.exit_pos[1] < other.exit_pos[1]
                        else:  # left
                            if jog_direction < 0:  # Jog goes down (+Y)
                                is_outside = route.exit_pos[1] < other.exit_pos[1]
                            else:  # Jog goes up (-Y)
                                is_outside = route.exit_pos[1] > other.exit_pos[1]
                        break
            else:
                # Vertical escape, jog is in X direction
                for other in routes:
                    if other.pair_id == route.pair_id and other is not route:
                        if route.escape_dir == 'down':
                            if jog_direction < 0:  # Jog goes right (+X)
                                is_outside = route.exit_pos[0] < other.exit_pos[0]
                            else:  # Jog goes left (-X)
                                is_outside = route.exit_pos[0] > other.exit_pos[0]
                        else:  # up
                            if jog_direction < 0:  # Jog goes left (-X)
                                is_outside = route.exit_pos[0] > other.exit_pos[0]
                            else:  # Jog goes right (+X)
                                is_outside = route.exit_pos[0] < other.exit_pos[0]
                        break

        jog_end, extension = calculate_jog_end(
            route.exit_pos,
            route.escape_dir,
            route.layer,
            layers,
            jog_length,
            is_diff_pair=route.pair_id is not None,
            is_outside_track=is_outside,
            pair_spacing=pair_spacing
        )
        route.jog_end = jog_end
        route.jog_extension = extension

    # Generate tracks
    tracks = []
    edge_count = 0
    inner_count = 0

    for route in routes:
        if route.is_edge:
            # Edge pad: Check if stub_end differs from pad_pos (differential pair convergence)
            if route.pair_id and (abs(route.stub_end[0] - route.pad_pos[0]) > 0.001 or
                                   abs(route.stub_end[1] - route.pad_pos[1]) > 0.001):
                # Differential edge pair: 45° stub to converge, then straight to exit
                # 45° stub: pad -> stub_end (convergence point)
                tracks.append({
                    'start': route.pad_pos,
                    'end': route.stub_end,
                    'width': track_width,
                    'layer': route.layer,
                    'net_id': route.net_id,
                    'pair_id': route.pair_id
                })
                # Straight segment: stub_end -> exit
                tracks.append({
                    'start': route.stub_end,
                    'end': route.exit_pos,
                    'width': track_width,
                    'layer': route.layer,
                    'net_id': route.net_id,
                    'pair_id': route.pair_id
                })
            else:
                # Single-ended edge pad: direct segment to exit
                tracks.append({
                    'start': route.pad_pos,
                    'end': route.exit_pos,
                    'width': track_width,
                    'layer': route.layer,
                    'net_id': route.net_id,
                    'pair_id': route.pair_id
                })
            # Add jog segment(s)
            if route.jog_end:
                if route.jog_extension:
                    # Outside track of diff pair: extension segment first
                    tracks.append({
                        'start': route.exit_pos,
                        'end': route.jog_extension,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })
                    # Then the 45° jog from extension point
                    tracks.append({
                        'start': route.jog_extension,
                        'end': route.jog_end,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })
                else:
                    # Inside track or single-ended: direct 45° jog
                    tracks.append({
                        'start': route.exit_pos,
                        'end': route.jog_end,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })
            edge_count += 1
        else:
            # Inner pad: 45° stub + channel segment + jog
            # For half-edge pairs: pad -> channel_point -> stub_end -> exit

            if route.channel_point:
                # Half-edge inner pad: tent shape with channel segment
                # Path: pad -> channel_point -> channel_point2 -> stub_end -> exit
                #
                # First 45°: pad -> channel_point (entry to channel)
                tracks.append({
                    'start': route.pad_pos,
                    'end': route.channel_point,
                    'width': track_width,
                    'layer': route.layer,
                    'net_id': route.net_id,
                    'pair_id': route.pair_id
                })
                # Straight segment in channel: channel_point -> channel_point2 (1 pitch)
                if route.channel_point2:
                    tracks.append({
                        'start': route.channel_point,
                        'end': route.channel_point2,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })
                    # Second 45°: channel_point2 -> stub_end (exit from channel)
                    tracks.append({
                        'start': route.channel_point2,
                        'end': route.stub_end,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })
                else:
                    # Fallback if no channel_point2 (shouldn't happen)
                    tracks.append({
                        'start': route.channel_point,
                        'end': route.stub_end,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })
                # Straight segment: stub_end -> exit
                tracks.append({
                    'start': route.stub_end,
                    'end': route.exit_pos,
                    'width': track_width,
                    'layer': route.layer,
                    'net_id': route.net_id,
                    'pair_id': route.pair_id
                })
            else:
                # Normal inner pad: single 45° stub to channel, then straight to exit
                # Skip zero-length stubs
                dx = abs(route.stub_end[0] - route.pad_pos[0])
                dy = abs(route.stub_end[1] - route.pad_pos[1])
                if dx < 0.001 and dy < 0.001:
                    inner_count += 1
                    continue

                # 45-degree stub: pad -> channel
                tracks.append({
                    'start': route.pad_pos,
                    'end': route.stub_end,
                    'width': track_width,
                    'layer': route.layer,
                    'net_id': route.net_id,
                    'pair_id': route.pair_id
                })

                # Channel segment: stub_end -> exit
                tracks.append({
                    'start': route.stub_end,
                    'end': route.exit_pos,
                    'width': track_width,
                    'layer': route.layer,
                    'net_id': route.net_id,
                    'pair_id': route.pair_id
                })

            # Jog segment(s): exit -> jog_end
            if route.jog_end:
                if route.jog_extension:
                    # Outside track of diff pair: extension segment first
                    tracks.append({
                        'start': route.exit_pos,
                        'end': route.jog_extension,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })
                    # Then the 45° jog from extension point
                    tracks.append({
                        'start': route.jog_extension,
                        'end': route.jog_end,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })
                else:
                    # Inside track or single-ended: direct 45° jog
                    tracks.append({
                        'start': route.exit_pos,
                        'end': route.jog_end,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
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
            # Skip collision check for tracks from the same differential pair
            if t1.get('pair_id') and t1.get('pair_id') == t2.get('pair_id'):
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
    parser.add_argument('--diff-pairs', '-d', nargs='*',
                        help='Differential pair net patterns (e.g., "*lvds*"). '
                             'Matching P/N pairs will be routed together on same layer.')
    parser.add_argument('--diff-pair-gap', type=float, default=0.1,
                        help='Gap between differential pair traces in mm')
    parser.add_argument('--exit-margin', type=float, default=0.5,
                        help='Distance past BGA boundary')
    parser.add_argument('--primary-escape', '-p', choices=['horizontal', 'vertical'],
                        default='horizontal',
                        help='Primary escape direction preference (default: horizontal). '
                             'Pairs will use this direction first, then switch if channels are full.')

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
        diff_pair_patterns=args.diff_pairs,
        layers=args.layers,
        track_width=args.width,
        clearance=args.clearance,
        diff_pair_gap=args.diff_pair_gap,
        exit_margin=args.exit_margin,
        primary_escape=args.primary_escape
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
