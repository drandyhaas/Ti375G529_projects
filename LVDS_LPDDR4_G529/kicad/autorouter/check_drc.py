"""
DRC Checker - Find overlapping tracks and vias between different nets.
"""

import sys
import math
from typing import List, Tuple, Set
from kicad_parser import parse_kicad_pcb, Segment, Via


def point_to_segment_distance(px: float, py: float,
                               x1: float, y1: float,
                               x2: float, y2: float) -> float:
    """Calculate minimum distance from point (px, py) to segment (x1,y1)-(x2,y2)."""
    dx = x2 - x1
    dy = y2 - y1

    if dx == 0 and dy == 0:
        return math.sqrt((px - x1)**2 + (py - y1)**2)

    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))

    proj_x = x1 + t * dx
    proj_y = y1 + t * dy

    return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)


def segment_to_segment_distance(seg1: Segment, seg2: Segment) -> float:
    """Calculate minimum distance between two segments."""
    # Check all endpoint-to-segment distances
    d1 = point_to_segment_distance(seg1.start_x, seg1.start_y,
                                    seg2.start_x, seg2.start_y,
                                    seg2.end_x, seg2.end_y)
    d2 = point_to_segment_distance(seg1.end_x, seg1.end_y,
                                    seg2.start_x, seg2.start_y,
                                    seg2.end_x, seg2.end_y)
    d3 = point_to_segment_distance(seg2.start_x, seg2.start_y,
                                    seg1.start_x, seg1.start_y,
                                    seg1.end_x, seg1.end_y)
    d4 = point_to_segment_distance(seg2.end_x, seg2.end_y,
                                    seg1.start_x, seg1.start_y,
                                    seg1.end_x, seg1.end_y)

    return min(d1, d2, d3, d4)


def check_segment_overlap(seg1: Segment, seg2: Segment, clearance: float, tolerance: float = 0.001) -> Tuple[bool, float]:
    """Check if two segments on the same layer violate clearance.

    Args:
        tolerance: Ignore violations smaller than this (mm). Default 0.001mm (1 micron).
    """
    if seg1.layer != seg2.layer:
        return False, 0.0

    # Required distance is half-widths plus clearance
    required_dist = seg1.width / 2 + seg2.width / 2 + clearance
    actual_dist = segment_to_segment_distance(seg1, seg2)
    overlap = required_dist - actual_dist

    if overlap > tolerance:
        return True, overlap
    return False, 0.0


def check_via_segment_overlap(via: Via, seg: Segment, clearance: float, tolerance: float = 0.001) -> Tuple[bool, float]:
    """Check if a via overlaps with a segment on any common layer.

    Args:
        tolerance: Ignore violations smaller than this (mm). Default 0.001mm (1 micron).
    """
    # Check if they share a layer
    via_layers = set(via.layers) if via.layers else {'F.Cu', 'B.Cu'}
    if seg.layer not in via_layers:
        return False, 0.0

    required_dist = via.size / 2 + seg.width / 2 + clearance
    actual_dist = point_to_segment_distance(via.x, via.y,
                                            seg.start_x, seg.start_y,
                                            seg.end_x, seg.end_y)
    overlap = required_dist - actual_dist

    if overlap > tolerance:
        return True, overlap
    return False, 0.0


def check_via_via_overlap(via1: Via, via2: Via, clearance: float, tolerance: float = 0.001) -> Tuple[bool, float]:
    """Check if two vias overlap.

    Args:
        tolerance: Ignore violations smaller than this (mm). Default 0.001mm (1 micron).
    """
    # All vias are through-hole, so they always potentially conflict
    required_dist = via1.size / 2 + via2.size / 2 + clearance
    actual_dist = math.sqrt((via1.x - via2.x)**2 + (via1.y - via2.y)**2)
    overlap = required_dist - actual_dist

    if overlap > tolerance:
        return True, overlap
    return False, 0.0


def run_drc(pcb_file: str, clearance: float = 0.1):
    """Run DRC checks on the PCB file."""
    print(f"Loading {pcb_file}...")
    pcb_data = parse_kicad_pcb(pcb_file)

    print(f"Found {len(pcb_data.segments)} segments and {len(pcb_data.vias)} vias")

    # Group segments and vias by net
    segments_by_net = {}
    for seg in pcb_data.segments:
        if seg.net_id not in segments_by_net:
            segments_by_net[seg.net_id] = []
        segments_by_net[seg.net_id].append(seg)

    vias_by_net = {}
    for via in pcb_data.vias:
        if via.net_id not in vias_by_net:
            vias_by_net[via.net_id] = []
        vias_by_net[via.net_id].append(via)

    violations = []

    # Check segment-to-segment violations (different nets only)
    print("\nChecking segment-to-segment clearances...")
    net_ids = list(segments_by_net.keys())
    for i, net1 in enumerate(net_ids):
        for net2 in net_ids[i+1:]:
            if net1 == net2:
                continue
            for seg1 in segments_by_net[net1]:
                for seg2 in segments_by_net[net2]:
                    has_violation, overlap = check_segment_overlap(seg1, seg2, clearance)
                    if has_violation:
                        net1_name = pcb_data.nets.get(net1, None)
                        net2_name = pcb_data.nets.get(net2, None)
                        net1_str = net1_name.name if net1_name else f"net_{net1}"
                        net2_str = net2_name.name if net2_name else f"net_{net2}"
                        violations.append({
                            'type': 'segment-segment',
                            'net1': net1_str,
                            'net2': net2_str,
                            'layer': seg1.layer,
                            'overlap_mm': overlap,
                            'loc1': (seg1.start_x, seg1.start_y, seg1.end_x, seg1.end_y),
                            'loc2': (seg2.start_x, seg2.start_y, seg2.end_x, seg2.end_y),
                        })

    # Check via-to-segment violations (different nets only)
    print("Checking via-to-segment clearances...")
    via_net_ids = list(vias_by_net.keys())
    for via_net in via_net_ids:
        for seg_net in net_ids:
            if via_net == seg_net:
                continue
            for via in vias_by_net[via_net]:
                for seg in segments_by_net.get(seg_net, []):
                    has_violation, overlap = check_via_segment_overlap(via, seg, clearance)
                    if has_violation:
                        via_net_name = pcb_data.nets.get(via_net, None)
                        seg_net_name = pcb_data.nets.get(seg_net, None)
                        via_net_str = via_net_name.name if via_net_name else f"net_{via_net}"
                        seg_net_str = seg_net_name.name if seg_net_name else f"net_{seg_net}"
                        violations.append({
                            'type': 'via-segment',
                            'net1': via_net_str,
                            'net2': seg_net_str,
                            'layer': seg.layer,
                            'overlap_mm': overlap,
                            'via_loc': (via.x, via.y),
                            'seg_loc': (seg.start_x, seg.start_y, seg.end_x, seg.end_y),
                        })

    # Check via-to-via violations (different nets only)
    print("Checking via-to-via clearances...")
    for i, net1 in enumerate(via_net_ids):
        for net2 in via_net_ids[i+1:]:
            if net1 == net2:
                continue
            for via1 in vias_by_net[net1]:
                for via2 in vias_by_net[net2]:
                    has_violation, overlap = check_via_via_overlap(via1, via2, clearance)
                    if has_violation:
                        net1_name = pcb_data.nets.get(net1, None)
                        net2_name = pcb_data.nets.get(net2, None)
                        net1_str = net1_name.name if net1_name else f"net_{net1}"
                        net2_str = net2_name.name if net2_name else f"net_{net2}"
                        violations.append({
                            'type': 'via-via',
                            'net1': net1_str,
                            'net2': net2_str,
                            'overlap_mm': overlap,
                            'loc1': (via1.x, via1.y),
                            'loc2': (via2.x, via2.y),
                        })

    # Report violations
    print("\n" + "=" * 60)
    if violations:
        print(f"FOUND {len(violations)} DRC VIOLATIONS:\n")

        # Group by type
        by_type = {}
        for v in violations:
            t = v['type']
            if t not in by_type:
                by_type[t] = []
            by_type[t].append(v)

        for vtype, vlist in by_type.items():
            print(f"\n{vtype.upper()} violations ({len(vlist)}):")
            print("-" * 40)
            for v in vlist[:20]:  # Show first 20 of each type
                if vtype == 'segment-segment':
                    print(f"  {v['net1']} <-> {v['net2']}")
                    print(f"    Layer: {v['layer']}, Overlap: {v['overlap_mm']:.3f}mm")
                    print(f"    Seg1: ({v['loc1'][0]:.2f},{v['loc1'][1]:.2f})-({v['loc1'][2]:.2f},{v['loc1'][3]:.2f})")
                    print(f"    Seg2: ({v['loc2'][0]:.2f},{v['loc2'][1]:.2f})-({v['loc2'][2]:.2f},{v['loc2'][3]:.2f})")
                elif vtype == 'via-segment':
                    print(f"  Via:{v['net1']} <-> Seg:{v['net2']}")
                    print(f"    Layer: {v['layer']}, Overlap: {v['overlap_mm']:.3f}mm")
                    print(f"    Via: ({v['via_loc'][0]:.2f},{v['via_loc'][1]:.2f})")
                    print(f"    Seg: ({v['seg_loc'][0]:.2f},{v['seg_loc'][1]:.2f})-({v['seg_loc'][2]:.2f},{v['seg_loc'][3]:.2f})")
                elif vtype == 'via-via':
                    print(f"  {v['net1']} <-> {v['net2']}")
                    print(f"    Overlap: {v['overlap_mm']:.3f}mm")
                    print(f"    Via1: ({v['loc1'][0]:.2f},{v['loc1'][1]:.2f})")
                    print(f"    Via2: ({v['loc2'][0]:.2f},{v['loc2'][1]:.2f})")

            if len(vlist) > 20:
                print(f"  ... and {len(vlist) - 20} more")
    else:
        print("NO DRC VIOLATIONS FOUND!")

    print("=" * 60)
    return violations


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python check_drc.py <pcb_file> [clearance_mm]")
        sys.exit(1)

    pcb_file = sys.argv[1]
    clearance = float(sys.argv[2]) if len(sys.argv) > 2 else 0.1

    run_drc(pcb_file, clearance)
