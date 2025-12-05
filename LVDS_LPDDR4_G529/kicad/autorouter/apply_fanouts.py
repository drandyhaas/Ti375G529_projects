"""
Apply BGA and QFN fanouts to create a starting point PCB for routing.

This script applies:
1. BGA fanout for BGA packages - 45° stubs to channels, edge pads direct escape
2. QFN fanout for QFN/QFP packages - straight + 45° stubs for DATA signals

Components are auto-detected by package type if not explicitly specified.
The output file can then be used for connecting the fanout endpoints.
"""

import argparse
from kicad_parser import parse_kicad_pcb, find_components_by_type, detect_package_type
from kicad_writer import add_tracks_and_vias_to_pcb
from bga_fanout import generate_bga_fanout
from qfn_fanout import generate_qfn_fanout


def main():
    parser = argparse.ArgumentParser(description='Apply BGA and QFN fanouts')
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--output', '-o', default='fanout_starting_point.kicad_pcb',
                        help='Output PCB file')
    parser.add_argument('--bga-component', default=None,
                        help='BGA component reference (auto-detected if not specified)')
    parser.add_argument('--qfn-component', default=None,
                        help='QFN component reference (auto-detected if not specified)')
    parser.add_argument('--nets', '-n', nargs='*', default=['*DATA_*'],
                        help='Net patterns to include')
    parser.add_argument('--width', '-w', type=float, default=0.1,
                        help='Track width in mm')
    parser.add_argument('--clearance', type=float, default=0.1,
                        help='Track clearance in mm')

    args = parser.parse_args()

    print(f"Parsing {args.pcb}...")
    pcb_data = parse_kicad_pcb(args.pcb)

    # Auto-detect BGA component if not specified
    if args.bga_component is None:
        bga_components = find_components_by_type(pcb_data, 'BGA')
        if bga_components:
            args.bga_component = bga_components[0].reference
            print(f"Auto-detected BGA component: {args.bga_component}")
            if len(bga_components) > 1:
                print(f"  (Other BGAs found: {[fp.reference for fp in bga_components[1:]]})")
        else:
            print("No BGA components auto-detected")

    # Auto-detect QFN component if not specified
    if args.qfn_component is None:
        qfn_components = find_components_by_type(pcb_data, 'QFN')
        if not qfn_components:
            # Also check for QFP packages
            qfn_components = find_components_by_type(pcb_data, 'QFP')
        if qfn_components:
            args.qfn_component = qfn_components[0].reference
            print(f"Auto-detected QFN/QFP component: {args.qfn_component}")
            if len(qfn_components) > 1:
                print(f"  (Other QFN/QFPs found: {[fp.reference for fp in qfn_components[1:]]})")

    all_tracks = []
    all_vias = []

    # Apply BGA fanout
    if args.bga_component and args.bga_component in pcb_data.footprints:
        print(f"\n=== BGA Fanout for {args.bga_component} ===")
        bga_footprint = pcb_data.footprints[args.bga_component]
        bga_tracks, bga_vias = generate_bga_fanout(
            bga_footprint,
            pcb_data,
            net_filter=args.nets,
            track_width=args.width,
            clearance=args.clearance
        )
        all_tracks.extend(bga_tracks)
        all_vias.extend(bga_vias)
        print(f"  Added {len(bga_tracks)} tracks, {len(bga_vias)} vias")
    elif args.bga_component:
        print(f"Warning: BGA component {args.bga_component} not found in PCB")
    else:
        print("No BGA component specified or detected - skipping BGA fanout")

    # Apply QFN fanout
    if args.qfn_component and args.qfn_component in pcb_data.footprints:
        print(f"\n=== QFN Fanout for {args.qfn_component} ===")
        qfn_footprint = pcb_data.footprints[args.qfn_component]
        qfn_tracks, qfn_vias = generate_qfn_fanout(
            qfn_footprint,
            pcb_data,
            net_filter=args.nets,
            track_width=args.width,
            clearance=args.clearance
        )
        all_tracks.extend(qfn_tracks)
        all_vias.extend(qfn_vias)
        print(f"  Added {len(qfn_tracks)} tracks, {len(qfn_vias)} vias")
    elif args.qfn_component:
        print(f"Warning: QFN component {args.qfn_component} not found in PCB")
    else:
        print("No QFN component specified or detected - skipping QFN fanout")

    # Write output
    if all_tracks or all_vias:
        print(f"\n=== Writing Output ===")
        print(f"Total: {len(all_tracks)} tracks, {len(all_vias)} vias")
        add_tracks_and_vias_to_pcb(args.pcb, args.output, all_tracks, all_vias)
        print(f"Saved to: {args.output}")
    else:
        print("\nNo fanout elements generated")

    return 0


if __name__ == '__main__':
    exit(main())
