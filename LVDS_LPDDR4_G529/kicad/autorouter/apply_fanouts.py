"""
Apply BGA and QFN fanouts to create a starting point PCB for routing.

This script applies:
1. BGA fanout for U1 (FPGA) - 45° stubs to channels, edge pads direct escape
2. QFN fanout for U2 (FT601) - straight + 45° stubs for DATA signals

The output file can then be used for connecting the fanout endpoints.
"""

import argparse
from kicad_parser import parse_kicad_pcb
from kicad_writer import add_tracks_and_vias_to_pcb
from bga_fanout import generate_bga_fanout
from qfn_fanout import generate_qfn_fanout


def main():
    parser = argparse.ArgumentParser(description='Apply BGA and QFN fanouts')
    parser.add_argument('pcb', nargs='?',
                        default='../haasoscope_pro_max/haasoscope_pro_max.kicad_pcb',
                        help='Input PCB file')
    parser.add_argument('--output', '-o', default='fanout_starting_point.kicad_pcb',
                        help='Output PCB file')
    parser.add_argument('--bga-component', default='U3',
                        help='BGA component reference')
    parser.add_argument('--qfn-component', default='U2',
                        help='QFN component reference')
    parser.add_argument('--nets', '-n', nargs='*', default=['*DATA_*'],
                        help='Net patterns to include')
    parser.add_argument('--width', '-w', type=float, default=0.1,
                        help='Track width in mm')
    parser.add_argument('--clearance', type=float, default=0.1,
                        help='Track clearance in mm')

    args = parser.parse_args()

    print(f"Parsing {args.pcb}...")
    pcb_data = parse_kicad_pcb(args.pcb)

    all_tracks = []
    all_vias = []

    # Apply BGA fanout
    if args.bga_component in pcb_data.footprints:
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
    else:
        print(f"Warning: BGA component {args.bga_component} not found")

    # Apply QFN fanout
    if args.qfn_component in pcb_data.footprints:
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
    else:
        print(f"Warning: QFN component {args.qfn_component} not found")

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
