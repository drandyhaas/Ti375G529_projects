"""
Route DATA_0 through DATA_11 nets with optimized ordering.

Routing order matters for congested channels - routing from both ends toward
the middle gives better results than sequential ordering.
"""

import subprocess
import sys

# Alternating order: route from both ends toward middle
# This prevents early routes from blocking paths for later ones
NET_ORDER = [
    "Net-(U2A-DATA_0)",
    "Net-(U2A-DATA_11)",
    "Net-(U2A-DATA_1)",
    "Net-(U2A-DATA_10)",
    "Net-(U2A-DATA_2)",
    "Net-(U2A-DATA_9)",
    "Net-(U2A-DATA_3)",
    "Net-(U2A-DATA_8)",
    "Net-(U2A-DATA_4)",
    "Net-(U2A-DATA_7)",
    "Net-(U2A-DATA_5)",
    "Net-(U2A-DATA_6)",
]


def main():
    input_file = "fanout_starting_point.kicad_pcb"
    output_file = "routed_output.kicad_pcb"

    if len(sys.argv) > 1:
        input_file = sys.argv[1]
    if len(sys.argv) > 2:
        output_file = sys.argv[2]

    cmd = ["python", "batch_router.py", input_file, output_file] + NET_ORDER
    print(f"Running: {' '.join(cmd[:4])} [12 nets in optimized order]")
    print(f"Input:  {input_file}")
    print(f"Output: {output_file}")
    print()

    subprocess.run(cmd)


if __name__ == "__main__":
    main()
