"""
Route DATA_0 through DATA_31 nets with optimized ordering.

Routing order matters for congested channels - routing from both ends toward
the middle gives better results than sequential ordering.
"""

import subprocess
import sys


def generate_alternating_order(n):
    """Generate alternating order from both ends toward middle."""
    order = []
    for i in range((n + 1) // 2):
        order.append(i)  # From start
        if n - 1 - i != i:  # Avoid duplicating middle element
            order.append(n - 1 - i)  # From end
    return order


# Generate alternating order for 32 nets
NUM_NETS = 32
indices = generate_alternating_order(NUM_NETS)
NET_ORDER = [f"Net-(U2A-DATA_{i})" for i in indices]


def main():
    input_file = "fanout_starting_point.kicad_pcb"
    output_file = "routed_output.kicad_pcb"

    if len(sys.argv) > 1:
        input_file = sys.argv[1]
    if len(sys.argv) > 2:
        output_file = sys.argv[2]

    cmd = ["python", "batch_router.py", input_file, output_file] + NET_ORDER
    print(f"Running: {' '.join(cmd[:4])} [{len(NET_ORDER)} nets in optimized order]")
    print(f"Input:  {input_file}")
    print(f"Output: {output_file}")
    print()

    subprocess.run(cmd)


if __name__ == "__main__":
    main()
