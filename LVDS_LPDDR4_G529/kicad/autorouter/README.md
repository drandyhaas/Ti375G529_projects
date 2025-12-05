# KiCad PCB Grid Router

A fast Rust-accelerated A* autorouter for KiCad PCB files using integer grid coordinates. Routes nets between existing stub endpoints with multi-layer support and automatic via placement.

## Features

- **Grid-based A* pathfinding** - Integer coordinates for fast collision detection
- **Octilinear routing** - Horizontal, vertical, and 45-degree diagonal moves
- **Multi-layer routing** - 4 layers (F.Cu, In1.Cu, In2.Cu, B.Cu) with automatic via insertion
- **Obstacle avoidance** - Respects existing tracks, vias, and pads with configurable clearance
- **Rectangular pad blocking** - Proper clearance for non-square pads with rotation handling
- **BGA exclusion zone** - Prevents vias under BGA packages
- **Batch routing** - Routes multiple nets sequentially, each avoiding previously routed tracks
- **Stub proximity avoidance** - Penalizes routes near unrouted stubs to prevent blocking

## Requirements

The router requires the Rust module for A* pathfinding. Build it first:

```bash
cd rust_router
cargo build --release

# Windows:
cp target/release/grid_router.dll grid_router.pyd

# Linux:
cp target/release/libgrid_router.so grid_router.so

# macOS:
cp target/release/libgrid_router.dylib grid_router.so
```

See [rust_router/README.md](rust_router/README.md) for detailed build instructions.

## Files

| File | Description |
|------|-------------|
| `batch_grid_router.py` | Main router - routes multiple nets using Rust A* |
| `kicad_parser.py` | Parses .kicad_pcb files into Python data structures (handles pad rotation) |
| `kicad_writer.py` | Generates KiCad S-expressions for segments and vias |
| `check_drc.py` | DRC checker for detecting clearance violations |
| `visualize_routing.py` | Matplotlib visualization of routing progress |
| `bga_fanout.py` | BGA fanout stub generation |
| `qfn_fanout.py` | QFN fanout stub generation |
| `apply_fanouts.py` | Apply generated fanouts to PCB file |
| `rust_router/` | Rust A* implementation with Python bindings |

## Quick Start

### Route Multiple Nets

```bash
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_0)" "Net-(U2A-DATA_1)"
```

### Route 32 DATA Nets with Inside-Out Ordering

For BGA fanout routing, inside-out ordering (routing inner pins first) achieves the best success rate:

```bash
python batch_grid_router.py fanout_starting_point.kicad_pcb routed_output.kicad_pcb \
  "Net-(U2A-DATA_23)" "Net-(U2A-DATA_20)" "Net-(U2A-DATA_17)" "Net-(U2A-DATA_22)" \
  "Net-(U2A-DATA_24)" "Net-(U2A-DATA_16)" "Net-(U2A-DATA_18)" "Net-(U2A-DATA_14)" \
  "Net-(U2A-DATA_26)" "Net-(U2A-DATA_11)" "Net-(U2A-DATA_21)" "Net-(U2A-DATA_29)" \
  "Net-(U2A-DATA_25)" "Net-(U2A-DATA_30)" "Net-(U2A-DATA_15)" "Net-(U2A-DATA_13)" \
  "Net-(U2A-DATA_9)" "Net-(U2A-DATA_19)" "Net-(U2A-DATA_27)" "Net-(U2A-DATA_8)" \
  "Net-(U2A-DATA_10)" "Net-(U2A-DATA_28)" "Net-(U2A-DATA_31)" "Net-(U2A-DATA_7)" \
  "Net-(U2A-DATA_5)" "Net-(U2A-DATA_12)" "Net-(U2A-DATA_0)" "Net-(U2A-DATA_4)" \
  "Net-(U2A-DATA_2)" "Net-(U2A-DATA_3)" "Net-(U2A-DATA_6)" "Net-(U2A-DATA_1)"
```

## Determining Net Ordering

For BGA breakout routing, order nets by distance from the BGA center (inside-out):

```python
from kicad_parser import parse_kicad_pcb
import math

pcb_data = parse_kicad_pcb('input.kicad_pcb')

# BGA center coordinates
bga_center_x, bga_center_y = 195.4, 103.0

net_distances = []
for i in range(32):
    net_name = f'Net-(U2A-DATA_{i})'
    net_id = next((nid for nid, net in pcb_data.nets.items() if net.name == net_name), None)
    if net_id is None:
        continue

    # Get stub segment positions
    segs = [s for s in pcb_data.segments if s.net_id == net_id]
    if not segs:
        continue

    # Calculate centroid
    points = [(s.start_x, s.start_y) for s in segs] + [(s.end_x, s.end_y) for s in segs]
    avg_x = sum(p[0] for p in points) / len(points)
    avg_y = sum(p[1] for p in points) / len(points)

    dist = math.sqrt((avg_x - bga_center_x)**2 + (avg_y - bga_center_y)**2)
    net_distances.append((net_name, dist))

# Sort by distance (inside-out)
net_distances.sort(key=lambda x: x[1])
print([name for name, _ in net_distances])
```

## Configuration

Default parameters in `batch_grid_router.py`:

```python
GridRouteConfig(
    track_width=0.1,        # mm - width of new tracks
    clearance=0.1,          # mm - minimum edge-to-edge spacing
    via_size=0.3,           # mm - via outer diameter
    via_drill=0.2,          # mm - via drill size
    grid_step=0.1,          # mm - routing grid resolution
    via_cost=500,           # integer cost penalty (1000 = 1 grid step)
    layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
    max_iterations=100000,  # max A* iterations per route
    heuristic_weight=1.5,   # A* greediness (1.0=optimal, >1.0=faster)
    bga_exclusion_zone=(185.9, 93.5, 204.9, 112.5),  # no vias in this rectangle
    stub_proximity_radius=1.0,  # mm - penalize routes near unrouted stubs
    stub_proximity_cost=3.0,    # mm equivalent cost at stub center
)
```

### Parameter Notes

- **clearance**: Edge-to-edge spacing between tracks. A 0.1mm clearance means tracks with 0.1mm width can pass with 0.2mm center-to-center spacing.
- **heuristic_weight**: Values > 1.0 make the search greedier (faster but potentially suboptimal). 1.5 is a good balance.
- **max_iterations**: Increase for complex routes. Easy routes need ~200 iterations, hard routes may need 10,000+.
- **via_cost**: Higher values discourage layer changes. 500 means a via costs as much as 0.5 grid steps of travel.
- **stub_proximity_radius/cost**: Routes passing near unrouted stub endpoints incur extra cost. This prevents early routes from blocking later ones. Vias near stubs are penalized 2x.

## Performance

Tested on 32 DATA nets with BGA fanout:

| Metric | Value |
|--------|-------|
| Success Rate | **32/32 (100%)** |
| Total Time | ~7 seconds |
| Total Iterations | ~285,000 |
| DRC Violations | 0 (DATA nets) |

**Key features for high success rates:**
- **Stub proximity avoidance**: Routes are penalized for passing near unrouted stub endpoints, preventing early routes from blocking later ones
- **Smart direction search**: Try forward direction quickly (5000 iterations), then reverse with full iterations, then forward with full iterations. This finds the "easy" direction faster.
- **Rectangular pad blocking**: Proper clearance for non-square pads (e.g., QFN 0.2x0.875mm pads)

### Iterative Retry Strategy

When some nets fail, retry routing just the failed nets on the already-routed output file. This often succeeds because the routing order changes:

```bash
# Initial routing
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-A" "Net-B" "Net-C"

# Retry failed nets
python batch_grid_router.py output.kicad_pcb output.kicad_pcb "Net-B"
```

## How It Works

### 1. Grid Discretization

Coordinates are converted to integers: `grid_x = round(mm_x / grid_step)`

This enables fast set-based collision detection with O(1) lookups.

### 2. Obstacle Map

Before routing, all obstacles are rasterized to grid cells:
- Existing tracks (expanded by half-width + clearance)
- Existing vias (all layers blocked)
- Component pads with **rectangular bounds** (respects pad rotation for correct orientation)
- Via placement zones (tracks and pads block via placement with appropriate clearances)
- **Stub proximity cost field** (unrouted stub locations have decaying cost penalty)

The net being routed is excluded from obstacles.

**Pad Rotation Handling**: The parser correctly transforms pad dimensions based on rotation. For example, QFN pads defined as (0.875 x 0.2)mm with 90° rotation become (0.2 x 0.875)mm in board coordinates, ensuring rectangular via blocking zones are correctly oriented.

**Via Blocking**: Uses `int()` instead of `round()` for grid discretization to avoid over-blocking valid via positions.

### 3. A* Search (Rust)

States are `(grid_x, grid_y, layer_index)` tuples. Moves include:
- 8 directions on same layer (orthogonal + diagonal)
- Via transitions to adjacent layers

Cost = distance + via penalty + **stub proximity penalty**. Heuristic = octile distance to goal.

The Rust implementation uses:
- FxHashSet for O(1) obstacle lookups
- BinaryHeap with counter-based tie-breaking for deterministic ordering
- Packed u64 state keys for fast hashing

### 4. Path Output

The grid path is converted back to mm coordinates. Segments and vias are generated for the KiCad file.

## Limitations

- Requires existing stub tracks to identify connection points
- Grid-based (0.1mm default) - may miss very tight fits
- No length matching or differential pair support
- No push-and-shove (routes around obstacles, doesn't move them)
- No rip-up and reroute (failed nets stay failed)

## DRC Checking

Verify routed output has no clearance violations:

```bash
python check_drc.py routed_output.kicad_pcb 0.1
```

The checker reports:
- **Segment-to-segment** violations (tracks too close on same layer)
- **Via-to-segment** violations (via too close to track)
- **Via-to-via** violations (vias too close together)

Note: Pre-existing differential pair routing (e.g., LVDS signals) may intentionally have tight spacing and will show as violations.

## Dependencies

- Python 3.7+
- Rust toolchain (for building the router module)
