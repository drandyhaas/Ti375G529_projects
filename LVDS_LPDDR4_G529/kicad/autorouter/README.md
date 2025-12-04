# KiCad PCB Grid Router

A fast Python-based A* autorouter for KiCad PCB files using integer grid coordinates. Routes nets between existing stub endpoints with multi-layer support and automatic via placement.

## Features

- **Grid-based A* pathfinding** - Integer coordinates for fast collision detection
- **Octilinear routing** - Horizontal, vertical, and 45-degree diagonal moves
- **Multi-layer routing** - 4 layers (F.Cu, In1.Cu, In2.Cu, B.Cu) with automatic via insertion
- **Obstacle avoidance** - Respects existing tracks, vias, and pads with configurable clearance
- **BGA exclusion zone** - Prevents vias under BGA packages
- **Batch routing** - Routes multiple nets sequentially, each avoiding previously routed tracks

## Files

| File | Description |
|------|-------------|
| `grid_astar_router.py` | Core grid-based A* routing engine |
| `batch_grid_router.py` | Batch routing for multiple nets |
| `kicad_parser.py` | Parses .kicad_pcb files into Python data structures |
| `kicad_writer.py` | Generates KiCad S-expressions for segments and vias |

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
    max_iterations=25000,   # max A* iterations per route
    heuristic_weight=1.5,   # A* greediness (1.0=optimal, >1.0=faster)
    bga_exclusion_zone=(185.9, 93.5, 204.9, 112.5),  # no vias in this rectangle
)
```

### Parameter Notes

- **clearance**: Edge-to-edge spacing between tracks. A 0.1mm clearance means tracks with 0.1mm width can pass with 0.2mm center-to-center spacing.
- **heuristic_weight**: Values > 1.0 make the search greedier (faster but potentially suboptimal). 1.5 is a good balance.
- **max_iterations**: Increase for complex routes. Easy routes need ~200 iterations, hard routes may need 10,000+.
- **via_cost**: Higher values discourage layer changes. 500 means a via costs as much as 0.5 grid steps of travel.

## Performance

Tested on 32 DATA nets with BGA fanout:

| Tracks | Success Rate | Time | Notes |
|--------|-------------|------|-------|
| 1 | 100% | 24ms | 160 iterations |
| 4 | 100% | 116ms | 160-893 iterations |
| 8 | 100% | 908ms | 160-12873 iterations |
| 16 | 56% | 2.6s | Some routes blocked |
| 32 (inside-out) | **97%** | **12s** | 31/32 successful |
| 32 (outside-in) | 88% | 28s | 28/32 successful |

### Routing Order Comparison

| Order | Success | Time | Failed Nets |
|-------|---------|------|-------------|
| Inside-out | 31/32 (97%) | 12s | DATA_5 |
| Outside-in | 28/32 (88%) | 28s | DATA_9, DATA_12, DATA_14, DATA_20 |

**Inside-out is recommended** for BGA breakout routing. Inner nets route first and establish clear escape paths before outer nets can block them.

## How It Works

### 1. Grid Discretization

Coordinates are converted to integers: `grid_x = round(mm_x / grid_step)`

This enables fast set-based collision detection with O(1) lookups.

### 2. Obstacle Map

Before routing, all obstacles are rasterized to grid cells:
- Existing tracks (expanded by half-width + clearance)
- Existing vias (all layers blocked)
- Component pads

The net being routed is excluded from obstacles.

### 3. A* Search

States are `(grid_x, grid_y, layer_index)` tuples. Moves include:
- 8 directions on same layer (orthogonal + diagonal)
- Via transitions to adjacent layers

Cost = manhattan/diagonal distance + via penalty. Heuristic = octile distance to goal.

### 4. Path Output

The grid path is simplified (collinear points merged) and converted back to mm coordinates. Segments and vias are generated for the KiCad file.

## Limitations

- Requires existing stub tracks to identify connection points
- Grid-based (0.1mm default) - may miss very tight fits
- No length matching or differential pair support
- No push-and-shove (routes around obstacles, doesn't move them)
- No rip-up and reroute (failed nets stay failed)

## Dependencies

- Python 3.7+
- No external packages required (standard library only)
