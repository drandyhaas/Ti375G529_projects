# KiCad PCB Autorouter

A Python-based A* autorouter for KiCad PCB files. Routes nets between existing stub endpoints using octilinear (0°, 45°, 90°) routing with multi-layer support and automatic via placement.

## Features

- **A* pathfinding** with octilinear constraints (horizontal, vertical, and 45° diagonal)
- **Multi-layer routing** with automatic via insertion
- **Obstacle avoidance** - respects existing tracks, vias, and pads
- **Mid-segment connections** - can connect to any point along existing stubs, not just endpoints
- **Stub trimming** - automatically shortens destination stubs when connecting mid-segment
- **Path smoothing** - removes zigzag staircase patterns for cleaner routes
- **Batch routing** - routes multiple nets sequentially, each avoiding previously routed tracks

## Files

| File | Description |
|------|-------------|
| `astar_router.py` | Core A* routing engine with obstacle detection and path optimization |
| `batch_router.py` | Routes multiple nets sequentially with collision avoidance |
| `route_data_nets.py` | Example script routing DATA_0-DATA_11 with optimized ordering |
| `kicad_parser.py` | Parses .kicad_pcb files into Python data structures |
| `kicad_writer.py` | Generates KiCad S-expressions for segments and vias |

## Usage

### Route a Single Net

```bash
python astar_router.py input.kicad_pcb "Net-(U2A-DATA_0)" output.kicad_pcb
```

### Route Multiple Nets

```bash
python batch_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_0)" "Net-(U2A-DATA_1)" "Net-(U2A-DATA_2)"
```

### Route DATA_0 through DATA_11 (Optimized Order)

```bash
python route_data_nets.py
python route_data_nets.py input.kicad_pcb output.kicad_pcb
```

## How It Works

### 1. Stub Detection

The router expects each net to have existing "stub" tracks that fan out from component pads. It identifies the endpoints of these stubs (points that aren't connected to other segments or pads) and routes between them.

```
  Pad ----[stub]---- Endpoint A
                          |
                     [new route]
                          |
  Pad ----[stub]---- Endpoint B
```

### 2. A* Search

The router uses A* pathfinding on a grid (default 0.1mm resolution):

- **States**: (x, y, layer) tuples
- **Moves**: 8 directions on same layer + via transitions to other layers
- **Cost**: Distance traveled + via penalty (default 0.5mm equivalent per via)
- **Heuristic**: Euclidean distance to goal + via cost if layer differs

### 3. Obstacle Avoidance

Before routing, the router builds a spatial index of all obstacles:
- Existing tracks (with clearance expansion)
- Existing vias (blocking all layers)
- Component pads (except those on the net being routed)

The net being routed is excluded from obstacles, allowing the router to connect to its own stubs.

### 4. Connection Modes

The router tries three connection strategies in order:

1. **Endpoint to segment**: Start from source stub endpoint, connect to any point on target stub
2. **Endpoint to endpoint**: Traditional point-to-point routing
3. **Segment to segment**: Start from any point on source stub, connect to any point on target stub (escapes congested areas)

### 5. Path Optimization

After finding a path:
1. **Smoothing**: Replaces zigzag staircase patterns with clean horizontal + diagonal segments
2. **Simplification**: Merges collinear segments into single longer segments

### 6. Stub Trimming

When connecting mid-segment on a destination stub, the router:
- Identifies which part of the stub leads to the pad
- Keeps that portion, removes the rest
- Shortens the intersected segment to the connection point

## Configuration

Default routing parameters in `RouteConfig`:

```python
RouteConfig(
    track_width=0.1,    # mm - width of new tracks
    clearance=0.1,      # mm - minimum spacing between tracks
    via_size=0.3,       # mm - via outer diameter
    via_drill=0.2,      # mm - via drill size
    grid_step=0.1,      # mm - routing grid resolution
    via_cost=0.5,       # penalty for via (in mm equivalent)
    layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']  # available layers
)
```

## Routing Order Matters

For congested routing channels, the order in which nets are routed significantly affects success rate. Early routes become obstacles for later ones.

**Recommended strategy**: Route from both ends toward the middle (alternating). This prevents outer routes from blocking paths for inner ones.

```python
# Bad: Sequential order - inner nets get blocked
["DATA_0", "DATA_1", "DATA_2", ..., "DATA_11"]

# Good: Alternating from ends - each net has more room
["DATA_0", "DATA_11", "DATA_1", "DATA_10", "DATA_2", "DATA_9", ...]
```

## Limitations

- Requires existing stub tracks to identify connection points
- Grid-based routing (0.1mm default) - may miss tight fits
- No length matching or differential pair support
- No push-and-shove (routes around obstacles, doesn't move them)
- Maximum 100,000 iterations per route (configurable)

## Example Output

```
Loading fanout_starting_point.kicad_pcb...

Routing 12 nets...
============================================================

[1/12] Routing Net-(U2A-DATA_0) (id=302)
----------------------------------------
Routing from (205.100, 100.200) on In2.Cu
       to   (223.361, 94.920) on F.Cu
  Target stub has 2 segments, allowing mid-segment connection
Route found in 20044 iterations, path length: 165
  Connecting to segment at (221.200, 97.700)
Smoothed from 165 to 5 points
Simplified to 5 waypoints
Removing 1 segments beyond intersection
Shortening 1 segments at intersection
  SUCCESS: 3 segments, 1 vias

...

============================================================
Routing complete: 12 successful, 0 failed

Writing output to routed_output.kicad_pcb...
Successfully wrote routed_output.kicad_pcb
```

## Dependencies

- Python 3.7+
- No external packages required (uses only standard library)
