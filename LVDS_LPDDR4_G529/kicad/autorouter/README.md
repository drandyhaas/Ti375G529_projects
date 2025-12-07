# KiCad PCB Grid Router

A fast Rust-accelerated A* autorouter for KiCad PCB files using integer grid coordinates. Routes nets between existing stub endpoints with multi-layer support and automatic via placement.

## Features

- **Grid-based A* pathfinding** - Integer coordinates for fast collision detection
- **Octilinear routing** - Horizontal, vertical, and 45-degree diagonal moves
- **Multi-layer routing** - Configurable layers with automatic via insertion
- **Obstacle avoidance** - Respects existing tracks, vias, and pads with configurable clearance
- **Rectangular pad blocking** - Proper clearance for non-square pads with rotation handling
- **Auto-detected BGA exclusion zones** - Prevents vias under BGA packages (detected from footprint)
- **Auto-detected component types** - BGA and QFN/QFP components detected by footprint pattern
- **Multiple net ordering strategies** - Inside-out (default), MPS (crossing conflicts), or original order
- **Inside-out BGA routing** - Nets with BGA pads are automatically sorted by distance from center
- **MPS ordering** - Maximum Planar Subset algorithm minimizes crossing conflicts between nets
- **Via-to-via clearance** - Proper spacing between vias (separate from track clearance)
- **Batch routing** - Routes multiple nets sequentially, each avoiding previously routed tracks
- **Stub proximity avoidance** - Penalizes routes near unrouted stubs to prevent blocking
- **Incremental obstacle caching** - Base obstacles built once, cloned per-net for ~7x speedup
- **Pad-only routing** - Routes nets with only pads (no stubs) or pad+stub combinations

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
| `pygame_visualizer/` | Real-time visualization with persistent route display |
| `MPS.html` | Interactive visualization of MPS crossing conflict algorithm |

## Quick Start

### Route Multiple Nets

```bash
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_0)" "Net-(U2A-DATA_1)"
```

### Route with Wildcard Patterns

Use `*` and `?` wildcards to match multiple nets:

```bash
# Route all DATA nets (matches Net-(U2A-DATA_0) through Net-(U2A-DATA_31))
python batch_grid_router.py fanout_starting_point.kicad_pcb routed.kicad_pcb "Net-(U2A-DATA_*)"

# Route all nets containing "CLK"
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "*CLK*"

# Mix wildcards and explicit names
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_*)" "Net-(U2A-CLK)"
```

### Net Ordering Strategies

The router supports three net ordering strategies via the `--ordering` flag:

```bash
# Inside-out ordering (default) - best for BGA fanout routing
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_*)" --ordering inside_out

# MPS ordering - minimizes crossing conflicts between nets
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_*)" --ordering mps

# Original ordering - routes nets in the order specified
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_*)" --ordering original
```

#### Strategy Comparison (32 DATA nets)

| Metric | inside_out | mps | original |
|--------|-----------|-----|----------|
| Success Rate | 32/32 (100%) | 32/32 (100%) | 32/32 (100%) |
| Time | 4.08s | 4.06s | 4.60s |
| Iterations | 723,666 | 1,029,824 | 1,071,958 |
| Via Count | 47 | 52 | 50 |

**Recommendation**: Use `inside_out` (default) for BGA fanout routing. The MPS algorithm is better suited for scenarios with actual crossing conflicts (e.g., connections between two rows of pins where net paths interleave).

### Route 32 DATA Nets (Inside-Out Ordering is Automatic)

The router automatically detects BGA components and sorts nets with BGA pads inside-out (center pads first). With wildcards, this is now a single command:

```bash
python batch_grid_router.py fanout_starting_point.kicad_pcb routed_output.kicad_pcb "Net-(U2A-DATA_*)"
```

The router will output:
```
Auto-detected 3 BGA exclusion zone(s):
  IC1: (135.7, 94.2) to (152.3, 110.8)
  U3: (185.9, 93.5) to (204.9, 112.5)
  U1: (165.3, 95.5) to (175.5, 110.5)

Sorted 32 BGA nets inside-out (0 non-BGA nets unchanged)
```

### Disable BGA Exclusion Zones

For nets that need to route directly between pads inside BGA areas (e.g., LVDS signals between BGAs), use `--no-bga-zones` to disable the BGA exclusion zone blocking:

```bash
# Route LVDS nets that connect pads inside BGA packages
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "*lvds*" --no-bga-zones
```

## Command-Line Options

All routing parameters can be configured via command-line arguments:

```bash
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-*)" [OPTIONS]
```

### Ordering and Strategy Options

| Option | Default | Description |
|--------|---------|-------------|
| `--ordering`, `-o` | `inside_out` | Net ordering: `inside_out`, `mps`, or `original` |
| `--direction`, `-d` | `forward` | Search direction: `forward`, `backwards`, or `random` |
| `--no-bga-zones` | (disabled) | Disable BGA exclusion zone detection |
| `--layers`, `-l` | `F.Cu In1.Cu In2.Cu B.Cu` | Routing layers to use |

### Track and Via Geometry

| Option | Default | Description |
|--------|---------|-------------|
| `--track-width` | `0.1` | Track width in mm |
| `--clearance` | `0.1` | Clearance between tracks in mm |
| `--via-size` | `0.3` | Via outer diameter in mm |
| `--via-drill` | `0.2` | Via drill size in mm |

### Router Algorithm Parameters

| Option | Default | Description |
|--------|---------|-------------|
| `--grid-step` | `0.1` | Grid resolution in mm |
| `--via-cost` | `25` | Penalty for placing a via (in grid steps) |
| `--max-iterations` | `100000` | Max A* iterations before giving up |
| `--heuristic-weight` | `1.5` | A* heuristic weight (higher = faster but less optimal) |

### Stub Proximity Penalty

| Option | Default | Description |
|--------|---------|-------------|
| `--stub-proximity-radius` | `1.5` | Radius around stubs to penalize routing (mm) |
| `--stub-proximity-cost` | `3.0` | Cost penalty near stubs (mm equivalent) |

### Example with Custom Parameters

```bash
# Route with wider tracks and larger clearance
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-*)" \
    --track-width 0.15 --clearance 0.15 --via-size 0.4 --via-drill 0.25

# Route with higher via penalty (fewer layer changes)
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-*)" \
    --via-cost 100

# Route with more iterations for difficult nets
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-*)" \
    --max-iterations 500000 --heuristic-weight 1.2

# Use specific layers
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-*)" \
    --layers F.Cu In1.Cu In2.Cu In8.Cu B.Cu
```

### Parameter Notes

- **clearance**: Edge-to-edge spacing between tracks. A 0.1mm clearance means tracks with 0.1mm width can pass with 0.2mm center-to-center spacing.
- **via-to-via spacing**: Automatically calculated as `via_size + clearance` (0.4mm with defaults). This is larger than track-to-via spacing to ensure proper clearance between via barrels.
- **heuristic_weight**: Values > 1.0 make the search greedier (faster but potentially suboptimal). 1.5 is a good balance.
- **max_iterations**: Increase for complex routes. Easy routes need ~200 iterations, hard routes may need 10,000+.
- **via_cost**: Higher values discourage layer changes. 25 means a via costs as much as 0.025 grid steps of travel (encourages using vias freely for optimal routing).
- **stub_proximity_radius/cost**: Routes passing near unrouted stub endpoints incur extra cost. This prevents early routes from blocking later ones. Vias near stubs are penalized 2x.
- **bga_exclusion_zones**: Automatically detected from BGA footprints in the PCB. Can be disabled with `--no-bga-zones`.
- **inside-out ordering**: Nets with pads inside a BGA zone are automatically sorted by distance from BGA center (closest first). This improves routing success for BGA escape routing.
- **mps ordering**: Maximum Planar Subset algorithm that detects crossing conflicts between nets and orders them to minimize blocking. See [MPS Algorithm](#mps-algorithm) section below.

## Performance

Tested on 32 DATA nets with BGA fanout:

| Metric | Value |
|--------|-------|
| Success Rate | **32/32 (100%)** |
| Total Time | ~3.5 seconds |
| Total Iterations | ~724,000 |
| DRC Violations | 0 (DATA nets) |

The batch router uses incremental obstacle caching for ~7x speedup over rebuilding obstacles from scratch for each net.

**Key features for high success rates:**
- **Stub proximity avoidance**: Routes are penalized for passing near unrouted stub endpoints, preventing early routes from blocking later ones
- **Forward-then-backwards search**: Try forward direction first, then backwards if forward fails. Some routes are only possible in one direction.
- **Rectangular pad blocking**: Proper clearance for non-square pads (e.g., QFN 0.2x0.875mm pads)
- **BGA zone blocking**: All layers blocked inside BGA exclusion zones (not just vias)

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

## MPS Algorithm

The Maximum Planar Subset (MPS) ordering strategy identifies crossing conflicts between nets and orders them to minimize blocking. This is useful when nets have paths that geometrically cross each other.

### How It Works

1. **Find endpoints**: For each net, identify the two stub endpoint centroids (source and target)
2. **Angular projection**: Project all endpoints onto a circular boundary centered on the routing region
3. **Detect crossings**: Two nets A and B cross if their endpoints interleave on the boundary:
   - Order A1, B1, A2, B2 means the paths must cross
   - Order A1, A2, B1, B2 means paths don't cross (can be routed without conflict)
4. **Build conflict graph**: Create edges between all pairs of crossing nets
5. **Greedy ordering**: In rounds:
   - Select nets with fewest active conflicts
   - Add them to the routing order
   - Remove conflicting neighbors (they go to next round)
6. **Output**: Ordered list where non-conflicting nets are routed first

### Interactive Visualization

Open `MPS.html` in a browser to see the algorithm in action:
- Draw nets by clicking pairs of pins on the circular boundary
- Click "Convert to Graph" to build the conflict graph
- Click "Run Micro-Step" repeatedly to watch the greedy selection process
- Green nets are selected for "Layer 1" (route first), red nets go to "Layer 2" (route later)

### When to Use MPS

MPS ordering is most effective when:
- Nets connect pins that are interleaved (e.g., alternating connections between two chip edges)
- Routes would naturally cross if drawn as straight lines
- The routing region is relatively flat (not radial like BGA fanout)

For BGA fanout routing where nets radiate from center, `inside_out` ordering typically performs better.

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

## Real-Time Visualizer

Watch the A* algorithm explore the routing grid in real-time using the PyGame visualizer:

```bash
pip install pygame-ce
python pygame_visualizer/run_visualizer.py input.kicad_pcb "Net-(U2A-DATA_0)"

# Auto-advance mode for benchmarking (no manual N key needed)
python pygame_visualizer/run_visualizer.py --auto input.kicad_pcb "Net-(U2A-DATA_*)"

# Disable BGA zones for pad-only nets (e.g., LVDS between BGAs)
python pygame_visualizer/run_visualizer.py --no-bga-zones input.kicad_pcb "*lvds*"

# Use same options as batch router
python pygame_visualizer/run_visualizer.py --auto --ordering mps --via-cost 50 input.kicad_pcb "Net-(U2A-*)"
```

Features:
- **Identical results to batch router** - Same Rust A* engine, same obstacle handling, same iteration counts
- **Same command-line options** - All batch_grid_router.py parameters supported
- Pause, step, zoom, pan controls
- Speed control with 2x scaling (1x, 2x, 4x, 8x... up to 65536x)
- Layer filtering and legend display
- Persistent route display (completed routes remain visible)
- Incremental obstacle caching for fast net setup
- Pad-only routing support (nets with only pads, no stubs)

Both routers use the same obstacle handling:
- **Routed nets**: segments, vias, and pads added as obstacles
- **Unrouted nets**: segments (stubs), vias (if any), and pads added as obstacles
- **Stub proximity costs**: applied to all remaining unrouted nets

See [pygame_visualizer/README.md](pygame_visualizer/README.md) for full documentation.

## Dependencies

- Python 3.7+
- Rust toolchain (for building the router module)
- pygame-ce (optional, for visualizer)
