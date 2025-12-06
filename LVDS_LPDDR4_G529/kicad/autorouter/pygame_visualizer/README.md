# PyGame PCB Routing Visualizer

Real-time visualization of the A* routing algorithm for PCB autorouting.

## Features

- **Real-time A* visualization** - Watch the Rust router explore the grid
- **Same algorithm as batch router** - Uses the exact same Rust A* implementation
- **Layer-based coloring** - Each copper layer has a distinct color
- **Persistent route display** - Completed routes remain visible when routing subsequent nets
- **Interactive controls** - Pause, step, zoom, pan, adjust speed
- **Search state display** - See open set (frontier), closed set (explored), final path

## Installation

```bash
# pygame-ce (Community Edition) has better Python version support
pip install pygame-ce

# Or standard pygame (may not work on Python 3.13+)
pip install pygame
```

The Rust router must also be built (same as for the batch router):

```bash
cd rust_router && cargo build --release
cp target/release/grid_router.dll grid_router.pyd  # Windows
```

## Usage

From the autorouter directory:

```bash
python pygame_visualizer/run_visualizer.py input.kicad_pcb "Net-Name"
```

Or as a module:

```bash
python -m pygame_visualizer.run_visualizer input.kicad_pcb "Net-Name"
```

### Wildcard Patterns

Use `*` and `?` wildcards to match multiple nets. All matching nets will be routed sequentially:

```bash
# Route and visualize all 32 DATA nets
python pygame_visualizer/run_visualizer.py fanout_starting_point.kicad_pcb "Net-(U2A-DATA_*)"
```

Each successfully routed net is added as an obstacle for subsequent routes, just like the batch router.

### Auto-Advance Mode

Use `--auto` to automatically progress through all nets without waiting for the N key:

```bash
python pygame_visualizer/run_visualizer.py --auto fanout_starting_point.kicad_pcb "Net-(U2A-DATA_*)"
```

This is useful for benchmarking or batch visualization. The visualizer will quit automatically when all nets are processed.

### Example

```bash
python pygame_visualizer/run_visualizer.py fanout_starting_point.kicad_pcb "Net-(U2A-DATA_0)"
```

## Keyboard Controls

| Key | Action |
|-----|--------|
| Space | Pause/Resume |
| S | Single step (when paused) |
| N | Next net (after route completes), or try backwards if forward failed |
| B | Force backwards direction (restart track & route backwards) |
| R | Restart current net |
| Ctrl+R | Restart all nets from beginning |
| +/- | Double/Halve speed (1x to 65536x) |
| 1-4 | Show layer 1-4 only |
| 0 | Show all layers |
| G | Toggle grid lines |
| O | Toggle open set display |
| C | Toggle closed set display |
| P | Toggle proximity field |
| H | Toggle legend |
| L | Toggle layer legend |
| Q/Esc | Quit |

## Mouse Controls

| Action | Effect |
|--------|--------|
| Scroll | Zoom in/out |
| Drag | Pan view |

## Color Legend

### Layers
- **Red** - F.Cu (Front copper)
- **Green** - In1.Cu (Inner layer 1)
- **Blue** - In2.Cu (Inner layer 2)
- **Magenta** - B.Cu (Back copper)

### Search State
- **Bright colors** - Open set (cells in priority queue)
- **Dark colors** - Closed set (already explored)
- **Yellow** - Current node being expanded
- **Cyan-green circles** - Source points
- **Red circles** - Target points

### Routes
- **Wide layer-colored lines** - Current net's route
- **Normal layer-colored lines** - Previously completed routes
- **White circles** - Vias (larger for current route)

### Obstacles
- **Light gray outline** - BGA exclusion zone
- **Gray outline** - Blocked cells (tracks, pads)
- **Gray X marks** - Blocked via positions

Note: Blocked cell/via indicators are hidden where completed routes exist for better visibility.

## Architecture

```
pygame_visualizer/
├── __init__.py          # Package exports
├── config.py            # Configuration and color schemes
├── visualizer.py        # PyGame rendering engine
├── run_visualizer.py    # Main entry point
└── README.md
```

### Key Classes

- **VisualRouter** (Rust) - A* router with `step()` method that returns search snapshots
- **SearchSnapshot** (Rust) - Contains iteration count, open/closed sets, path
- **RoutingVisualizer** - PyGame-based renderer with camera controls
- **Camera** - Pan/zoom handling

## How It Works

The visualizer uses the same Rust router as `batch_grid_router.py`, but with a different interface:

1. **VisualRouter** - A new Rust class that exposes incremental search:
   - `init(sources, targets, max_iterations)` - Initialize the search
   - `step(obstacles, num_iterations)` - Run N iterations, return snapshot
   - `is_done()` - Check if search is complete
   - `get_path()` - Get the final path

2. **SearchSnapshot** - Returned by `step()`, contains:
   - `iteration` - Current iteration count
   - `open_count`, `closed_count` - Set sizes
   - `open_cells`, `closed_cells` - Actual cell coordinates for rendering
   - `found` - Whether path was found
   - `path` - The final path (if found)

This approach has **zero impact on the normal batch router** - `GridRouter.route_multi()` is unchanged and runs at full speed.

## Performance Notes

- The Rust router runs at full native speed between visualization updates
- Snapshots are created only when needed (configurable iterations per frame)
- Cell lists are limited to 50,000 entries to prevent memory issues
- Pre-rendered obstacle surface is cached for efficient rendering
- **Incremental obstacle caching** - Base obstacle map is built once at startup (~0.6s), then cloned and updated per-net (~0.15-0.4s vs ~1.2s without caching)
- Results match the batch router exactly (verified on 32 DATA nets: 723,666 iterations)

## Integration

To integrate with your own code:

```python
from grid_router import GridObstacleMap, VisualRouter
from pygame_visualizer import RoutingVisualizer, VisualizerConfig

# Build obstacle map (same as batch router)
obstacles = GridObstacleMap(num_layers=4)
# ... add blocked cells, vias, etc.

# Set up visualizer
config = VisualizerConfig(layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'])
visualizer = RoutingVisualizer(config)
visualizer.set_routing_context(obstacles, sources, targets, ...)

# Initialize Rust router
router = VisualRouter(via_cost=500000, h_weight=1.5)
router.init(sources, targets, max_iterations=100000)

# Run with visualization
while not router.is_done():
    if visualizer.should_step():
        snapshot = router.step(obstacles, visualizer.iterations_per_frame)
        visualizer.update_snapshot(snapshot)
    visualizer.render()
    visualizer.tick()
    if not visualizer.handle_events():
        break
```
