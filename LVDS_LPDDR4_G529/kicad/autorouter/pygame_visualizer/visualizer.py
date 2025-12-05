"""
PyGame-based real-time visualizer for PCB routing.

Renders the A* search progression in real-time using the Rust router with:
- Obstacles (tracks, pads, vias)
- BGA exclusion zones
- Open set (cells in priority queue)
- Closed set (explored cells)
- Current path being explored
- Final path when found
"""

import sys
from typing import List, Tuple, Optional, Set

try:
    import pygame
    from pygame import Surface, Rect
except ImportError:
    print("PyGame not installed. Install with: pip install pygame")
    sys.exit(1)

from .config import VisualizerConfig, LayerColors, SearchColors


class Camera:
    """2D camera for pan and zoom."""

    def __init__(self, screen_width: int, screen_height: int):
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.offset_x = 0.0  # World offset in pixels
        self.offset_y = 0.0
        self.zoom = 1.0
        self.min_zoom = 0.1
        self.max_zoom = 10.0

        # Drag state
        self.dragging = False
        self.drag_start = (0, 0)
        self.drag_offset_start = (0.0, 0.0)

    def world_to_screen(self, wx: float, wy: float) -> Tuple[int, int]:
        """Convert world coordinates to screen coordinates."""
        sx = int((wx + self.offset_x) * self.zoom)
        sy = int((wy + self.offset_y) * self.zoom)
        return (sx, sy)

    def screen_to_world(self, sx: int, sy: int) -> Tuple[float, float]:
        """Convert screen coordinates to world coordinates."""
        wx = sx / self.zoom - self.offset_x
        wy = sy / self.zoom - self.offset_y
        return (wx, wy)

    def zoom_at(self, screen_x: int, screen_y: int, factor: float):
        """Zoom centered on screen position."""
        wx, wy = self.screen_to_world(screen_x, screen_y)
        new_zoom = max(self.min_zoom, min(self.max_zoom, self.zoom * factor))
        if new_zoom == self.zoom:
            return
        self.zoom = new_zoom
        self.offset_x = screen_x / self.zoom - wx
        self.offset_y = screen_y / self.zoom - wy

    def start_drag(self, screen_x: int, screen_y: int):
        """Start panning."""
        self.dragging = True
        self.drag_start = (screen_x, screen_y)
        self.drag_offset_start = (self.offset_x, self.offset_y)

    def drag(self, screen_x: int, screen_y: int):
        """Continue panning."""
        if self.dragging:
            dx = (screen_x - self.drag_start[0]) / self.zoom
            dy = (screen_y - self.drag_start[1]) / self.zoom
            self.offset_x = self.drag_offset_start[0] + dx
            self.offset_y = self.drag_offset_start[1] + dy

    def end_drag(self):
        """End panning."""
        self.dragging = False

    def fit_to_bounds(self, min_x: float, min_y: float, max_x: float, max_y: float, padding: float = 50):
        """Fit view to world bounds."""
        world_width = max_x - min_x
        world_height = max_y - min_y

        if world_width <= 0 or world_height <= 0:
            return

        zoom_x = (self.screen_width - 2 * padding) / world_width
        zoom_y = (self.screen_height - 2 * padding) / world_height
        self.zoom = min(zoom_x, zoom_y)
        self.zoom = max(self.min_zoom, min(self.max_zoom, self.zoom))

        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        self.offset_x = self.screen_width / (2 * self.zoom) - center_x
        self.offset_y = self.screen_height / (2 * self.zoom) - center_y


class RoutingVisualizer:
    """Main visualizer class using the Rust router."""

    def __init__(self, config: VisualizerConfig = None):
        """Initialize visualizer."""
        self.config = config or VisualizerConfig()

        pygame.init()
        pygame.display.set_caption(self.config.title)

        self.screen = pygame.display.set_mode(
            (self.config.window_width, self.config.window_height),
            pygame.RESIZABLE
        )
        self.clock = pygame.time.Clock()
        self.camera = Camera(self.config.window_width, self.config.window_height)

        # Fonts
        self.font = pygame.font.SysFont('consolas', 14)
        self.font_large = pygame.font.SysFont('consolas', 18)

        # State
        self.running = True
        self.paused = False
        self.step_mode = False
        self.restart_requested = False      # R = restart current net
        self.restart_all_requested = False  # Ctrl+R = restart all nets
        self.next_net_requested = False     # N = advance to next net
        self.backwards_requested = False    # B = try backwards direction
        self.iterations_per_frame = self.config.iterations_per_frame

        # Current net info for display
        self.current_net_name = ""
        self.current_net_num = 0
        self.total_nets = 0
        self.status_message = ""  # Additional status message (e.g., "Forward failed, press N for reverse")

        # Routing data (from Rust)
        self.rust_obstacles = None  # GridObstacleMap from Rust
        self.sources: List[Tuple[int, int, int]] = []
        self.targets: List[Tuple[int, int, int]] = []
        self.grid_step: float = 0.1
        self.bounds: Tuple[float, float, float, float] = (0, 0, 100, 100)

        # BGA zones for display
        self.bga_zones: List[Tuple[int, int, int, int]] = []

        # Current search snapshot (from Rust VisualRouter)
        self.snapshot = None

        # Pre-rendered surfaces
        self._obstacle_surface: Optional[Surface] = None
        self._obstacle_dirty = True
        self._obstacle_offset = (0, 0)

        # Blocked cells cache for rendering (from Python obstacle building)
        self._blocked_cells_cache: List[Set[Tuple[int, int]]] = []
        self._blocked_vias_cache: Set[Tuple[int, int]] = set()

    def set_routing_context(
        self,
        rust_obstacles,  # GridObstacleMap from Rust module
        sources: List[Tuple[int, int, int]],
        targets: List[Tuple[int, int, int]],
        grid_step: float = 0.1,
        bounds: Tuple[float, float, float, float] = None,
        bga_zones: List[Tuple[int, int, int, int]] = None,
        blocked_cells: List[Set[Tuple[int, int]]] = None,
        blocked_vias: Set[Tuple[int, int]] = None,
    ):
        """Set the routing context."""
        self.rust_obstacles = rust_obstacles
        self.sources = list(sources)
        self.targets = list(targets)
        self.grid_step = grid_step
        self.bga_zones = bga_zones or []
        self._blocked_cells_cache = blocked_cells or []
        self._blocked_vias_cache = blocked_vias or set()
        self._obstacle_dirty = True

        if bounds:
            self.bounds = bounds
            min_gx = int(bounds[0] / grid_step)
            min_gy = int(bounds[1] / grid_step)
            max_gx = int(bounds[2] / grid_step)
            max_gy = int(bounds[3] / grid_step)
            cell_size = self.config.cell_size
            self.camera.fit_to_bounds(
                min_gx * cell_size, min_gy * cell_size,
                max_gx * cell_size, max_gy * cell_size
            )

    def update_snapshot(self, snapshot):
        """Update the current search snapshot from Rust VisualRouter."""
        self.snapshot = snapshot

    def set_current_net(self, net_name: str, net_num: int, total_nets: int):
        """Set the current net being routed for status display."""
        self.current_net_name = net_name
        self.current_net_num = net_num
        self.total_nets = total_nets

    def grid_to_screen(self, gx: int, gy: int) -> Tuple[int, int]:
        """Convert grid coordinates to screen coordinates."""
        cell_size = self.config.cell_size
        wx = gx * cell_size
        wy = gy * cell_size
        return self.camera.world_to_screen(wx, wy)

    def _render_obstacles_to_surface(self) -> Optional[Surface]:
        """Pre-render obstacles to a surface for efficiency."""
        min_gx = int(self.bounds[0] / self.grid_step) - 10
        min_gy = int(self.bounds[1] / self.grid_step) - 10
        max_gx = int(self.bounds[2] / self.grid_step) + 10
        max_gy = int(self.bounds[3] / self.grid_step) + 10

        cell_size = self.config.cell_size
        width = (max_gx - min_gx + 1) * cell_size
        height = (max_gy - min_gy + 1) * cell_size

        max_size = 4000
        if width > max_size or height > max_size:
            return None

        surface = Surface((width, height))
        surface.fill(SearchColors.BACKGROUND)

        # Render BGA zones
        for zone in self.bga_zones:
            zmin_gx, zmin_gy, zmax_gx, zmax_gy = zone
            rect = Rect(
                (zmin_gx - min_gx) * cell_size,
                (zmin_gy - min_gy) * cell_size,
                (zmax_gx - zmin_gx + 1) * cell_size,
                (zmax_gy - zmin_gy + 1) * cell_size
            )
            pygame.draw.rect(surface, SearchColors.BGA_ZONE, rect)

        # Render blocked cells from cache
        layer_idx = self.config.current_layer
        num_layers = len(self._blocked_cells_cache)

        if layer_idx >= 0 and layer_idx < num_layers:
            color = LayerColors.get_layer_color_by_index(layer_idx)
            color = tuple(c // 3 for c in color)
            for gx, gy in self._blocked_cells_cache[layer_idx]:
                if min_gx <= gx <= max_gx and min_gy <= gy <= max_gy:
                    rect = Rect(
                        (gx - min_gx) * cell_size,
                        (gy - min_gy) * cell_size,
                        cell_size, cell_size
                    )
                    pygame.draw.rect(surface, color, rect)
        else:
            for layer in range(num_layers):
                color = LayerColors.get_layer_color_by_index(layer)
                color = tuple(c // 4 for c in color)
                for gx, gy in self._blocked_cells_cache[layer]:
                    if min_gx <= gx <= max_gx and min_gy <= gy <= max_gy:
                        rect = Rect(
                            (gx - min_gx) * cell_size,
                            (gy - min_gy) * cell_size,
                            cell_size, cell_size
                        )
                        pygame.draw.rect(surface, color, rect)

        # Render blocked vias as small X marks
        for gx, gy in self._blocked_vias_cache:
            if min_gx <= gx <= max_gx and min_gy <= gy <= max_gy:
                x = (gx - min_gx) * cell_size
                y = (gy - min_gy) * cell_size
                # Draw X (two diagonal lines)
                pygame.draw.line(surface, SearchColors.VIA_BLOCKED,
                               (x + 1, y + 1), (x + cell_size - 2, y + cell_size - 2), 1)
                pygame.draw.line(surface, SearchColors.VIA_BLOCKED,
                               (x + cell_size - 2, y + 1), (x + 1, y + cell_size - 2), 1)

        self._obstacle_offset = (min_gx, min_gy)
        return surface

    def render(self):
        """Render the current state."""
        self.screen.fill(SearchColors.BACKGROUND)
        cell_size = self.config.cell_size

        # Render pre-cached obstacles
        if self._obstacle_dirty:
            self._obstacle_surface = self._render_obstacles_to_surface()
            self._obstacle_dirty = False

        if self._obstacle_surface and self.config.show_obstacles:
            min_gx, min_gy = self._obstacle_offset
            dest_x, dest_y = self.camera.world_to_screen(min_gx * cell_size, min_gy * cell_size)
            scaled_size = (
                int(self._obstacle_surface.get_width() * self.camera.zoom),
                int(self._obstacle_surface.get_height() * self.camera.zoom)
            )
            if scaled_size[0] > 0 and scaled_size[1] > 0:
                scaled = pygame.transform.scale(self._obstacle_surface, scaled_size)
                self.screen.blit(scaled, (dest_x, dest_y))

        # Render search state from Rust snapshot
        if self.snapshot:
            snapshot = self.snapshot
            layer_filter = self.config.current_layer

            # Closed set
            if self.config.show_closed_set and snapshot.closed_cells:
                for gx, gy, layer in snapshot.closed_cells:
                    if layer_filter >= 0 and layer != layer_filter:
                        continue
                    sx, sy = self.grid_to_screen(gx, gy)
                    size = int(cell_size * self.camera.zoom)
                    if 0 <= sx < self.config.window_width + size and 0 <= sy < self.config.window_height + size:
                        color = LayerColors.get_layer_color_by_index(layer)
                        color = tuple(c // 3 for c in color)
                        rect = Rect(sx, sy, max(1, size), max(1, size))
                        pygame.draw.rect(self.screen, color, rect)

            # Open set
            if self.config.show_open_set and snapshot.open_cells:
                for gx, gy, layer in snapshot.open_cells:
                    if layer_filter >= 0 and layer != layer_filter:
                        continue
                    sx, sy = self.grid_to_screen(gx, gy)
                    size = int(cell_size * self.camera.zoom)
                    if 0 <= sx < self.config.window_width + size and 0 <= sy < self.config.window_height + size:
                        color = LayerColors.get_layer_color_by_index(layer)
                        color = tuple(min(255, c + 80) for c in color)
                        rect = Rect(sx, sy, max(1, size), max(1, size))
                        pygame.draw.rect(self.screen, color, rect)

            # Final path
            if snapshot.path and len(snapshot.path) > 1:
                points = []
                prev_layer = snapshot.path[0][2]
                for gx, gy, layer in snapshot.path:
                    sx, sy = self.grid_to_screen(gx, gy)
                    sx += int(cell_size * self.camera.zoom // 2)
                    sy += int(cell_size * self.camera.zoom // 2)

                    if layer != prev_layer:
                        pygame.draw.circle(self.screen, (255, 255, 255), (sx, sy), max(3, int(4 * self.camera.zoom)))
                    points.append((sx, sy))
                    prev_layer = layer

                if len(points) > 1:
                    pygame.draw.lines(self.screen, SearchColors.PATH_FOUND, False, points, max(2, int(3 * self.camera.zoom)))

            # Current node
            if snapshot.current:
                gx, gy, layer = snapshot.current
                sx, sy = self.grid_to_screen(gx, gy)
                size = int(cell_size * self.camera.zoom)
                rect = Rect(sx, sy, max(2, size), max(2, size))
                pygame.draw.rect(self.screen, SearchColors.CURRENT, rect)

        # Render sources and targets
        for gx, gy, _ in self.sources:
            sx, sy = self.grid_to_screen(gx, gy)
            size = max(4, int(cell_size * self.camera.zoom))
            pygame.draw.circle(self.screen, SearchColors.SOURCE,
                             (sx + size // 2, sy + size // 2), size // 2 + 2)

        for gx, gy, _ in self.targets:
            sx, sy = self.grid_to_screen(gx, gy)
            size = max(4, int(cell_size * self.camera.zoom))
            pygame.draw.circle(self.screen, SearchColors.TARGET,
                             (sx + size // 2, sy + size // 2), size // 2 + 2)

        self._render_ui()
        pygame.display.flip()

    def _render_ui(self):
        """Render UI overlay."""
        y_offset = 10

        if self.paused:
            status = "PAUSED (Space to resume, S to step)"
            color = (255, 200, 100)
        else:
            status = f"Running ({self.iterations_per_frame} iter/frame)"
            color = (100, 255, 100)

        text = self.font.render(status, True, color)
        self.screen.blit(text, (10, y_offset))
        y_offset += 20

        # Show current net being routed
        if self.current_net_name:
            net_status = f"Net [{self.current_net_num}/{self.total_nets}]: {self.current_net_name}"
            text = self.font.render(net_status, True, (150, 200, 255))
            self.screen.blit(text, (10, y_offset))
            y_offset += 20

        # Show additional status message (e.g., forward failed)
        if self.status_message:
            text = self.font.render(self.status_message, True, (255, 150, 100))
            self.screen.blit(text, (10, y_offset))
            y_offset += 20

        if self.snapshot and self.config.show_stats:
            snapshot = self.snapshot
            info_lines = [
                f"Iteration: {snapshot.iteration}",
                f"Open: {snapshot.open_count}",
                f"Closed: {snapshot.closed_count}",
            ]
            if snapshot.found:
                path_len = len(snapshot.path) if snapshot.path else 0
                info_lines.append(f"PATH FOUND! Length: {path_len}")
                info_lines.append("Press N for next net")

            for line in info_lines:
                text = self.font.render(line, True, (200, 200, 200))
                self.screen.blit(text, (10, y_offset))
                y_offset += 18

        # Layer legend (bottom-left)
        if self.config.show_layer_legend:
            y_offset = self.config.window_height - 100
            text = self.font.render("Layers (1-4, 0=all):", True, (200, 200, 200))
            self.screen.blit(text, (10, y_offset))
            y_offset += 18

            for i, layer_name in enumerate(self.config.layers):
                color = LayerColors.get_layer_color(layer_name)
                indicator = " *" if self.config.current_layer == i else ""
                if self.config.current_layer < 0:
                    indicator = ""
                text = self.font.render(f"  {i+1}: {layer_name}{indicator}", True, color)
                self.screen.blit(text, (10, y_offset))
                y_offset += 16

        # Speed indicator (top-right)
        speed_text = f"+/- Speed: {self.iterations_per_frame}x"
        text = self.font.render(speed_text, True, (150, 150, 150))
        self.screen.blit(text, (self.config.window_width - 160, 10))

        # Legend (right side)
        if self.config.show_legend:
            self._render_legend()

    def _render_legend(self):
        """Render the legend explaining what each visual element represents."""
        x_offset = self.config.window_width - 220
        y_offset = 40
        line_height = 18
        swatch_size = 12

        # Background box for legend
        legend_height = 280
        legend_rect = Rect(x_offset - 10, y_offset - 5, 220, legend_height)
        pygame.draw.rect(self.screen, (40, 40, 45), legend_rect)
        pygame.draw.rect(self.screen, (80, 80, 85), legend_rect, 1)

        # Title
        text = self.font.render("Legend", True, (220, 220, 220))
        self.screen.blit(text, (x_offset, y_offset))
        y_offset += line_height + 5

        # Search state section
        text = self.font.render("Search State:", True, (180, 180, 180))
        self.screen.blit(text, (x_offset, y_offset))
        y_offset += line_height

        # Get current layer color for accurate legend
        layer_idx = self.config.current_layer if self.config.current_layer >= 0 else 0
        layer_color = LayerColors.get_layer_color_by_index(layer_idx)
        open_color = tuple(min(255, c + 80) for c in layer_color)
        closed_color = tuple(c // 3 for c in layer_color)

        legend_items = [
            (SearchColors.SOURCE, "Source (start)"),
            (SearchColors.TARGET, "Target (goal)"),
            (SearchColors.CURRENT, "Current node"),
            (open_color, "Open set (frontier)"),
            (closed_color, "Closed set (explored)"),
            (SearchColors.PATH_FOUND, "Final path"),
            ((255, 255, 255), "Via (layer change)"),
        ]

        for color, label in legend_items:
            # Draw color swatch
            swatch_rect = Rect(x_offset + 5, y_offset + 2, swatch_size, swatch_size)
            pygame.draw.rect(self.screen, color, swatch_rect)
            pygame.draw.rect(self.screen, (100, 100, 100), swatch_rect, 1)
            # Draw label
            text = self.font.render(label, True, (200, 200, 200))
            self.screen.blit(text, (x_offset + swatch_size + 12, y_offset))
            y_offset += line_height

        y_offset += 5

        # Obstacles section
        text = self.font.render("Obstacles:", True, (180, 180, 180))
        self.screen.blit(text, (x_offset, y_offset))
        y_offset += line_height

        obstacle_items = [
            (SearchColors.BGA_ZONE, "BGA exclusion zone"),
            ((60, 40, 40), "Blocked cells"),
            (SearchColors.VIA_BLOCKED, "Blocked via positions"),
        ]

        for color, label in obstacle_items:
            swatch_rect = Rect(x_offset + 5, y_offset + 2, swatch_size, swatch_size)
            pygame.draw.rect(self.screen, color, swatch_rect)
            pygame.draw.rect(self.screen, (100, 100, 100), swatch_rect, 1)
            text = self.font.render(label, True, (200, 200, 200))
            self.screen.blit(text, (x_offset + swatch_size + 12, y_offset))
            y_offset += line_height

        # Controls legend (bottom-right)
        self._render_controls_legend()

    def _render_controls_legend(self):
        """Render keyboard/mouse controls in bottom-right."""
        line_height = 16
        x_offset = self.config.window_width - 220

        controls = [
            ("Space", "Pause/Resume"),
            ("S", "Step (paused)"),
            ("N", "Next net"),
            ("R", "Restart net"),
            ("Ctrl+R", "Restart all"),
            ("+/-", "Speed 2x"),
            ("1-4", "Layer filter"),
            ("0", "All layers"),
            ("O/C", "Open/Closed set"),
            ("H", "Toggle legend"),
            ("Scroll", "Zoom"),
            ("Drag", "Pan"),
            ("Q/Esc", "Quit"),
        ]

        box_height = len(controls) * line_height + 25
        y_start = self.config.window_height - box_height - 10

        # Background box
        legend_rect = Rect(x_offset - 10, y_start, 220, box_height)
        pygame.draw.rect(self.screen, (40, 40, 45), legend_rect)
        pygame.draw.rect(self.screen, (80, 80, 85), legend_rect, 1)

        y_offset = y_start + 5

        # Title
        text = self.font.render("Controls", True, (220, 220, 220))
        self.screen.blit(text, (x_offset, y_offset))
        y_offset += line_height + 3

        for key, action in controls:
            key_text = self.font.render(f"{key}:", True, (150, 200, 150))
            action_text = self.font.render(action, True, (180, 180, 180))
            self.screen.blit(key_text, (x_offset + 5, y_offset))
            self.screen.blit(action_text, (x_offset + 70, y_offset))
            y_offset += line_height

    def handle_events(self) -> bool:
        """Handle pygame events. Returns False if should quit."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False

            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    return False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_s:
                    if self.paused:
                        self.step_mode = True
                elif event.key == pygame.K_r:
                    mods = pygame.key.get_mods()
                    if mods & pygame.KMOD_CTRL:
                        # Ctrl+R = restart all nets
                        self.restart_all_requested = True
                    else:
                        # R = restart current net
                        self.restart_requested = True
                    self.snapshot = None
                elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                    # 2x speed increase
                    self.iterations_per_frame = min(
                        self.config.max_speed,
                        self.iterations_per_frame * 2
                    )
                elif event.key == pygame.K_MINUS:
                    # 2x speed decrease (halve)
                    self.iterations_per_frame = max(
                        self.config.min_speed,
                        self.iterations_per_frame // 2
                    )
                elif event.key == pygame.K_g:
                    self.config.show_grid_lines = not self.config.show_grid_lines
                    self._obstacle_dirty = True
                elif event.key == pygame.K_o:
                    self.config.show_open_set = not self.config.show_open_set
                elif event.key == pygame.K_c:
                    self.config.show_closed_set = not self.config.show_closed_set
                elif event.key == pygame.K_p:
                    self.config.show_proximity = not self.config.show_proximity
                elif event.key == pygame.K_l:
                    self.config.show_layer_legend = not self.config.show_layer_legend
                elif event.key == pygame.K_h:
                    self.config.show_legend = not self.config.show_legend
                elif event.key == pygame.K_n:
                    self.next_net_requested = True
                elif event.key == pygame.K_b:
                    self.backwards_requested = True
                elif event.key == pygame.K_0:
                    self.config.current_layer = -1
                    self._obstacle_dirty = True
                elif pygame.K_1 <= event.key <= pygame.K_9:
                    layer = event.key - pygame.K_1
                    if layer < len(self.config.layers):
                        self.config.current_layer = layer
                        self._obstacle_dirty = True

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    self.camera.start_drag(*event.pos)
                elif event.button == 4:
                    self.camera.zoom_at(*event.pos, 1.2)
                elif event.button == 5:
                    self.camera.zoom_at(*event.pos, 1 / 1.2)

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    self.camera.end_drag()

            elif event.type == pygame.MOUSEMOTION:
                if self.camera.dragging:
                    self.camera.drag(*event.pos)

            elif event.type == pygame.VIDEORESIZE:
                self.config.window_width = event.w
                self.config.window_height = event.h
                self.camera.screen_width = event.w
                self.camera.screen_height = event.h

        return True

    def should_step(self) -> bool:
        """Check if we should advance the search."""
        if not self.paused:
            return True
        if self.step_mode:
            self.step_mode = False
            return True
        return False

    def tick(self):
        """Advance frame timing."""
        self.clock.tick(self.config.target_fps)

    def quit(self):
        """Clean up pygame."""
        pygame.quit()
