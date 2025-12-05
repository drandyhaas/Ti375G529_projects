//! Grid-based A* PCB Router - Rust implementation for speed.
//!
//! This is a high-performance implementation of the grid router algorithm.
//! It's designed to be called from Python via PyO3 bindings.

use pyo3::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// Grid state: (x, y, layer) packed into a single u64 for fast hashing
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
struct GridState {
    gx: i32,
    gy: i32,
    layer: u8,
}

impl GridState {
    #[inline]
    fn new(gx: i32, gy: i32, layer: u8) -> Self {
        Self { gx, gy, layer }
    }

    #[inline]
    fn as_key(&self) -> u64 {
        // Pack into u64 for fast hashing: 20 bits x, 20 bits y, 8 bits layer
        let x = (self.gx as u64) & 0xFFFFF;
        let y = (self.gy as u64) & 0xFFFFF;
        let l = self.layer as u64;
        (x << 28) | (y << 8) | l
    }
}

/// A* open set entry with reverse ordering for min-heap
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct OpenEntry {
    f_score: i32,
    g_score: i32,
    state: GridState,
    counter: u32, // Tie-breaker for deterministic ordering
}

impl Ord for OpenEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap (lowest f_score first)
        other.f_score.cmp(&self.f_score)
            .then_with(|| other.counter.cmp(&self.counter))
    }
}

impl PartialOrd for OpenEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// 8 directions for octilinear routing
const DIRECTIONS: [(i32, i32); 8] = [
    (1, 0),   // East
    (1, -1),  // NE
    (0, -1),  // North
    (-1, -1), // NW
    (-1, 0),  // West
    (-1, 1),  // SW
    (0, 1),   // South
    (1, 1),   // SE
];

const ORTHO_COST: i32 = 1000;
const DIAG_COST: i32 = 1414; // sqrt(2) * 1000

/// Grid-based obstacle map
#[pyclass]
struct GridObstacleMap {
    /// Blocked cells per layer: layer -> set of (gx, gy) packed as u64
    blocked_cells: Vec<FxHashSet<u64>>,
    /// Blocked via positions
    blocked_vias: FxHashSet<u64>,
    /// Stub proximity costs: (gx, gy) -> cost
    stub_proximity: FxHashMap<u64, i32>,
    /// Number of layers
    num_layers: usize,
    /// BGA exclusion zone (min_gx, min_gy, max_gx, max_gy)
    bga_zone: Option<(i32, i32, i32, i32)>,
    /// Allowed cells that override BGA zone (for source/target points)
    allowed_cells: FxHashSet<u64>,
}

#[inline]
fn pack_xy(gx: i32, gy: i32) -> u64 {
    let x = (gx as u64) & 0xFFFFFFFF;
    let y = (gy as u64) & 0xFFFFFFFF;
    (x << 32) | y
}

#[pymethods]
impl GridObstacleMap {
    #[new]
    fn new(num_layers: usize) -> Self {
        Self {
            blocked_cells: (0..num_layers).map(|_| FxHashSet::default()).collect(),
            blocked_vias: FxHashSet::default(),
            stub_proximity: FxHashMap::default(),
            num_layers,
            bga_zone: None,
            allowed_cells: FxHashSet::default(),
        }
    }

    /// Add an allowed cell that overrides BGA zone blocking
    fn add_allowed_cell(&mut self, gx: i32, gy: i32) {
        self.allowed_cells.insert(pack_xy(gx, gy));
    }

    /// Set BGA exclusion zone
    fn set_bga_zone(&mut self, min_gx: i32, min_gy: i32, max_gx: i32, max_gy: i32) {
        self.bga_zone = Some((min_gx, min_gy, max_gx, max_gy));
    }

    /// Add a blocked cell
    fn add_blocked_cell(&mut self, gx: i32, gy: i32, layer: usize) {
        if layer < self.num_layers {
            self.blocked_cells[layer].insert(pack_xy(gx, gy));
        }
    }

    /// Add a blocked via position
    fn add_blocked_via(&mut self, gx: i32, gy: i32) {
        self.blocked_vias.insert(pack_xy(gx, gy));
    }

    /// Set stub proximity cost
    fn set_stub_proximity(&mut self, gx: i32, gy: i32, cost: i32) {
        let key = pack_xy(gx, gy);
        let existing = self.stub_proximity.get(&key).copied().unwrap_or(0);
        if cost > existing {
            self.stub_proximity.insert(key, cost);
        }
    }

    /// Check if cell is blocked
    #[inline]
    fn is_blocked(&self, gx: i32, gy: i32, layer: usize) -> bool {
        // First check if blocked by regular obstacles (segments, vias, pads)
        if layer >= self.num_layers {
            return true;
        }
        if self.blocked_cells[layer].contains(&pack_xy(gx, gy)) {
            return true;
        }

        // Check BGA zone - but allowed_cells can override this
        if let Some((min_gx, min_gy, max_gx, max_gy)) = self.bga_zone {
            if gx >= min_gx && gx <= max_gx && gy >= min_gy && gy <= max_gy {
                // Inside BGA zone - blocked unless explicitly allowed
                return !self.allowed_cells.contains(&pack_xy(gx, gy));
            }
        }

        false
    }

    /// Check if via is blocked
    #[inline]
    fn is_via_blocked(&self, gx: i32, gy: i32) -> bool {
        self.blocked_vias.contains(&pack_xy(gx, gy))
    }

    /// Get stub proximity cost
    #[inline]
    fn get_stub_proximity_cost(&self, gx: i32, gy: i32) -> i32 {
        self.stub_proximity.get(&pack_xy(gx, gy)).copied().unwrap_or(0)
    }
}

/// Grid A* Router
#[pyclass]
struct GridRouter {
    via_cost: i32,
    h_weight: f32,
}

#[pymethods]
impl GridRouter {
    #[new]
    fn new(via_cost: i32, h_weight: f32) -> Self {
        Self { via_cost, h_weight }
    }

    /// Route from multiple source points to multiple target points.
    /// Returns (path, iterations) where path is list of (gx, gy, layer) tuples,
    /// or (None, iterations) if no path found.
    fn route_multi(
        &self,
        obstacles: &GridObstacleMap,
        sources: Vec<(i32, i32, u8)>,
        targets: Vec<(i32, i32, u8)>,
        max_iterations: u32,
    ) -> (Option<Vec<(i32, i32, u8)>>, u32) {
        // Convert targets to set for O(1) lookup
        let target_set: FxHashSet<u64> = targets
            .iter()
            .map(|(gx, gy, layer)| GridState::new(*gx, *gy, *layer).as_key())
            .collect();

        let target_states: Vec<GridState> = targets
            .iter()
            .map(|(gx, gy, layer)| GridState::new(*gx, *gy, *layer))
            .collect();

        // Initialize open set with all sources
        let mut open_set = BinaryHeap::new();
        let mut g_costs: FxHashMap<u64, i32> = FxHashMap::default();
        let mut parents: FxHashMap<u64, u64> = FxHashMap::default();
        let mut closed: FxHashSet<u64> = FxHashSet::default();
        let mut counter: u32 = 0;

        for (gx, gy, layer) in sources {
            let state = GridState::new(gx, gy, layer);
            let key = state.as_key();
            let h = self.heuristic_to_targets(&state, &target_states);
            open_set.push(OpenEntry {
                f_score: h,
                g_score: 0,
                state,
                counter,
            });
            counter += 1;
            g_costs.insert(key, 0);
        }

        let mut iterations: u32 = 0;

        while let Some(current_entry) = open_set.pop() {
            iterations += 1;
            if iterations > max_iterations {
                break;
            }

            let current = current_entry.state;
            let current_key = current.as_key();
            let g = current_entry.g_score;

            if closed.contains(&current_key) {
                continue;
            }
            closed.insert(current_key);

            // Check if reached target
            if target_set.contains(&current_key) {
                // Reconstruct path
                let path = self.reconstruct_path(&parents, current_key, &g_costs);
                return (Some(path), iterations);
            }

            // Expand neighbors - 8 directions
            for (dx, dy) in DIRECTIONS {
                let ngx = current.gx + dx;
                let ngy = current.gy + dy;

                if obstacles.is_blocked(ngx, ngy, current.layer as usize) {
                    continue;
                }

                let neighbor = GridState::new(ngx, ngy, current.layer);
                let neighbor_key = neighbor.as_key();

                if closed.contains(&neighbor_key) {
                    continue;
                }

                let move_cost = if dx != 0 && dy != 0 { DIAG_COST } else { ORTHO_COST };
                let proximity_cost = obstacles.get_stub_proximity_cost(ngx, ngy);
                let new_g = g + move_cost + proximity_cost;

                let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                if new_g < existing_g {
                    g_costs.insert(neighbor_key, new_g);
                    parents.insert(neighbor_key, current_key);
                    let h = self.heuristic_to_targets(&neighbor, &target_states);
                    let f = new_g + h;
                    open_set.push(OpenEntry {
                        f_score: f,
                        g_score: new_g,
                        state: neighbor,
                        counter,
                    });
                    counter += 1;
                }
            }

            // Try via to other layers
            if !obstacles.is_via_blocked(current.gx, current.gy) {
                for layer in 0..obstacles.num_layers as u8 {
                    if layer == current.layer {
                        continue;
                    }

                    let neighbor = GridState::new(current.gx, current.gy, layer);
                    let neighbor_key = neighbor.as_key();

                    if closed.contains(&neighbor_key) {
                        continue;
                    }

                    let proximity_cost = obstacles.get_stub_proximity_cost(current.gx, current.gy) * 2;
                    let new_g = g + self.via_cost + proximity_cost;

                    let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                    if new_g < existing_g {
                        g_costs.insert(neighbor_key, new_g);
                        parents.insert(neighbor_key, current_key);
                        let h = self.heuristic_to_targets(&neighbor, &target_states);
                        let f = new_g + h;
                        open_set.push(OpenEntry {
                            f_score: f,
                            g_score: new_g,
                            state: neighbor,
                            counter,
                        });
                        counter += 1;
                    }
                }
            }
        }

        (None, iterations)
    }
}

impl GridRouter {
    /// Octile distance heuristic to nearest target
    #[inline]
    fn heuristic_to_targets(&self, state: &GridState, targets: &[GridState]) -> i32 {
        let mut min_h = i32::MAX;
        for target in targets {
            let dx = (state.gx - target.gx).abs();
            let dy = (state.gy - target.gy).abs();
            let diag = dx.min(dy);
            let orth = (dx - dy).abs();
            let mut h = diag * DIAG_COST + orth * ORTHO_COST;
            if state.layer != target.layer {
                h += self.via_cost;
            }
            min_h = min_h.min(h);
        }
        (min_h as f32 * self.h_weight) as i32
    }

    /// Reconstruct path from parents map
    fn reconstruct_path(
        &self,
        parents: &FxHashMap<u64, u64>,
        goal_key: u64,
        g_costs: &FxHashMap<u64, i32>,
    ) -> Vec<(i32, i32, u8)> {
        let mut path = Vec::new();
        let mut current_key = goal_key;

        loop {
            // Unpack key back to state
            let layer = (current_key & 0xFF) as u8;
            let y = ((current_key >> 8) & 0xFFFFF) as i32;
            let x = ((current_key >> 28) & 0xFFFFF) as i32;
            // Handle negative coordinates (sign extension)
            let x = if x & 0x80000 != 0 { x | !0xFFFFF_i32 } else { x };
            let y = if y & 0x80000 != 0 { y | !0xFFFFF_i32 } else { y };

            path.push((x, y, layer));

            match parents.get(&current_key) {
                Some(&parent_key) => current_key = parent_key,
                None => break,
            }
        }

        path.reverse();
        path
    }
}

/// Search state snapshot for visualization
#[pyclass]
#[derive(Clone)]
struct SearchSnapshot {
    #[pyo3(get)]
    iteration: u32,
    #[pyo3(get)]
    current: Option<(i32, i32, u8)>,
    #[pyo3(get)]
    open_count: usize,
    #[pyo3(get)]
    closed_count: usize,
    #[pyo3(get)]
    found: bool,
    #[pyo3(get)]
    path: Option<Vec<(i32, i32, u8)>>,
    /// Closed set cells for visualization (sampled if too large)
    #[pyo3(get)]
    closed_cells: Vec<(i32, i32, u8)>,
    /// Open set cells for visualization (sampled if too large)
    #[pyo3(get)]
    open_cells: Vec<(i32, i32, u8)>,
}

#[pymethods]
impl SearchSnapshot {
    fn __repr__(&self) -> String {
        format!(
            "SearchSnapshot(iter={}, open={}, closed={}, found={})",
            self.iteration, self.open_count, self.closed_count, self.found
        )
    }
}

/// Iterator-based router for visualization
#[pyclass]
struct VisualRouter {
    via_cost: i32,
    h_weight: f32,
    // Search state
    open_set: BinaryHeap<OpenEntry>,
    g_costs: FxHashMap<u64, i32>,
    parents: FxHashMap<u64, u64>,
    closed: FxHashSet<u64>,
    counter: u32,
    iterations: u32,
    max_iterations: u32,
    target_set: FxHashSet<u64>,
    target_states: Vec<GridState>,
    // Result
    found: bool,
    final_path: Option<Vec<(i32, i32, u8)>>,
}

#[pymethods]
impl VisualRouter {
    #[new]
    fn new(via_cost: i32, h_weight: f32) -> Self {
        Self {
            via_cost,
            h_weight,
            open_set: BinaryHeap::new(),
            g_costs: FxHashMap::default(),
            parents: FxHashMap::default(),
            closed: FxHashSet::default(),
            counter: 0,
            iterations: 0,
            max_iterations: 0,
            target_set: FxHashSet::default(),
            target_states: Vec::new(),
            found: false,
            final_path: None,
        }
    }

    /// Initialize the search with sources and targets
    fn init(
        &mut self,
        sources: Vec<(i32, i32, u8)>,
        targets: Vec<(i32, i32, u8)>,
        max_iterations: u32,
    ) {
        // Reset state
        self.open_set.clear();
        self.g_costs.clear();
        self.parents.clear();
        self.closed.clear();
        self.counter = 0;
        self.iterations = 0;
        self.max_iterations = max_iterations;
        self.found = false;
        self.final_path = None;

        // Set up targets
        self.target_set = targets
            .iter()
            .map(|(gx, gy, layer)| GridState::new(*gx, *gy, *layer).as_key())
            .collect();
        self.target_states = targets
            .iter()
            .map(|(gx, gy, layer)| GridState::new(*gx, *gy, *layer))
            .collect();

        // Initialize open set with sources
        for (gx, gy, layer) in sources {
            let state = GridState::new(gx, gy, layer);
            let key = state.as_key();
            let h = self.heuristic_to_targets(&state);
            self.open_set.push(OpenEntry {
                f_score: h,
                g_score: 0,
                state,
                counter: self.counter,
            });
            self.counter += 1;
            self.g_costs.insert(key, 0);
        }
    }

    /// Run N iterations of the search, returns snapshot of current state
    fn step(&mut self, obstacles: &GridObstacleMap, num_iterations: u32) -> SearchSnapshot {
        let mut current_node: Option<GridState> = None;

        for _ in 0..num_iterations {
            if self.found || self.open_set.is_empty() || self.iterations >= self.max_iterations {
                break;
            }

            self.iterations += 1;

            let current_entry = match self.open_set.pop() {
                Some(e) => e,
                None => break,
            };

            let current = current_entry.state;
            let current_key = current.as_key();
            let g = current_entry.g_score;

            if self.closed.contains(&current_key) {
                continue;
            }
            self.closed.insert(current_key);
            current_node = Some(current);

            // Check if reached target
            if self.target_set.contains(&current_key) {
                self.found = true;
                self.final_path = Some(self.reconstruct_path(current_key));
                break;
            }

            // Expand neighbors - 8 directions
            for (dx, dy) in DIRECTIONS {
                let ngx = current.gx + dx;
                let ngy = current.gy + dy;

                if obstacles.is_blocked(ngx, ngy, current.layer as usize) {
                    continue;
                }

                let neighbor = GridState::new(ngx, ngy, current.layer);
                let neighbor_key = neighbor.as_key();

                if self.closed.contains(&neighbor_key) {
                    continue;
                }

                let move_cost = if dx != 0 && dy != 0 { DIAG_COST } else { ORTHO_COST };
                let proximity_cost = obstacles.get_stub_proximity_cost(ngx, ngy);
                let new_g = g + move_cost + proximity_cost;

                let existing_g = self.g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                if new_g < existing_g {
                    self.g_costs.insert(neighbor_key, new_g);
                    self.parents.insert(neighbor_key, current_key);
                    let h = self.heuristic_to_targets(&neighbor);
                    let f = new_g + h;
                    self.open_set.push(OpenEntry {
                        f_score: f,
                        g_score: new_g,
                        state: neighbor,
                        counter: self.counter,
                    });
                    self.counter += 1;
                }
            }

            // Try via to other layers
            if !obstacles.is_via_blocked(current.gx, current.gy) {
                for layer in 0..obstacles.num_layers as u8 {
                    if layer == current.layer {
                        continue;
                    }

                    let neighbor = GridState::new(current.gx, current.gy, layer);
                    let neighbor_key = neighbor.as_key();

                    if self.closed.contains(&neighbor_key) {
                        continue;
                    }

                    let proximity_cost = obstacles.get_stub_proximity_cost(current.gx, current.gy) * 2;
                    let new_g = g + self.via_cost + proximity_cost;

                    let existing_g = self.g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                    if new_g < existing_g {
                        self.g_costs.insert(neighbor_key, new_g);
                        self.parents.insert(neighbor_key, current_key);
                        let h = self.heuristic_to_targets(&neighbor);
                        let f = new_g + h;
                        self.open_set.push(OpenEntry {
                            f_score: f,
                            g_score: new_g,
                            state: neighbor,
                            counter: self.counter,
                        });
                        self.counter += 1;
                    }
                }
            }
        }

        // Build snapshot
        self.create_snapshot(current_node)
    }

    /// Check if search is complete (found path or exhausted)
    fn is_done(&self) -> bool {
        self.found || self.open_set.is_empty() || self.iterations >= self.max_iterations
    }

    /// Get the final path if found
    fn get_path(&self) -> Option<Vec<(i32, i32, u8)>> {
        self.final_path.clone()
    }

    /// Get iteration count
    fn get_iterations(&self) -> u32 {
        self.iterations
    }
}

impl VisualRouter {
    fn heuristic_to_targets(&self, state: &GridState) -> i32 {
        let mut min_h = i32::MAX;
        for target in &self.target_states {
            let dx = (state.gx - target.gx).abs();
            let dy = (state.gy - target.gy).abs();
            let diag = dx.min(dy);
            let orth = (dx - dy).abs();
            let mut h = diag * DIAG_COST + orth * ORTHO_COST;
            if state.layer != target.layer {
                h += self.via_cost;
            }
            min_h = min_h.min(h);
        }
        (min_h as f32 * self.h_weight) as i32
    }

    fn reconstruct_path(&self, goal_key: u64) -> Vec<(i32, i32, u8)> {
        let mut path = Vec::new();
        let mut current_key = goal_key;

        loop {
            let layer = (current_key & 0xFF) as u8;
            let y = ((current_key >> 8) & 0xFFFFF) as i32;
            let x = ((current_key >> 28) & 0xFFFFF) as i32;
            let x = if x & 0x80000 != 0 { x | !0xFFFFF_i32 } else { x };
            let y = if y & 0x80000 != 0 { y | !0xFFFFF_i32 } else { y };

            path.push((x, y, layer));

            match self.parents.get(&current_key) {
                Some(&parent_key) => current_key = parent_key,
                None => break,
            }
        }

        path.reverse();
        path
    }

    fn create_snapshot(&self, current: Option<GridState>) -> SearchSnapshot {
        // Sample closed and open sets (limit size for performance)
        const MAX_CELLS: usize = 50000;

        let closed_cells: Vec<(i32, i32, u8)> = self.closed
            .iter()
            .take(MAX_CELLS)
            .map(|&key| {
                let layer = (key & 0xFF) as u8;
                let y = ((key >> 8) & 0xFFFFF) as i32;
                let x = ((key >> 28) & 0xFFFFF) as i32;
                let x = if x & 0x80000 != 0 { x | !0xFFFFF_i32 } else { x };
                let y = if y & 0x80000 != 0 { y | !0xFFFFF_i32 } else { y };
                (x, y, layer)
            })
            .collect();

        // For open set, we need to peek at the heap without consuming
        // Since BinaryHeap doesn't allow efficient iteration, collect g_costs keys that aren't closed
        let open_cells: Vec<(i32, i32, u8)> = self.g_costs
            .keys()
            .filter(|k| !self.closed.contains(k))
            .take(MAX_CELLS)
            .map(|&key| {
                let layer = (key & 0xFF) as u8;
                let y = ((key >> 8) & 0xFFFFF) as i32;
                let x = ((key >> 28) & 0xFFFFF) as i32;
                let x = if x & 0x80000 != 0 { x | !0xFFFFF_i32 } else { x };
                let y = if y & 0x80000 != 0 { y | !0xFFFFF_i32 } else { y };
                (x, y, layer)
            })
            .collect();

        SearchSnapshot {
            iteration: self.iterations,
            current: current.map(|s| (s.gx, s.gy, s.layer)),
            open_count: self.open_set.len(),
            closed_count: self.closed.len(),
            found: self.found,
            path: self.final_path.clone(),
            closed_cells,
            open_cells,
        }
    }
}

/// Module version
const VERSION: &str = "0.3.0";

/// Python module
#[pymodule]
fn grid_router(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", VERSION)?;
    m.add_class::<GridObstacleMap>()?;
    m.add_class::<GridRouter>()?;
    m.add_class::<VisualRouter>()?;
    m.add_class::<SearchSnapshot>()?;
    Ok(())
}
