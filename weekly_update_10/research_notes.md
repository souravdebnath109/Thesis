# Research Notes: Route Optimization Concepts

## 1. TSP (Travelling Salesman Problem)

**Definition**: Given a set of cities and distances between them, find the shortest possible route that visits every city exactly once and returns to the starting city.

**Key Properties**:
- Single vehicle / agent
- Must visit **all** nodes exactly once
- Returns to origin (Hamiltonian cycle)
- Minimizes total tour length
- NP-hard problem

**Relation to Our Work**: TSP is **not directly applicable** because:
- We do NOT need to visit every node
- We have specific source-destination pairs, not a tour
- We care about dynamic traffic, not static distances
- Our paths are point-to-point, not cyclic

---

## 2. CVRP (Capacitated Vehicle Routing Problem)

**Definition**: Given a depot, a set of customers with demands, and a fleet of vehicles with capacity limits, find optimal routes for all vehicles such that:
- All customer demands are fulfilled
- No vehicle exceeds its capacity
- Total travel distance/cost is minimized

**Key Properties**:
- Multiple vehicles from a central depot
- Each vehicle has a load capacity constraint
- All customers must be served
- Extension of TSP with capacity and multiple vehicles
- NP-hard problem

**Relation to Our Work**: CVRP is **not our focus** because:
- We do not have a depot or capacity constraints
- We do not have customer demands to satisfy
- However, CVRP shares the concept of **multiple routes in a shared network**, which connects to combined route optimization

---

## 3. Route Network Optimization (Our Focus)

**Definition**: Given a road network with dynamic traffic conditions, compute optimal paths for **multiple source-destination pairs** while considering the overall network state.

**Key Properties**:
- Point-to-point paths (not tours)
- Dynamic traffic: road conditions R_i(t) change over time
- Multiple S-D pairs share the same network infrastructure
- Optimization uses metaheuristics (PSO-ACO hybrid in our case)
- Re-planning occurs when traffic conditions change significantly

**Our Approach**:
- PSO tunes ACO parameters (alpha, beta, rho) for optimal pathfinding
- ACO finds paths using pheromone-based exploration with Road Condition Factor R
- Dynamic monitoring triggers re-planning when:
  - ΔR_i > ΔR_max (sudden congestion change)
  - R_i(t) > R_max (absolute congestion threshold)

**Why This Differs from TSP/CVRP**:

| Aspect              | TSP           | CVRP          | Route Network Opt. |
|---------------------|---------------|---------------|---------------------|
| Objective           | Min tour length | Min fleet cost | Min path cost per S-D pair |
| Nodes to visit      | All           | All customers | Only path nodes |
| Vehicles            | 1             | Multiple      | Multiple S-D pairs |
| Capacity constraint | No            | Yes           | No |
| Dynamic traffic     | No            | Rarely        | Yes (core feature) |
| Return to origin    | Yes           | Yes (depot)   | No |
| Network sharing     | No            | Partially     | Yes (key focus) |

---

## 4. Combined Route Optimization

**What it means (as described by the teacher)**: When computing routes for multiple source-destination pairs, some edges or nodes may appear in multiple routes. Instead of treating each route independently, combined route optimization considers these **overlapping segments** and their cumulative effect on the network.

### Why It Matters

Consider two routes:
- Route A: 0 → 1 → 5 → 9
- Route B: 1 → 5 → 8

Both use edge (1→5). If both routes operate simultaneously, edge (1→5) carries traffic from **both** routes, which increases congestion C_i(t) and the road condition factor R_i(t).

### Analysis Approach in Our Implementation

1. **Edge Frequency Analysis**: After computing all S-D pair routes, count how many routes use each edge. High-frequency edges are potential bottlenecks.

2. **Node Frequency Analysis**: Count how many routes pass through each node. High-frequency nodes are critical intersections.

3. **Overlap Detection**: Identify which S-D pairs share common sub-paths (sequences of consecutive shared edges).

4. **Stabilized Route Assessment**: After all pairs are processed, examine whether the final routes collectively form a balanced network or concentrate traffic on certain corridors.

### Future Research Directions

- **Simultaneous optimization**: Instead of computing each pair independently, model the combined traffic load and optimize all routes jointly
- **Game-theoretic approach**: Model each S-D pair as a player competing for network resources
- **Iterative refinement**: Run all pairs, update traffic based on combined flows, then re-optimize
- **Load balancing**: Penalize edges that are already heavily used by other routes to encourage path diversity

---

## 5. How Our PSO-ACO Algorithm Fits

```
PSO-ACO Dynamic Path Planning (Wu et al., 2020)
│
├── PSO layer: Optimizes ACO parameters (α, β, ρ)
│   └── Uses Road Condition Factor R as fitness metric
│
├── ACO layer: Finds optimal path using pheromone + heuristic
│   ├── Pheromone update: τ_ij(t+1) = (1-ρ)·τ_ij(t) + Δτ_ij
│   ├── Heuristic: η_ij = 1 / R_ij (prefer less congested roads)
│   └── Transfer probability: p_ij = [τ_ij]^α · [η_ij]^β / Σ
│
└── Dynamic Re-planning: Monitors R_i(t) during traversal
    ├── Trigger: ΔR_i > ΔR_max OR R_i > R_max
    └── Compare: newtour cost < remaining besttour cost → reroute
```

This algorithm is applied **independently** to each source-destination pair, maintaining the exact flowchart logic. The combined route optimization analysis is performed **after** all pairs complete, examining the collective network usage patterns.

---

## 6. Iterative Combined Route Optimization — Implementation

### Concept

Instead of treating each S-D pair in isolation, the combined optimization recognizes that **multiple routes share the same network**. When Route A and Route B both use edge (5→9), the actual congestion on that edge is higher than what either route sees independently.

### Algorithm (Outer Loop)

```
For iteration = 1 to COMBINED_ITERS:
    1. Start with base traffic (original_edges)
    2. If iteration > 1:
       - For each edge used by ≥2 routes in previous iteration:
           f_current += COMBINED_WEIGHT × (freq - 1) × capacity_factor
           f_in      += COMBINED_WEIGHT × (freq - 1) × 2.0
    3. Run ALL S-D pairs using EXACT flowchart (PSO → ACO → Replan)
    4. Compute avg_cost across all successful routes
    5. If |avg_cost - prev_avg_cost| / prev_avg_cost < STABILITY_THRESH:
       → STABILIZED, stop
    6. Store results, continue to next iteration
```

### Traffic Accumulation Formula

For an edge used by `N` routes (where `N ≥ 2`):
```
penalty = COMBINED_WEIGHT × (N - 1)
Δf_current = penalty × (lanes × length / L) × 0.1
Δf_in      = penalty × 2.0
```

This increases the congestion coefficient C_i(t) via formula 8, which in turn increases the road condition factor R_i(t) via formula 10. Routes passing through heavily-shared edges will see higher R values and may choose alternative paths.

### Stabilization Criterion

The system is considered **stabilized** when:
```
|avg_cost(iter) - avg_cost(iter-1)| / avg_cost(iter-1) < STABILITY_THRESH
```

Default threshold: 5% (configurable via `STABILITY_THRESH` in config.txt).

### Output Comparison

The program produces a `combined_comparison.txt` file showing:
1. **Convergence history**: avg cost, change %, route changes per iteration
2. **Per-pair comparison**: independent vs combined costs and routes
3. **Summary**: how many pairs improved/worsened under combined optimization

### Key Insight

- If combined optimization **increases** average cost: routes now account for realistic aggregate traffic, yielding a more balanced network even if individual paths cost more.
- If combined optimization **decreases** average cost: the iterative feedback caused some routes to discover better alternatives, reducing shared-edge congestion.
- If results are **similar**: the network has sufficient capacity and independent routing already yields near-optimal results.
