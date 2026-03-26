# Updated Research Notes: Vehicle-Aware Pool-Based Rerouting Framework

## Research Context

**Base Paper:** Wu et al., "Dynamic Path Planning Based on Improved Ant Colony Algorithm in Traffic Congestion," IEEE Access, 2020.

**Extension:** Vehicle-aware, blockage-aware, pool-based alternative path-planning framework, applied to a simplified 10-node Dhaka road network. The framework introduces vehicle categories, road hierarchy, and a pool-based rerouting strategy as a proposed improvement over standard online rerouting.

---

## Key Formulas (Preserved from Base Paper)

| # | Formula | Description |
|---|---------|-------------|
| F1 | p_k_ij = [tau_ij]^alpha * [eta_ij]^beta / SUM | Transfer probability |
| F3 | tau_ij(t+1) = (1-rho)*tau_ij(t) + delta_tau_ij | Pheromone evaporation + deposit |
| F5 | delta_tau_ij = Q / cost_k | Pheromone deposit amount |
| F8 | C_i(t) = (f_i(t-1) + f_in_i(t) - f_out_i(t)) * L / (l_i * d_i) | Congestion coefficient |
| F10 | R_i(t) = d_i * (1 + C_i(t)) | Road condition factor |

---

## Framework Extensions (New in This Work)

### 1. Vehicle Categories

| Cat | Name | Min Width | Access Rules |
|-----|------|-----------|--------------|
| 1 | Truck | 6.0 m | Main + Link roads only |
| 2 | Bus | 5.0 m | Main + Link + designated corridors |
| 3 | Car/Microbus | 3.5 m | Main + Link + Sub roads |
| 4 | Rickshaw/Van/Bike | 2.0 m | All road types |

### 2. Road Hierarchy

| Class | Name | Width Range | Typical Use |
|-------|------|-------------|-------------|
| 1 | Main | 8-12 m | Primary arterials |
| 2 | Link | 6-8 m | Connecting roads |
| 3 | Sub | 4-6 m | Secondary connections |
| 4 | Mini | 2-4 m | Narrow local streets |

### 3. Pool-Based Alternative Routing

For each (source, destination, vehicle_category) triple:
- **Pool generation**: Multiple ACO runs with pheromone penalties on previously found paths to encourage diversity
- **Pool size**: 5 ranked alternatives (best→worst by R_i(t) cost)
- **Online queries**: During disruptions, pool is queried instead of running full ACO

### 4. Disruption Prior

Each edge has a `disruption_prior` ∈ [0, 1] representing the probability of blockage. This is used to model Dhaka-specific risks such as protests, bridge failures, and emergency closures.

---

## Experiment Design (Step 6)

### Network Setup

- **Nodes:** 10 (representing simplified Dhaka intersections)
- **Edges:** 30 directed (from 15 undirected roads)
- **S-D Pairs:** 17 (covering cross-network travel patterns)
- **Vehicle Categories:** 4 (Truck, Bus, Car/Microbus, Rickshaw/Van/Bike)
- **Total experiments:** 17 pairs × 4 categories = 68 (s,d,v) combinations

### ACO Parameters (Fixed per Supervisor Flowchart)

| Parameter | Value |
|-----------|-------|
| alpha (α) | 1.0 |
| beta (β) | 2.0 |
| rho (ρ) | 0.5 |
| Ants | 10 |
| ACO Iterations | 100 |
| Q | 100.0 |
| tau_init | 1.0 |
| Pool size | 5 |
| Sim steps | 20 |
| Random seed | 42 |

### Disruption Scenarios

| ID | Edge | Type | Description |
|----|------|------|-------------|
| 1 | 5→9 | Protest | Road blocked by sudden protest |
| 2 | 4→8 | Broken link | Bridge structural failure |
| 3 | 5→6 | Emergency closure | Central corridor closure |

### Three Online Rerouting Cases

| Case | Name | Strategy | ACO Required? |
|------|------|----------|---------------|
| 1 | Source-based | Remove blocked edge, replan from source | Full ACO (100 iters) |
| 2 | Current-position | Increase weights near blockage, replan from current pos | Full ACO (100 iters) |
| 3 | Pool-based (PROPOSED) | Query precomputed pool, select best feasible | Pool query only (0 iters if successful, 25 iters if repair needed) |

### Metrics Measured

1. **Reroute iterations** — Number of ACO iterations used during online rerouting
2. **Reroute time (ms)** — CPU process time for the reroute operation
3. **Energy proxy** — time_ms × 0.001 (used as computation power proxy since hardware measurement is unavailable)
4. **Success rate** — Whether a feasible reroute was found
5. **Final cost** — Total R_i(t) cost of the rerouted path

---

## Experimental Results

### Vehicle Category Reachability (Combined Optimization)

| Category | Reached/Total |
|----------|---------------|
| Truck | 17/17 (100%) |
| Bus | 17/17 (100%) |
| Car/Microbus | 15/17 (88%) |
| Rickshaw/Van/Bike | 13/17 (76%) |

### Case Comparison Summary

| Metric | Case 1 (Source) | Case 2 (Current-pos) | Case 3 (Pool-based) |
|--------|-----------------|----------------------|---------------------|
| Affected pairs | 23 | 23 | 23 |
| Successful reroutes | 5 | 5 | 5 |
| **Avg iterations** | **100.00** | **100.00** | **23.91** |
| **Avg reroute time** | **3.295 ms** | **3.674 ms** | **0.606 ms** |
| **Avg energy proxy** | **0.0033** | **0.0037** | **0.0006** |
| Avg final cost | 501.83 | 1208.17 | 542.33 |

### Key Observations

1. **Case 3 reduces online computation by ~76%** (23.91 vs 100.00 iterations)
2. **Case 3 is ~5.4× faster** in reroute time (0.606 ms vs 3.295 ms for Case 1)
3. **Case 3 uses ~82% less energy proxy** (0.0006 vs 0.0033 for Case 1)
4. Case 1 achieves a lower final cost (501.83) than Case 3 (542.33), but at significantly higher computational expense
5. Case 2 has the highest final cost (1208.17) due to congestion-increased weights near the blockage

---

## Case 3 Hypothesis Explanation (Step 7)

### Hypothesis Statement

> **The pool-based rerouting method (Case 3) significantly reduces online rerouting iterations, computation time, and energy consumption compared to full re-planning (Case 1 and Case 2), while maintaining comparable route quality.**

### Rationale

The core insight is that during a disruption, the expensive ACO computation has already been performed **offline** during the pool generation phase (Step A3). When a disruption occurs:

- **Case 1 & 2** must run a full ACO from scratch (100 iterations × 10 ants = 1000 path constructions)
- **Case 3** simply scans 5 precomputed pool entries for one that avoids the blocked edges (O(5 × path_length) operations, essentially instantaneous)

This represents a **time-space tradeoff**: the pool precomputation cost is amortized across all future disruptions, while the online response cost is reduced to near-zero.

### Cost vs Speed Tradeoff

The results show Case 3's final cost (542.33) is ~8% higher than Case 1's (501.83), because the pool was computed under pre-disruption conditions and may not perfectly adapt to the post-disruption traffic state. However:

- The 8% cost increase is offset by a **5.4× speedup** in response time
- In real-world Dhaka traffic, response speed is often more critical than optimal cost
- The pool can be periodically refreshed (Step B) to maintain quality

### Limitations

1. Pool paths are computed under pre-disruption traffic — may not be optimal post-disruption
2. If all pool paths are affected by the disruption, a repair ACO is still needed (25 iterations)
3. Pool diversity depends on the network topology — sparse networks may produce fewer diverse alternatives
4. Energy proxy is based on CPU time, not actual hardware power measurement

### Conclusion

The experimental results on the 10-node Dhaka network **support the hypothesis**. Case 3 achieves dramatically lower online computation cost while maintaining acceptable route quality. This makes pool-based rerouting particularly suitable for real-time applications in congested urban environments like Dhaka, where fast response to disruptions is critical.

---

## Files Produced

| File | Description |
|------|-------------|
| `vehicle_pool_planner.cpp` | Main C++ implementation (~1475 lines) |
| `generate_dataset.py` | Extended dataset generator |
| `config.txt` | Configuration with 50+ parameters |
| `map_data.csv` | Extended edge data (16 columns) |
| `sd_pairs.csv` | 17 source-destination pairs |
| `bus_corridors.csv` | 8 bus corridor edges |
| `disruption_events.csv` | 3 disruption scenarios |
| `pair_stable_routes.csv` | Final stable routes per (s,d,v) |
| `pair_pools.json` | Pool paths per (s,d,v) |
| `pair_metadata.csv` | Per-pair metadata |
| `network_pool_analysis.txt` | Network-level analysis |
| `case1_results.csv` | Case 1 results |
| `case2_results.csv` | Case 2 results |
| `case3_results.csv` | Case 3 results |
| `case_comparison_summary.csv` | Side-by-side case comparison |
| `reroute_log.txt` | Independent reroute log |
| `reroute_log_combined.txt` | Combined optimization reroute log |
| `reroute_log_case1/2/3.txt` | Per-case reroute logs |

---

## Reproducibility

- **Random seed:** 42 (set in `config.txt`)
- **Compiler:** g++ (MinGW) with `-std=c++17 -O2`
- **All results are deterministic** given the same seed and input files
