# PSO-ACO Dynamic Path Planner — Build & Usage Guide

Based on: Wu, Zhou & Xiao — "Dynamic Path Planning Based on Improved
Ant Colony Algorithm in Traffic Congestion", IEEE Access 2020

## Files in This Package

| File                   | Purpose                                              |
|------------------------|------------------------------------------------------|
| `pso_aco_planner.cpp`  | Main C++ source (PSO + ACO + dynamic simulation)     |
| `generate_dataset.py`  | Python script that generates `map_data.csv`          |
| `config.txt`           | All tunable parameters (edit this, not the C++ code) |
| `map_data.csv`         | 10-node road network (auto-generated)                |
| `nodes.csv`            | Node coordinates (for reference / plotting)          |

---

## Step 1 — Generate the Dataset

Requires Python 3 (standard library only).

```bash
python3 generate_dataset.py
```

This produces `map_data.csv` and `nodes.csv` in the current folder.
Run this once; re-run to get a new random network.

---

## Step 2 — Compile the C++ Executable

### On Linux / macOS (GCC/G++)
```bash
g++ -O2 -std=c++17 -o pso_aco_planner pso_aco_planner.cpp
```

### On Windows — MinGW (produces .exe)
```cmd
g++ -O2 -std=c++17 -o pso_aco_planner.exe pso_aco_planner.cpp
```

Download MinGW from https://winlibs.com/ — choose the Win64 GCC release.
After installation, add the `bin/` folder to your PATH, then run the
command above in a Command Prompt or PowerShell window.

### On Windows — MSVC (Visual Studio)
Open Developer Command Prompt and run:
```cmd
cl /EHsc /O2 /std:c++17 pso_aco_planner.cpp /Fe:pso_aco_planner.exe
```

---

## Step 3 — Configure Parameters

Open `config.txt` in any text editor.
Key parameters to experiment with:

| Parameter      | Default | Meaning                                      |
|----------------|---------|----------------------------------------------|
| `START_NODE`   | 0       | Source intersection                          |
| `END_NODE`     | 9       | Destination intersection                     |
| `R_MAX`        | 250.0   | Re-plan if road factor exceeds this          |
| `DELTA_R_MAX`  | 30.0    | Re-plan if road factor changes by this much  |
| `PSO_SWARM_SIZE`| 30     | More particles = better tuning, slower       |
| `PSO_ITERATIONS`| 100   | PSO optimisation iterations                  |
| `NUM_ANTS`     | 10      | Ant colony size (paper used 10)              |
| `ACO_ITERATIONS`| 100   | ACO search iterations per run                |
| `SIM_STEPS`    | 20      | How many time steps to simulate              |
| `TRAFFIC_NOISE`| 0.2     | 0 = static traffic, 1.0 = very volatile      |

---

## Step 4 — Run

Place all four files in the same directory:
```
pso_aco_planner(.exe)   map_data.csv   nodes.csv   config.txt
```

Then:
```bash
./pso_aco_planner          # Linux / macOS
pso_aco_planner.exe        # Windows
```

---

## Output Explained

```
PHASE 1 — PSO Parameter Optimisation
  Tunes α, β, ρ over pso_swarm × pso_iter evaluations.

PHASE 2 — Initial ACO Path Planning
  Finds besttour: lowest Road Condition Factor path from start → end.

PHASE 3 — Dynamic Simulation
  Each step:  traverse one edge → update traffic → check trigger
  Trigger: (ΔR > DELTA_R_MAX) OR (R > R_MAX)
  If triggered: re-run ACO from current node and compare costs.
```

A `reroute_log.txt` file is also created, logging every step in CSV format.

---

## Key Formulas (from the paper)

**Congestion Coefficient** (§IV-B, Eq. 8):
```
Ci(t) = max(0,  (fi(t-1) + fin_i(t) - fout_i(t)) × L  /  (li × di))
```

**Road Condition Factor** (Eq. 10):
```
Ri(t) = di × (1 + Ci(t))
```

**Transfer Probability** (Eq. 1):
```
p_ij = [τ_ij]^α × [1/R_ij]^β  /  Σ_s [τ_is]^α × [1/R_is]^β
```

**Pheromone Update** (Eq. 3–5):
```
τ_ij(t+n) = (1 - ρ) × τ_ij(t) + Δτ_ij
Δτ^k_ij   = Q / Lk   if ant k used edge (i,j), else 0
```

**PSO Velocity / Position** (Eq. 6–7):
```
v_i(t+1) = w × v_i(t) + c1 × r1 × (pbest_i - x_i) + c2 × r2 × (gbest - x_i)
x_i(t+1) = x_i(t) + v_i(t)
```

---

## Typical Results (10-node network)

| Metric              | Value           |
|---------------------|-----------------|
| PSO-tuned α         | ~1.14           |
| PSO-tuned β         | ~4.45           |
| PSO-tuned ρ         | ~0.65           |
| Initial path        | 0 → 4 → 8 → 9  |
| Initial cost (R)    | ~472 metres×(1+C)|
| Reroutes at ΔR≤5    | 1               |
| Congestion reduction| 9.73%–13.63%*  |

*As reported in the paper for the 38-node Beijing network.

---

## Extending to Larger Maps

Edit `map_data.csv` to include more nodes/edges following the same
column format: `from_node,to_node,length_m,lanes,f_current,f_in,f_out`

The C++ code auto-detects the number of nodes from the highest node ID.
No code changes are required.
