"""
Dataset Generator for PSO-ACO Dynamic Path Planning
Generates map_data.csv with a 10-node road network
including: edge lengths, lane counts, and initial traffic flows

Based on: "Dynamic Path Planning Based on Improved Ant Colony Algorithm
           in Traffic Congestion" (Wu et al., IEEE Access 2020)
"""

import csv
import random
import math

random.seed(42)

# ─────────────────────────────────────────────
# 1. NODE POSITIONS  (x, y) in metres × 10
# ─────────────────────────────────────────────
nodes = {
    0: (0,   0),
    1: (100, 0),
    2: (200, 0),
    3: (300, 0),
    4: (0,   100),
    5: (100, 100),
    6: (200, 100),
    7: (300, 100),
    8: (100, 200),
    9: (200, 200),
}

# ─────────────────────────────────────────────
# 2. EDGE DEFINITIONS  (undirected → stored as
#    two directed rows for easy C++ parsing)
# ─────────────────────────────────────────────
undirected_edges = [
    (0, 1), (1, 2), (2, 3),       # bottom row
    (4, 5), (5, 6), (6, 7),       # middle row
    (8, 9),                        # top row
    (0, 4), (1, 5), (2, 6), (3, 7),  # verticals
    (4, 8), (5, 8), (5, 9), (6, 9),  # diagonals/verticals to top
]

def euclidean(a, b):
    x1, y1 = nodes[a]
    x2, y2 = nodes[b]
    return round(math.sqrt((x2-x1)**2 + (y2-y1)**2), 2)

def random_lanes():
    """Urban roads: 1, 2, or 3 lanes."""
    return random.choice([1, 2, 2, 3])

def random_traffic(lanes, length):
    """
    Generate plausible initial traffic:
      f_current : vehicles currently ON the road
      f_in      : vehicles entering per time-step
      f_out     : vehicles leaving per time-step
    Kept physically reasonable so Ci(t) ∈ [0, 1].
    """
    capacity = int(lanes * length / 5)   # rough capacity estimate
    f_current = random.randint(0, max(1, capacity // 2))
    f_in      = random.randint(0, max(1, capacity // 4))
    f_out     = random.randint(0, max(1, f_in + 1))
    return f_current, f_in, f_out

# ─────────────────────────────────────────────
# 3. WRITE  nodes.csv
# ─────────────────────────────────────────────
with open("nodes.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["node_id", "x", "y"])
    for nid, (x, y) in nodes.items():
        writer.writerow([nid, x, y])

print(f"[✓] nodes.csv  — {len(nodes)} nodes written")

# ─────────────────────────────────────────────
# 4. WRITE  map_data.csv
# ─────────────────────────────────────────────
# Columns:
#   from_node, to_node, length_m, lanes,
#   f_current, f_in, f_out
# Both directions are written so C++ can treat it as directed.
# ─────────────────────────────────────────────
rows = []
edge_id = 0
for (a, b) in undirected_edges:
    d   = euclidean(a, b)
    l   = random_lanes()
    fc, fi, fo = random_traffic(l, d)

    # Forward direction
    rows.append([a, b, d, l, fc, fi, fo])
    # Reverse direction (slightly varied traffic)
    fc2 = random.randint(0, max(1, int(fc * 0.8)))
    fi2 = random.randint(0, max(1, int(fi * 0.8)))
    fo2 = random.randint(0, max(1, fi2 + 1))
    rows.append([b, a, d, l, fc2, fi2, fo2])
    edge_id += 1

with open("map_data.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["from_node", "to_node", "length_m",
                     "lanes", "f_current", "f_in", "f_out"])
    for row in rows:
        writer.writerow(row)

print(f"[✓] map_data.csv — {len(rows)} directed edges written")

# ─────────────────────────────────────────────
# 5. PRINT SUMMARY TABLE
# ─────────────────────────────────────────────
print("\n━━━ Edge Summary ━━━")
print(f"{'Edge':>12}  {'Length':>8}  {'Lanes':>5}  "
      f"{'f_cur':>6}  {'f_in':>5}  {'f_out':>6}")
for row in rows:
    a, b, d, l, fc, fi, fo = row
    print(f"  {a:>2} → {b:>2}      {d:>7}  {l:>5}  "
          f"{fc:>6}  {fi:>5}  {fo:>6}")

print("\n[✓] Dataset generation complete.")
print("    Files produced: nodes.csv, map_data.csv")
print("    Place both files alongside the compiled executable.\n")
