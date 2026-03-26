"""
Extended Dataset Generator for Vehicle-Aware Pool-Based Rerouting Framework
Generates:
  - map_data.csv      (extended schema: road_class, width, condition, vehicle access, etc.)
  - nodes.csv
  - sd_pairs.csv
  - bus_corridors.csv  (predefined bus corridor edges)
  - disruption_events.csv (disruption scenarios for Case 1/2/3)

Based on original: "Dynamic Path Planning Based on Improved Ant Colony Algorithm
                     in Traffic Congestion" (Wu et al., IEEE Access 2020)
Extended for: Vehicle-aware, blockage-aware, pool-based alternative routing (Dhaka)
"""

import csv
import random
import math
import json

random.seed(42)

# ─────────────────────────────────────────────
# 1. NODE POSITIONS  (x, y) in metres × 10
#    Representing a simplified Dhaka road network
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
    (0, 1), (1, 2), (2, 3),          # bottom row
    (4, 5), (5, 6), (6, 7),          # middle row
    (8, 9),                           # top row
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
    """
    capacity = int(lanes * length / 5)
    f_current = random.randint(0, max(1, capacity // 2))
    f_in      = random.randint(0, max(1, capacity // 4))
    f_out     = random.randint(0, max(1, f_in + 1))
    return f_current, f_in, f_out

# ─────────────────────────────────────────────
# 3. ROAD CLASSIFICATION AND ATTRIBUTES
# ─────────────────────────────────────────────
# road_class: 1=main, 2=link, 3=sub, 4=mini
# Width ranges: main(8-12m), link(6-8m), sub(4-6m), mini(2-4m)
# Condition: 0.0=perfect ... 1.0=worst

# Assign road classes based on edge connectivity importance
# Main routes: central/high-traffic corridors
# Link routes: connecting main routes
# Sub routes: secondary connections
# Mini routes: narrow local streets

road_class_map = {
    # Bottom row - main arterial
    (0,1): 1, (1,2): 2, (2,3): 2,
    # Middle row - main arterial
    (4,5): 1, (5,6): 1, (6,7): 2,
    # Top row - link route
    (8,9): 2,
    # Verticals
    (0,4): 2, (1,5): 3, (2,6): 3, (3,7): 3,
    # Diagonals/Top connectors
    (4,8): 3, (5,8): 4, (5,9): 2, (6,9): 3,
}

def get_road_class(a, b):
    """Get road class for edge (a,b), try both directions."""
    key = (min(a,b), max(a,b))
    # Try exact match first
    if (a,b) in road_class_map:
        return road_class_map[(a,b)]
    if (b,a) in road_class_map:
        return road_class_map[(b,a)]
    if key in road_class_map:
        return road_class_map[key]
    return 3  # default: sub route

def get_width(road_class):
    """Width in metres based on road class."""
    width_ranges = {
        1: (8.0, 12.0),   # main
        2: (6.0, 8.0),    # link
        3: (4.0, 6.0),    # sub
        4: (2.0, 4.0),    # mini
    }
    lo, hi = width_ranges.get(road_class, (4.0, 6.0))
    return round(random.uniform(lo, hi), 1)

def get_road_condition():
    """Road condition factor: 0.0 (perfect) to 1.0 (worst)."""
    return round(random.uniform(0.0, 0.6), 2)

# Vehicle category access rules:
# Cat 1 (Truck):     main + link only, width >= 6.0
# Cat 2 (Bus):       main + link + designated corridors, width >= 5.0
# Cat 3 (Car/Micro): main + link + sub, width >= 3.5
# Cat 4 (Rick/Van):  all types, width >= 2.0

def get_vehicle_access(road_class, width, is_bus_corridor):
    """Return (cat1, cat2, cat3, cat4) access flags."""
    cat1 = 1 if (road_class <= 2 and width >= 6.0) else 0
    cat2 = 1 if ((road_class <= 2 or is_bus_corridor) and width >= 5.0) else 0
    cat3 = 1 if (road_class <= 3 and width >= 3.5) else 0
    cat4 = 1  # rickshaw/van/bike can use all routes (unless width < 2.0)
    if width < 2.0:
        cat4 = 0
    return cat1, cat2, cat3, cat4

# Bus corridors: predefined edges where buses are explicitly allowed
bus_corridor_edges = {(4, 5), (5, 4), (5, 6), (6, 5), (0, 1), (1, 0), (5, 9), (9, 5)}

# Disruption priors: some edges have higher chance of blockage
# (configurable, not hard-coded as facts)
disruption_prior_map = {
    (5, 9): 0.3,   # high-traffic diagonal (illustrative high-risk example)
    (9, 5): 0.3,
    (5, 6): 0.2,   # central corridor (illustrative)
    (6, 5): 0.2,
    (4, 8): 0.15,  # peripheral link
    (8, 4): 0.15,
}

# ─────────────────────────────────────────────
# 4. WRITE nodes.csv
# ─────────────────────────────────────────────
with open("nodes.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["node_id", "x", "y"])
    for nid, (x, y) in nodes.items():
        writer.writerow([nid, x, y])

print(f"[OK] nodes.csv  -- {len(nodes)} nodes written")

# ─────────────────────────────────────────────
# 5. WRITE extended map_data.csv
# ─────────────────────────────────────────────
rows = []
for (a, b) in undirected_edges:
    d = euclidean(a, b)
    l = random_lanes()
    fc, fi, fo = random_traffic(l, d)
    rc = get_road_class(a, b)
    w = get_width(rc)
    cond = get_road_condition()
    is_bc_fwd = 1 if (a, b) in bus_corridor_edges else 0
    dp_fwd = disruption_prior_map.get((a, b), round(random.uniform(0.0, 0.1), 2))
    c1, c2, c3, c4 = get_vehicle_access(rc, w, is_bc_fwd)

    # Forward direction
    rows.append([a, b, d, l, fc, fi, fo, rc, w, cond, c1, c2, c3, c4, is_bc_fwd, dp_fwd])

    # Reverse direction (slightly varied traffic, same road attributes)
    fc2 = random.randint(0, max(1, int(fc * 0.8)))
    fi2 = random.randint(0, max(1, int(fi * 0.8)))
    fo2 = random.randint(0, max(1, fi2 + 1))
    is_bc_rev = 1 if (b, a) in bus_corridor_edges else 0
    dp_rev = disruption_prior_map.get((b, a), round(random.uniform(0.0, 0.1), 2))
    c1r, c2r, c3r, c4r = get_vehicle_access(rc, w, is_bc_rev)

    rows.append([b, a, d, l, fc2, fi2, fo2, rc, w, cond, c1r, c2r, c3r, c4r, is_bc_rev, dp_rev])

with open("map_data.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "from_node", "to_node", "length_m", "lanes",
        "f_current", "f_in", "f_out",
        "road_class", "width_m", "road_condition",
        "cat1_allowed", "cat2_allowed", "cat3_allowed", "cat4_allowed",
        "bus_corridor", "disruption_prior"
    ])
    for row in rows:
        writer.writerow(row)

print(f"[OK] map_data.csv -- {len(rows)} directed edges written (extended schema)")

# ─────────────────────────────────────────────
# 6. WRITE sd_pairs.csv
# ─────────────────────────────────────────────
sd_pairs = [
    (0, 9), (0, 7), (0, 3),
    (1, 8), (1, 9),
    (2, 8), (2, 9),
    (3, 9),
    (4, 7), (4, 9),
    (5, 9),
    (6, 9),
    (7, 0), (7, 9),
    (8, 1),
    (9, 0), (9, 3),
]

with open("sd_pairs.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["source", "destination"])
    for src, dst in sd_pairs:
        writer.writerow([src, dst])

print(f"[OK] sd_pairs.csv -- {len(sd_pairs)} source-destination pairs written")

# ─────────────────────────────────────────────
# 7. WRITE bus_corridors.csv
# ─────────────────────────────────────────────
with open("bus_corridors.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["from_node", "to_node"])
    for (a, b) in sorted(bus_corridor_edges):
        writer.writerow([a, b])

print(f"[OK] bus_corridors.csv -- {len(bus_corridor_edges)} bus corridor edges written")

# ─────────────────────────────────────────────
# 8. WRITE disruption_events.csv
# ─────────────────────────────────────────────
disruption_events = [
    (1, 5, 9, "protest", "Road blocked by sudden protest near node 5-9 corridor"),
    (2, 4, 8, "broken_link", "Bridge structural failure on edge 4-8"),
    (3, 5, 6, "emergency_closure", "Emergency road closure on central corridor 5-6"),
]

with open("disruption_events.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["event_id", "blocked_from", "blocked_to", "event_type", "description"])
    for evt in disruption_events:
        writer.writerow(evt)

print(f"[OK] disruption_events.csv -- {len(disruption_events)} disruption events written")

# ─────────────────────────────────────────────
# 9. SUMMARY
# ─────────────────────────────────────────────
print("\n--- Edge Summary (Extended) ---")
hdr = f"{'Edge':>12}  {'Len':>7}  {'Ln':>3}  {'RC':>3}  {'W':>5}  {'Cond':>5}  {'C1':>3} {'C2':>3} {'C3':>3} {'C4':>3}  {'BC':>3}  {'DP':>5}"
print(hdr)
for row in rows:
    a, b, d, l, fc, fi, fo, rc, w, cond, c1, c2, c3, c4, bc, dp = row
    print(f"  {a:>2} -> {b:>2}  {d:>7}  {l:>3}  {rc:>3}  {w:>5}  {cond:>5}  {c1:>3} {c2:>3} {c3:>3} {c4:>3}  {bc:>3}  {dp:>5}")

print(f"\n[OK] Dataset generation complete.")
print(f"    Files produced: nodes.csv, map_data.csv, sd_pairs.csv, bus_corridors.csv, disruption_events.csv")
print(f"    Place all files alongside the compiled executable.\n")
