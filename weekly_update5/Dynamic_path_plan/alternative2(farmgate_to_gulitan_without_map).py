import math
import random
import numpy as np
import networkx as nx
import osmnx as ox
import folium

# Set random seeds for reproducibility
random.seed(42)
np.random.seed(42)

print("Starting ACO+PSO Traffic Routing System...")
print("=" * 60)

# ============================================================================
# CONFIGURATION
# ============================================================================

# Real coordinates: Farmgate → Gulistan (Dhaka, Bangladesh)
START_LATLON = (23.759954868041955, 90.38945528575461)  # Farmgate
GOAL_LATLON = (23.722931887487082, 90.41335657238497)   # Gulistan

GRAPH_RADIUS_M = 5000   # Radius for downloading road network

print(f"Start: Farmgate {START_LATLON}")
print(f"Goal: Gulistan {GOAL_LATLON}")
print(f"Graph radius: {GRAPH_RADIUS_M}m")
print("=" * 60)

# ============================================================================
# STEP 1: Build Road Network from OpenStreetMap
# ============================================================================

def build_osm_graph(start_latlon, goal_latlon, radius_m=5000):
    """Download road network from OpenStreetMap"""
    mid_lat = (start_latlon[0] + goal_latlon[0]) / 2.0
    mid_lon = (start_latlon[1] + goal_latlon[1]) / 2.0
    
    print(f"\nDownloading road network from OSM...")
    print(f"Center point: ({mid_lat:.6f}, {mid_lon:.6f})")
    
    # Download graph with simplification enabled
    G = ox.graph_from_point(
        (mid_lat, mid_lon),
        dist=radius_m,
        network_type="drive",
        simplify=True
    )
    
    # Add edge lengths in meters
    G = ox.add_edge_lengths(G)
    
    print(f"✓ Graph downloaded: {len(G.nodes)} nodes, {len(G.edges)} edges")
    return G

# Build the graph
G = build_osm_graph(START_LATLON, GOAL_LATLON, GRAPH_RADIUS_M)

# Find nearest nodes to start and goal coordinates
start_node = ox.distance.nearest_nodes(G, X=START_LATLON[1], Y=START_LATLON[0])
goal_node = ox.distance.nearest_nodes(G, X=GOAL_LATLON[1], Y=GOAL_LATLON[0])

print(f"Start node ID: {start_node}")
print(f"Goal node ID: {goal_node}")
print("=" * 60)

# ============================================================================
# STEP 2: Dynamic Traffic Simulation
# ============================================================================

# Road type base congestion factors
ROADTYPE_BASE = {
    "motorway": 0.3,
    "trunk": 0.35,
    "primary": 0.45,
    "secondary": 0.6,
    "tertiary": 0.75,
    "residential": 0.9,
    "service": 1.0
}

def highway_type(u, v, k, data):
    """Extract highway type from edge data"""
    hw = data.get("highway", "residential")
    if isinstance(hw, list):
        hw = hw[0]
    return hw

def get_lanes(data):
    """Extract number of lanes from edge data"""
    lanes = data.get("lanes", 1)
    try:
        if isinstance(lanes, list):
            lanes = lanes[0]
        lanes = int(float(lanes))
    except:
        lanes = 1
    return max(1, lanes)

def assign_dynamic_costs(G, t, severity=0.8):
    """
    Assign dynamic traffic costs to edges based on time t
    Cost = length * (1 + congestion_factor)
    """
    for u, v, k, data in G.edges(keys=True, data=True):
        length = float(data.get("length", 10.0))
        hw = highway_type(u, v, k, data)
        lanes = get_lanes(data)
        
        base = ROADTYPE_BASE.get(hw, 0.9)
        
        # Dynamic congestion: sinusoidal wave + random noise
        wave = 0.5 * (1 + math.sin(2 * math.pi * (t / 6.0)))  # 0..1
        noise = random.random()  # 0..1
        
        # Congestion coefficient (more lanes = less congestion)
        C = severity * (0.6 * wave + 0.4 * noise) / lanes
        
        # Final cost incorporating congestion
        cost = length * (1.0 + C)
        
        data["C"] = C
        data["cost"] = cost

# Initialize costs at t=0
assign_dynamic_costs(G, t=0)
print("✓ Initial traffic costs assigned")

# ============================================================================
# STEP 3: Ant Colony Optimization (ACO) for Path Finding
# ============================================================================

def roulette_choice(items):
    """Weighted random selection"""
    total = sum(w for _, w in items)
    if total <= 0:
        return random.choice([x for x, _ in items])
    r = random.random() * total
    cum = 0.0
    for x, w in items:
        cum += w
        if r <= cum:
            return x
    return items[-1][0]

def path_cost(G, path):
    """Calculate total cost of a path"""
    if path is None or len(path) < 2:
        return float("inf")
    
    total_cost = 0.0
    for a, b in zip(path[:-1], path[1:]):
        # Choose minimum cost edge if multigraph
        edata = min(G.get_edge_data(a, b).values(), key=lambda d: d["cost"])
        total_cost += edata["cost"]
    return total_cost

def aco_shortest_path(G, start, goal, alpha, beta, rho,
                      num_ants=30, iters=80, Q=1.0, seed=42):
    """
    Ant Colony Optimization for shortest path
    
    Parameters:
    - alpha: pheromone importance
    - beta: heuristic importance
    - rho: evaporation rate
    """
    random.seed(seed)
    
    # Initialize pheromones on all edges
    tau = {}
    for u, v, k in G.edges(keys=True):
        tau[(u, v, k)] = 1.0
    
    best_path = None
    best_cost = float("inf")
    
    for iteration in range(iters):
        ant_paths = []
        
        # Deploy ants
        for ant in range(num_ants):
            current = start
            visited = set([current])
            path = [current]
            
            # Ant constructs path
            for step in range(8000):
                if current == goal:
                    break
                
                # Find valid neighbors
                candidates = []
                for nxt in G.successors(current):
                    if nxt in visited:
                        continue
                    
                    # Pick cheapest edge among multiple keys
                    edges = G.get_edge_data(current, nxt)
                    best_k, best_d = min(edges.items(), key=lambda kv: kv[1]["cost"])
                    cost = best_d["cost"]
                    eta = 1.0 / (cost + 1e-9)  # Heuristic (inverse cost)
                    
                    # Desirability = pheromone^alpha * heuristic^beta
                    desir = (tau[(current, nxt, best_k)] ** alpha) * (eta ** beta)
                    candidates.append((nxt, desir))
                
                if not candidates:
                    break
                
                # Choose next node probabilistically
                nxt = roulette_choice(candidates)
                path.append(nxt)
                visited.add(nxt)
                current = nxt
            
            # Record successful paths
            if path[-1] == goal:
                c = path_cost(G, path)
                ant_paths.append((path, c))
                if c < best_cost:
                    best_cost = c
                    best_path = path
        
        # Pheromone evaporation
        for key in tau.keys():
            tau[key] *= (1.0 - rho)
        
        # Pheromone deposit
        for pth, c in ant_paths:
            delta = Q / (c + 1e-9)
            for a, b in zip(pth[:-1], pth[1:]):
                edges = G.get_edge_data(a, b)
                best_k = min(edges.items(), key=lambda kv: kv[1]["cost"])[0]
                tau[(a, b, best_k)] += delta
    
    return best_path, best_cost

# ============================================================================
# STEP 4: Particle Swarm Optimization (PSO) for Parameter Tuning
# ============================================================================

def clamp(x, lo, hi):
    """Clamp value between bounds"""
    return max(lo, min(hi, x))

def round_step(x, step=0.1):
    """Round to nearest step"""
    return math.floor(x / step + 0.5) * step

def pso_tune_params(G, start, goal,
                    swarm=15, pso_iters=20,
                    alpha_bounds=(0.5, 2.0),
                    beta_bounds=(1.0, 6.0),
                    rho_bounds=(0.1, 0.9),
                    step=0.1,
                    eval_ants=20, eval_iters=50, eval_runs=2,
                    seed=42, verbose=True):
    """
    Use PSO to tune ACO parameters (alpha, beta, rho)
    """
    random.seed(seed)
    
    cache = {}
    
    def fitness(a, b, r):
        """Evaluate ACO with given parameters"""
        key = (a, b, r)
        if key in cache:
            return cache[key]
        
        vals = []
        for run in range(eval_runs):
            p, c = aco_shortest_path(
                G, start, goal,
                alpha=a, beta=b, rho=r,
                num_ants=eval_ants, iters=eval_iters,
                seed=1000 + run
            )
            if p is None:
                c = 1e18
            vals.append(c)
        
        f = float(np.mean(vals))
        cache[key] = f
        return f
    
    # Initialize swarm
    particles = []
    gbest = None
    gbest_val = float("inf")
    
    if verbose:
        print("\nInitializing PSO swarm...")
    
    for i in range(swarm):
        a = round_step(random.uniform(*alpha_bounds), step)
        b = round_step(random.uniform(*beta_bounds), step)
        r = round_step(random.uniform(*rho_bounds), step)
        va = random.uniform(-0.3, 0.3)
        vb = random.uniform(-0.3, 0.3)
        vr = random.uniform(-0.3, 0.3)
        
        val = fitness(a, b, r)
        p = {
            "pos": [a, b, r],
            "vel": [va, vb, vr],
            "pbest": [a, b, r],
            "pbest_val": val
        }
        particles.append(p)
        
        if val < gbest_val:
            gbest_val = val
            gbest = [a, b, r]
        
        if verbose:
            print(f"  Particle {i+1}/{swarm}: cost={val:.1f}, gbest={gbest_val:.1f}")
    
    # PSO parameters
    w = 0.7   # Inertia weight
    c1 = 1.5  # Cognitive coefficient
    c2 = 1.5  # Social coefficient
    
    if verbose:
        print("\nRunning PSO optimization...")
    
    # PSO iterations
    for it in range(pso_iters):
        for p in particles:
            # Update velocity
            for d in range(3):
                r1, r2 = random.random(), random.random()
                p["vel"][d] = (w * p["vel"][d] +
                              c1 * r1 * (p["pbest"][d] - p["pos"][d]) +
                              c2 * r2 * (gbest[d] - p["pos"][d]))
            
            # Update position
            p["pos"][0] = round_step(clamp(p["pos"][0] + p["vel"][0], *alpha_bounds), step)
            p["pos"][1] = round_step(clamp(p["pos"][1] + p["vel"][1], *beta_bounds), step)
            p["pos"][2] = round_step(clamp(p["pos"][2] + p["vel"][2], *rho_bounds), step)
            
            # Evaluate new position
            val = fitness(p["pos"][0], p["pos"][1], p["pos"][2])
            
            # Update personal best
            if val < p["pbest_val"]:
                p["pbest_val"] = val
                p["pbest"] = p["pos"][:]
            
            # Update global best
            if val < gbest_val:
                gbest_val = val
                gbest = p["pos"][:]
        
        if verbose:
            a, b, r = gbest
            print(f"  Iteration {it+1}/{pso_iters}: best_cost={gbest_val:.1f} "
                  f"(α={a:.2f}, β={b:.2f}, ρ={r:.2f}) [cache: {len(cache)}]")
    
    return tuple(gbest), gbest_val

# ============================================================================
# STEP 5: Run Parameter Tuning
# ============================================================================

print("\n" + "=" * 60)
print("TUNING ACO PARAMETERS WITH PSO")
print("=" * 60)

best_params, estimated_cost = pso_tune_params(
    G, start_node, goal_node,
    swarm=15,
    pso_iters=20,
    verbose=True
)

alpha, beta, rho = best_params

print("\n" + "=" * 60)
print("TUNING COMPLETE")
print("=" * 60)
print(f"Best parameters:")
print(f"  α (pheromone): {alpha:.2f}")
print(f"  β (heuristic): {beta:.2f}")
print(f"  ρ (evaporation): {rho:.2f}")
print(f"Estimated cost: {estimated_cost:.1f}m")

# ============================================================================
# STEP 6: Run Dynamic Simulation Over Time
# ============================================================================

print("\n" + "=" * 60)
print("RUNNING DYNAMIC TRAFFIC SIMULATION")
print("=" * 60)

history = []
paths = []
num_time_steps = 5

for t in range(num_time_steps):
    print(f"\nTime step {t}:")
    
    # Update traffic conditions
    assign_dynamic_costs(G, t=t, severity=0.9)
    
    # Find optimal route with tuned parameters
    path, cost = aco_shortest_path(
        G, start_node, goal_node,
        alpha=alpha, beta=beta, rho=rho,
        num_ants=40, iters=120, seed=200 + t
    )
    
    history.append((t, cost, len(path) if path else 0))
    paths.append(path)
    
    if path:
        print(f"  ✓ Route found: {len(path)} nodes, cost={cost:.1f}m")
    else:
        print(f"  ✗ No route found")

# ============================================================================
# STEP 7: Visualize Results on Interactive Map
# ============================================================================

print("\n" + "=" * 60)
print("CREATING INTERACTIVE MAP")
print("=" * 60)

def node_latlon(G, n):
    """Get lat/lon coordinates of a node"""
    return (G.nodes[n]["y"], G.nodes[n]["x"])

# Create base map
m = folium.Map(location=START_LATLON, zoom_start=13)

# Add start and goal markers
folium.Marker(
    START_LATLON,
    popup="START: Farmgate",
    tooltip="START",
    icon=folium.Icon(color="green", icon="play")
).add_to(m)

folium.Marker(
    GOAL_LATLON,
    popup="GOAL: Gulistan",
    tooltip="GOAL",
    icon=folium.Icon(color="red", icon="stop")
).add_to(m)

# Draw the optimal route from the last time step
final_path = paths[-1]
if final_path is None:
    print("⚠ No path found in final time step")
else:
    route_latlon = [node_latlon(G, n) for n in final_path]
    
    folium.PolyLine(
        route_latlon,
        weight=6,
        color="blue",
        opacity=0.8,
        popup=f"ACO+PSO Route (cost: {history[-1][1]:.1f}m)",
        tooltip="Click for details"
    ).add_to(m)
    
    print(f"✓ Route visualized: {len(final_path)} nodes")
    print(f"  Total distance: {history[-1][1]:.1f}m")

# Save map to HTML file
output_file = "route_map.html"
m.save(output_file)

print(f"\n✓ Map saved to: {output_file}")
print("  Open this file in your web browser to view the interactive map")

# ============================================================================
# STEP 8: Summary Statistics
# ============================================================================

print("\n" + "=" * 60)
print("SIMULATION SUMMARY")
print("=" * 60)

for t, cost, nodes in history:
    print(f"Time {t}: cost={cost:.1f}m, nodes={nodes}")

print("\n✓ Simulation complete!")
print("=" * 60)