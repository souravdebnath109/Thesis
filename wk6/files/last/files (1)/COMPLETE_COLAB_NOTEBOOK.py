# 🚗 Dhaka Traffic: PSO-Optimized ACO Path Planning with Blockages
# Complete Google Colab Notebook

"""
This notebook implements dynamic path planning for Dhaka city traffic using:
1. Particle Swarm Optimization (PSO) to optimize ACO parameters
2. Ant Colony Optimization (ACO) for path finding
3. Blockage handling (congestion, construction, protests, accidents)

Based on: "Dynamic Path Planning Based on Improved Ant Colony Algorithm 
in Traffic Congestion" (Wu et al., 2020)
"""

# ============================================================================
# STEP 1: Setup and Import Libraries
# ============================================================================

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime
import json

# Mount Google Drive
from google.colab import drive, files
drive.mount('/content/drive')

# Configuration
DATA_DIR = '/content/drive/MyDrive/'  # Change this if your files are in a subfolder

print("✓ Setup complete!")

# ============================================================================
# STEP 2: Verify Datasets
# ============================================================================

print("\n📁 Checking for required CSV files...")

required_files = [
    'intersections.csv',
    'roads.csv',
    'time_patterns.csv',
    'traffic.csv',
    'congestion_data.csv',
    'blockage_events.csv',
    'blockage_impact.csv'
]

all_present = True
for file in required_files:
    file_path = os.path.join(DATA_DIR, file)
    if os.path.exists(file_path):
        print(f"✓ {file}")
    else:
        print(f"✗ {file} - NOT FOUND")
        all_present = False

if not all_present:
    print("\n⚠️  Some files are missing. Please upload all required CSV files.")
else:
    print("\n✅ All required files found!")

# ============================================================================
# STEP 3: Load Datasets
# ============================================================================

def load_all_datasets():
    """Load all datasets"""
    
    print("\n📊 Loading datasets...")
    
    datasets = {}
    datasets['intersections'] = pd.read_csv(os.path.join(DATA_DIR, 'intersections.csv'))
    datasets['roads'] = pd.read_csv(os.path.join(DATA_DIR, 'roads.csv'))
    datasets['time_patterns'] = pd.read_csv(os.path.join(DATA_DIR, 'time_patterns.csv'))
    datasets['traffic'] = pd.read_csv(os.path.join(DATA_DIR, 'traffic.csv'))
    datasets['congestion'] = pd.read_csv(os.path.join(DATA_DIR, 'congestion_data.csv'))
    datasets['blockage_events'] = pd.read_csv(os.path.join(DATA_DIR, 'blockage_events.csv'))
    datasets['blockage_impact'] = pd.read_csv(os.path.join(DATA_DIR, 'blockage_impact.csv'))
    
    print("✓ All datasets loaded!")
    print(f"\nDataset sizes:")
    for name, df in datasets.items():
        print(f"  {name}: {len(df)} records")
    
    return datasets

datasets = load_all_datasets()

# ============================================================================
# STEP 4: Explore Blockage Data
# ============================================================================

print("\n" + "="*70)
print("BLOCKAGE EVENTS SUMMARY")
print("="*70)

print("\n📊 Blockage Types:")
print(datasets['blockage_events']['blockage_type'].value_counts())

print("\n📊 Average Severity by Type:")
print(datasets['blockage_events'].groupby('blockage_type')['severity'].mean().round(2))

print("\n📋 Sample Blockage Events:")
print(datasets['blockage_events'][['road_id', 'blockage_type', 'severity', 'description']].head(10))

# Visualize blockages
fig, axes = plt.subplots(1, 2, figsize=(14, 5))

# Blockage types
datasets['blockage_events']['blockage_type'].value_counts().plot(
    kind='bar', ax=axes[0], color='coral'
)
axes[0].set_title('Blockage Events by Type', fontweight='bold')
axes[0].set_xlabel('Blockage Type')
axes[0].set_ylabel('Count')
axes[0].tick_params(axis='x', rotation=45)

# Severity by type
datasets['blockage_events'].groupby('blockage_type')['severity'].mean().plot(
    kind='bar', ax=axes[1], color='red'
)
axes[1].set_title('Average Severity by Blockage Type', fontweight='bold')
axes[1].set_xlabel('Blockage Type')
axes[1].set_ylabel('Average Severity')
axes[1].tick_params(axis='x', rotation=45)

plt.tight_layout()
plt.savefig(os.path.join(DATA_DIR, 'blockage_analysis.png'), dpi=300, bbox_inches='tight')
plt.show()

print("\n✓ Blockage analysis visualization saved!")

# ============================================================================
# STEP 5: PSO Optimizer Class
# ============================================================================

class PSOOptimizer:
    """Particle Swarm Optimization for ACO parameter tuning"""
    
    def __init__(self, n_particles=20, n_iterations=30):
        self.n_particles = n_particles
        self.n_iterations = n_iterations
        self.bounds = {
            'alpha': (0.5, 1.5),
            'beta': (1.0, 5.0),
            'rho': (0.5, 1.0)
        }
        
    def optimize(self, objective_function):
        """Optimize ACO parameters"""
        particles = []
        velocities = []
        personal_best = []
        personal_best_scores = []
        
        for _ in range(self.n_particles):
            particle = {
                'alpha': np.random.uniform(*self.bounds['alpha']),
                'beta': np.random.uniform(*self.bounds['beta']),
                'rho': np.random.uniform(*self.bounds['rho'])
            }
            particles.append(particle)
            
            velocity = {
                'alpha': np.random.uniform(-0.1, 0.1),
                'beta': np.random.uniform(-0.1, 0.1),
                'rho': np.random.uniform(-0.1, 0.1)
            }
            velocities.append(velocity)
            
            personal_best.append(particle.copy())
            personal_best_scores.append(float('inf'))
        
        global_best = None
        global_best_score = float('inf')
        
        w = 0.7
        c1 = 1.5
        c2 = 1.5
        
        print("\n🔄 Running PSO optimization...")
        
        for iteration in range(self.n_iterations):
            for i in range(self.n_particles):
                score = objective_function(
                    particles[i]['alpha'],
                    particles[i]['beta'],
                    particles[i]['rho']
                )
                
                if score < personal_best_scores[i]:
                    personal_best_scores[i] = score
                    personal_best[i] = particles[i].copy()
                
                if score < global_best_score:
                    global_best_score = score
                    global_best = particles[i].copy()
            
            for i in range(self.n_particles):
                for param in ['alpha', 'beta', 'rho']:
                    r1, r2 = np.random.random(), np.random.random()
                    
                    velocities[i][param] = (
                        w * velocities[i][param] +
                        c1 * r1 * (personal_best[i][param] - particles[i][param]) +
                        c2 * r2 * (global_best[param] - particles[i][param])
                    )
                    
                    particles[i][param] += velocities[i][param]
                    particles[i][param] = np.clip(
                        particles[i][param],
                        self.bounds[param][0],
                        self.bounds[param][1]
                    )
            
            if (iteration + 1) % 10 == 0:
                print(f"   Iteration {iteration + 1}/{self.n_iterations} - Best: {global_best_score:.4f}")
        
        print(f"\n✓ PSO complete! α={global_best['alpha']:.2f}, β={global_best['beta']:.2f}, ρ={global_best['rho']:.2f}")
        
        return global_best

print("✓ PSO Optimizer defined!")

# ============================================================================
# STEP 6: ACO Algorithm Class
# ============================================================================

class AntColonyOptimizer:
    """Ant Colony Optimization for path planning"""
    
    def __init__(self, graph, alpha=1.0, beta=2.5, rho=0.8, n_ants=10, n_iterations=100, Q=100):
        self.graph = graph
        self.alpha = alpha
        self.beta = beta
        self.rho = rho
        self.n_ants = n_ants
        self.n_iterations = n_iterations
        self.Q = Q
        
        self.pheromones = {}
        for edge in graph['edges']:
            self.pheromones[edge] = 1.0
    
    def find_path(self, start, end):
        """Find optimal path"""
        best_path = None
        best_cost = float('inf')
        convergence = []
        
        for iteration in range(self.n_iterations):
            iteration_best_path = None
            iteration_best_cost = float('inf')
            
            for ant in range(self.n_ants):
                path, cost = self._ant_find_path(start, end)
                if path and cost < iteration_best_cost:
                    iteration_best_cost = cost
                    iteration_best_path = path
            
            if iteration_best_path and iteration_best_cost < best_cost:
                best_cost = iteration_best_cost
                best_path = iteration_best_path
            
            self._update_pheromones(iteration_best_path, iteration_best_cost)
            convergence.append(best_cost)
        
        return best_path, best_cost, convergence
    
    def _ant_find_path(self, start, end):
        """Single ant pathfinding"""
        current = start
        path = [current]
        total_cost = 0
        visited = {current}
        
        while current != end:
            neighbors = self.graph['neighbors'].get(current, [])
            unvisited = [n for n in neighbors if n not in visited]
            
            if not unvisited:
                return None, float('inf')
            
            probabilities = []
            for neighbor in unvisited:
                edge = (current, neighbor)
                if edge not in self.pheromones:
                    edge = (neighbor, current)
                
                pheromone = self.pheromones.get(edge, 1.0)
                cost = self.graph['costs'].get(edge, 1.0)
                heuristic = 1.0 / (cost + 0.001)
                
                prob = (pheromone ** self.alpha) * (heuristic ** self.beta)
                probabilities.append(prob)
            
            prob_sum = sum(probabilities)
            if prob_sum == 0:
                probabilities = [1.0] * len(probabilities)
                prob_sum = len(probabilities)
            
            probabilities = [p / prob_sum for p in probabilities]
            next_node = np.random.choice(unvisited, p=probabilities)
            
            edge = (current, next_node)
            if edge not in self.graph['costs']:
                edge = (next_node, current)
            
            total_cost += self.graph['costs'].get(edge, 0)
            path.append(next_node)
            visited.add(next_node)
            current = next_node
        
        return path, total_cost
    
    def _update_pheromones(self, best_path, best_cost):
        """Update pheromone trails"""
        for edge in self.pheromones:
            self.pheromones[edge] *= (1 - self.rho)
        
        if best_path and len(best_path) > 1:
            deposit = self.Q / (best_cost + 0.001)
            
            for i in range(len(best_path) - 1):
                edge = (best_path[i], best_path[i + 1])
                if edge not in self.pheromones:
                    edge = (best_path[i + 1], best_path[i])
                
                if edge in self.pheromones:
                    self.pheromones[edge] += deposit

print("✓ ACO Algorithm defined!")

# ============================================================================
# STEP 7: Helper Functions
# ============================================================================

def calculate_road_cost_with_blockage(road_id, time_interval, datasets):
    """Calculate road cost including blockages"""
    road_info = datasets['roads'][datasets['roads']['road_id'] == road_id].iloc[0]
    distance = road_info['distance_km']
    
    congestion_data = datasets['congestion'][
        (datasets['congestion']['road_id'] == road_id) &
        (datasets['congestion']['time_interval'] == time_interval)
    ]
    congestion_coef = congestion_data['congestion_coefficient'].mean() if len(congestion_data) > 0 else 0
    
    blockage_data = datasets['blockage_impact'][
        (datasets['blockage_impact']['road_id'] == road_id) &
        (datasets['blockage_impact']['time_interval'] == time_interval)
    ]
    blockage_severity = blockage_data['blockage_severity'].values[0] if len(blockage_data) > 0 else 0
    
    cost = distance * (1 + congestion_coef + blockage_severity)
    return cost, congestion_coef, blockage_severity

def build_graph(datasets, time_interval, include_blockages=True):
    """Build graph for ACO"""
    graph = {'edges': [], 'costs': {}, 'neighbors': {}}
    
    for _, road in datasets['roads'].iterrows():
        from_node = road['from_intersection']
        to_node = road['to_intersection']
        road_id = road['road_id']
        
        if include_blockages:
            cost, _, _ = calculate_road_cost_with_blockage(road_id, time_interval, datasets)
        else:
            cost = road['distance_km']
        
        edge1 = (from_node, to_node)
        edge2 = (to_node, from_node)
        
        graph['edges'].extend([edge1, edge2])
        graph['costs'][edge1] = cost
        graph['costs'][edge2] = cost
        
        if from_node not in graph['neighbors']:
            graph['neighbors'][from_node] = []
        if to_node not in graph['neighbors']:
            graph['neighbors'][to_node] = []
        
        graph['neighbors'][from_node].append(to_node)
        graph['neighbors'][to_node].append(from_node)
    
    return graph

print("✓ Helper functions defined!")

# ============================================================================
# STEP 8: Run PSO-ACO Path Planning
# ============================================================================

print("\n" + "="*70)
print("RUNNING PSO-OPTIMIZED ACO PATH PLANNING")
print("="*70)

# Configuration
START_NODE = 0      # Shahbag
END_NODE = 3        # Mohakhali
TIME_INTERVAL = 14  # 7:00 AM (morning rush)

# Get time info
time_info = datasets['time_patterns'][datasets['time_patterns']['time_interval'] == TIME_INTERVAL].iloc[0]
print(f"\n⏰ Planning for: {time_info['time']} ({time_info['congestion_level']})")

# Check blockages
active_blockages = datasets['blockage_impact'][
    (datasets['blockage_impact']['time_interval'] == TIME_INTERVAL) &
    (datasets['blockage_impact']['has_blockage'] == 1)
]
print(f"🚧 Active blockages: {len(active_blockages)}")

# Build graphs
print("\n📊 Building graphs...")
graph_with_blockage = build_graph(datasets, TIME_INTERVAL, include_blockages=True)
graph_no_blockage = build_graph(datasets, TIME_INTERVAL, include_blockages=False)

# PSO optimization
print("\n" + "-"*70)
print("STEP 1: PSO Parameter Optimization")
print("-"*70)

def objective_function(alpha, beta, rho):
    aco = AntColonyOptimizer(graph_with_blockage, alpha=alpha, beta=beta, rho=rho, 
                             n_ants=5, n_iterations=20)
    _, cost, _ = aco.find_path(START_NODE, END_NODE)
    return cost

pso = PSOOptimizer(n_particles=15, n_iterations=20)
best_params = pso.optimize(objective_function)

# ACO with blockages
print("\n" + "-"*70)
print("STEP 2: ACO Path Planning WITH Blockages")
print("-"*70)

aco_with = AntColonyOptimizer(graph_with_blockage, alpha=best_params['alpha'], 
                               beta=best_params['beta'], rho=best_params['rho'],
                               n_ants=10, n_iterations=100)
path_with, cost_with, conv_with = aco_with.find_path(START_NODE, END_NODE)

print(f"\n✓ Path WITH blockages: {' → '.join(map(str, path_with))}")
print(f"   Cost: {cost_with:.4f}")

# ACO without blockages
print("\n" + "-"*70)
print("STEP 3: ACO Path Planning WITHOUT Blockages")
print("-"*70)

aco_without = AntColonyOptimizer(graph_no_blockage, alpha=best_params['alpha'],
                                  beta=best_params['beta'], rho=best_params['rho'],
                                  n_ants=10, n_iterations=100)
path_without, cost_without, conv_without = aco_without.find_path(START_NODE, END_NODE)

print(f"\n✓ Path WITHOUT blockages: {' → '.join(map(str, path_without))}")
print(f"   Cost: {cost_without:.4f}")

# Analysis
print("\n" + "="*70)
print("RESULTS")
print("="*70)
cost_increase = ((cost_with - cost_without) / cost_without) * 100
print(f"\n📊 Impact: Cost increased by {cost_increase:.2f}% due to blockages")
print(f"   Same path? {'Yes' if path_with == path_without else 'No - route changed!'}")

# ============================================================================
# STEP 9: Visualization
# ============================================================================

fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# Convergence
axes[0, 0].plot(conv_with, label='With Blockages', color='red', linewidth=2)
axes[0, 0].plot(conv_without, label='Without Blockages', color='blue', linewidth=2)
axes[0, 0].set_xlabel('Iteration')
axes[0, 0].set_ylabel('Best Cost')
axes[0, 0].set_title('ACO Convergence')
axes[0, 0].legend()
axes[0, 0].grid(True, alpha=0.3)

# Cost comparison
axes[0, 1].bar(['Without\nBlockages', 'With\nBlockages'], [cost_without, cost_with], 
               color=['blue', 'red'])
axes[0, 1].set_ylabel('Total Cost')
axes[0, 1].set_title('Path Cost Comparison')
axes[0, 1].grid(True, alpha=0.3, axis='y')

# Parameters
axes[1, 0].bar(['α', 'β', 'ρ'], 
               [best_params['alpha'], best_params['beta'], best_params['rho']],
               color=['green', 'orange', 'purple'])
axes[1, 0].set_ylabel('Value')
axes[1, 0].set_title('Optimized Parameters (PSO)')
axes[1, 0].grid(True, alpha=0.3, axis='y')

# Blockages
if len(active_blockages) > 0:
    road_names = []
    severities = []
    for _, b in active_blockages.iterrows():
        road_name = datasets['roads'][datasets['roads']['road_id'] == b['road_id']]['road_name'].values[0]
        road_names.append(road_name)
        severities.append(b['blockage_severity'])
    axes[1, 1].barh(road_names, severities, color='coral')
    axes[1, 1].set_xlabel('Severity')
    axes[1, 1].set_title('Active Blockages')
else:
    axes[1, 1].text(0.5, 0.5, 'No Active Blockages', ha='center', va='center', fontsize=14)
    axes[1, 1].set_title('Active Blockages')

plt.suptitle('PSO-ACO Path Planning Results - Dhaka', fontsize=16, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(DATA_DIR, 'pso_aco_results.png'), dpi=300, bbox_inches='tight')
plt.show()

print("\n✓ Visualization saved!")

# ============================================================================
# STEP 10: Save and Download Results
# ============================================================================

# Save JSON results
results_json = {
    'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    'time_interval': int(TIME_INTERVAL),
    'pso_parameters': {k: float(v) for k, v in best_params.items()},
    'path_with_blockage': [int(x) for x in path_with],
    'cost_with_blockage': float(cost_with),
    'path_without_blockage': [int(x) for x in path_without],
    'cost_without_blockage': float(cost_without),
    'cost_increase_percent': float(cost_increase)
}

with open(os.path.join(DATA_DIR, 'results.json'), 'w') as f:
    json.dump(results_json, f, indent=2)

print("\n✓ Results saved to Google Drive!")

# Download
print("\n📥 Downloading results...")
try:
    files.download(os.path.join(DATA_DIR, 'pso_aco_results.png'))
    files.download(os.path.join(DATA_DIR, 'results.json'))
    files.download(os.path.join(DATA_DIR, 'blockage_analysis.png'))
    print("✓ Files downloaded!")
except:
    print("Files saved to Google Drive (download may be blocked)")

print("\n" + "="*70)
print("✅ COMPLETE! All results saved to Google Drive.")
print("="*70)
