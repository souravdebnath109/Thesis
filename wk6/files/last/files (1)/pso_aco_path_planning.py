"""
PSO-Optimized ACO Path Planning with Blockage Handling
Dhaka Traffic Dataset - Google Colab Compatible

Based on: "Dynamic Path Planning Based on Improved Ant Colony Algorithm 
in Traffic Congestion" (Wu et al., 2020)
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime
import json

# For Google Colab file download
try:
    from google.colab import files
    COLAB_ENV = True
except ImportError:
    COLAB_ENV = False

class PSOOptimizer:
    """Particle Swarm Optimization for ACO parameter tuning"""
    
    def __init__(self, n_particles=20, n_iterations=30):
        self.n_particles = n_particles
        self.n_iterations = n_iterations
        
        # Parameter bounds for ACO
        self.bounds = {
            'alpha': (0.5, 1.5),    # Pheromone importance
            'beta': (1.0, 5.0),      # Heuristic importance
            'rho': (0.5, 1.0)        # Evaporation rate
        }
        
    def optimize(self, objective_function):
        """
        Optimize ACO parameters using PSO
        
        Parameters:
        -----------
        objective_function : callable
            Function that takes (alpha, beta, rho) and returns fitness score
        
        Returns:
        --------
        dict : Best parameters found
        """
        # Initialize particles
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
        
        # Global best
        global_best = None
        global_best_score = float('inf')
        
        # PSO parameters
        w = 0.7  # Inertia weight
        c1 = 1.5  # Cognitive parameter
        c2 = 1.5  # Social parameter
        
        print("\n🔄 Running PSO optimization...")
        
        for iteration in range(self.n_iterations):
            for i in range(self.n_particles):
                # Evaluate fitness
                score = objective_function(
                    particles[i]['alpha'],
                    particles[i]['beta'],
                    particles[i]['rho']
                )
                
                # Update personal best
                if score < personal_best_scores[i]:
                    personal_best_scores[i] = score
                    personal_best[i] = particles[i].copy()
                
                # Update global best
                if score < global_best_score:
                    global_best_score = score
                    global_best = particles[i].copy()
            
            # Update velocities and positions
            for i in range(self.n_particles):
                for param in ['alpha', 'beta', 'rho']:
                    r1, r2 = np.random.random(), np.random.random()
                    
                    # Update velocity
                    velocities[i][param] = (
                        w * velocities[i][param] +
                        c1 * r1 * (personal_best[i][param] - particles[i][param]) +
                        c2 * r2 * (global_best[param] - particles[i][param])
                    )
                    
                    # Update position
                    particles[i][param] += velocities[i][param]
                    
                    # Apply bounds
                    particles[i][param] = np.clip(
                        particles[i][param],
                        self.bounds[param][0],
                        self.bounds[param][1]
                    )
            
            if (iteration + 1) % 10 == 0:
                print(f"   Iteration {iteration + 1}/{self.n_iterations} - "
                      f"Best score: {global_best_score:.4f}")
        
        print(f"\n✓ PSO optimization complete!")
        print(f"   Best parameters: α={global_best['alpha']:.2f}, "
              f"β={global_best['beta']:.2f}, ρ={global_best['rho']:.2f}")
        
        return global_best


class AntColonyOptimizer:
    """Ant Colony Optimization for path planning"""
    
    def __init__(self, graph, alpha=1.0, beta=2.5, rho=0.8, 
                 n_ants=10, n_iterations=100, Q=100):
        """
        Initialize ACO
        
        Parameters:
        -----------
        graph : dict
            Graph representation with edges and costs
        alpha : float
            Pheromone importance
        beta : float
            Heuristic importance (distance/cost)
        rho : float
            Pheromone evaporation rate
        n_ants : int
            Number of ants
        n_iterations : int
            Number of iterations
        Q : float
            Pheromone deposit factor
        """
        self.graph = graph
        self.alpha = alpha
        self.beta = beta
        self.rho = rho
        self.n_ants = n_ants
        self.n_iterations = n_iterations
        self.Q = Q
        
        # Initialize pheromones
        self.pheromones = {}
        for edge in graph['edges']:
            self.pheromones[edge] = 1.0
    
    def find_path(self, start, end):
        """Find optimal path from start to end"""
        
        best_path = None
        best_cost = float('inf')
        convergence_history = []
        
        for iteration in range(self.n_iterations):
            # Each ant finds a path
            iteration_best_path = None
            iteration_best_cost = float('inf')
            
            for ant in range(self.n_ants):
                path, cost = self._ant_find_path(start, end)
                
                if path and cost < iteration_best_cost:
                    iteration_best_cost = cost
                    iteration_best_path = path
            
            # Update global best
            if iteration_best_path and iteration_best_cost < best_cost:
                best_cost = iteration_best_cost
                best_path = iteration_best_path
            
            # Update pheromones
            self._update_pheromones(iteration_best_path, iteration_best_cost)
            
            convergence_history.append(best_cost)
        
        return best_path, best_cost, convergence_history
    
    def _ant_find_path(self, start, end):
        """Single ant finds a path"""
        
        current = start
        path = [current]
        total_cost = 0
        visited = {current}
        
        while current != end:
            # Get possible next nodes
            neighbors = self.graph['neighbors'].get(current, [])
            unvisited_neighbors = [n for n in neighbors if n not in visited]
            
            if not unvisited_neighbors:
                return None, float('inf')  # Dead end
            
            # Calculate probabilities
            probabilities = []
            for neighbor in unvisited_neighbors:
                edge = (current, neighbor)
                if edge not in self.pheromones:
                    edge = (neighbor, current)
                
                pheromone = self.pheromones.get(edge, 1.0)
                cost = self.graph['costs'].get(edge, 1.0)
                heuristic = 1.0 / (cost + 0.001)  # Avoid division by zero
                
                prob = (pheromone ** self.alpha) * (heuristic ** self.beta)
                probabilities.append(prob)
            
            # Normalize probabilities
            prob_sum = sum(probabilities)
            if prob_sum == 0:
                probabilities = [1.0] * len(probabilities)
                prob_sum = len(probabilities)
            
            probabilities = [p / prob_sum for p in probabilities]
            
            # Select next node
            next_node = np.random.choice(unvisited_neighbors, p=probabilities)
            
            # Update path
            edge = (current, next_node)
            if edge not in self.graph['costs']:
                edge = (next_node, current)
            
            total_cost += self.graph['costs'].get(edge, 0)
            path.append(next_node)
            visited.add(next_node)
            current = next_node
        
        return path, total_cost
    
    def _update_pheromones(self, best_path, best_cost):
        """Update pheromone levels"""
        
        # Evaporate
        for edge in self.pheromones:
            self.pheromones[edge] *= (1 - self.rho)
        
        # Deposit on best path
        if best_path and len(best_path) > 1:
            deposit = self.Q / (best_cost + 0.001)
            
            for i in range(len(best_path) - 1):
                edge = (best_path[i], best_path[i + 1])
                if edge not in self.pheromones:
                    edge = (best_path[i + 1], best_path[i])
                
                if edge in self.pheromones:
                    self.pheromones[edge] += deposit


def load_datasets(data_dir):
    """Load all datasets including blockages"""
    
    print("📁 Loading datasets...")
    
    datasets = {}
    datasets['intersections'] = pd.read_csv(os.path.join(data_dir, 'intersections.csv'))
    datasets['roads'] = pd.read_csv(os.path.join(data_dir, 'roads.csv'))
    datasets['time_patterns'] = pd.read_csv(os.path.join(data_dir, 'time_patterns.csv'))
    datasets['traffic'] = pd.read_csv(os.path.join(data_dir, 'traffic.csv'))
    datasets['congestion'] = pd.read_csv(os.path.join(data_dir, 'congestion_data.csv'))
    datasets['blockage_events'] = pd.read_csv(os.path.join(data_dir, 'blockage_events.csv'))
    datasets['blockage_impact'] = pd.read_csv(os.path.join(data_dir, 'blockage_impact.csv'))
    
    print("✓ All datasets loaded!")
    
    return datasets


def calculate_road_condition_factor_with_blockage(road_id, time_interval, datasets):
    """
    Calculate road condition factor including blockage impact
    
    Formula: R_i(t) = d_i × (1 + C_i(t) + B_i(t))
    Where:
    - d_i: road distance
    - C_i(t): congestion coefficient
    - B_i(t): blockage severity
    """
    
    # Get road info
    road_info = datasets['roads'][datasets['roads']['road_id'] == road_id].iloc[0]
    distance = road_info['distance_km']
    
    # Get congestion data
    congestion_data = datasets['congestion'][
        (datasets['congestion']['road_id'] == road_id) &
        (datasets['congestion']['time_interval'] == time_interval)
    ]
    
    if len(congestion_data) > 0:
        congestion_coef = congestion_data['congestion_coefficient'].mean()
    else:
        congestion_coef = 0
    
    # Get blockage severity
    blockage_data = datasets['blockage_impact'][
        (datasets['blockage_impact']['road_id'] == road_id) &
        (datasets['blockage_impact']['time_interval'] == time_interval)
    ]
    
    if len(blockage_data) > 0:
        blockage_severity = blockage_data['blockage_severity'].values[0]
    else:
        blockage_severity = 0
    
    # Calculate road condition factor
    road_condition_factor = distance * (1 + congestion_coef + blockage_severity)
    
    return road_condition_factor, congestion_coef, blockage_severity


def build_graph(datasets, time_interval, include_blockages=True):
    """Build graph representation for ACO"""
    
    graph = {
        'edges': [],
        'costs': {},
        'neighbors': {}
    }
    
    # Build graph from roads
    for _, road in datasets['roads'].iterrows():
        from_node = road['from_intersection']
        to_node = road['to_intersection']
        road_id = road['road_id']
        
        # Calculate cost
        if include_blockages:
            cost, _, _ = calculate_road_condition_factor_with_blockage(
                road_id, time_interval, datasets
            )
        else:
            # Use only distance
            cost = road['distance_km']
        
        # Add edge (bidirectional)
        edge1 = (from_node, to_node)
        edge2 = (to_node, from_node)
        
        graph['edges'].extend([edge1, edge2])
        graph['costs'][edge1] = cost
        graph['costs'][edge2] = cost
        
        # Add to neighbors
        if from_node not in graph['neighbors']:
            graph['neighbors'][from_node] = []
        if to_node not in graph['neighbors']:
            graph['neighbors'][to_node] = []
        
        graph['neighbors'][from_node].append(to_node)
        graph['neighbors'][to_node].append(from_node)
    
    return graph


def run_pso_aco_path_planning(datasets, start, end, time_interval, data_dir):
    """
    Complete PSO-ACO path planning workflow
    """
    
    results = {}
    
    print("\n" + "="*70)
    print("PSO-OPTIMIZED ACO PATH PLANNING WITH BLOCKAGE HANDLING")
    print("="*70)
    
    # Get time info
    time_info = datasets['time_patterns'][
        datasets['time_patterns']['time_interval'] == time_interval
    ].iloc[0]
    
    print(f"\n⏰ Planning Time: {time_info['time']} (Interval {time_interval})")
    print(f"   Congestion Level: {time_info['congestion_level']}")
    print(f"   Rush Hour: {'Yes' if time_info['is_rush_hour'] == 1 else 'No'}")
    
    # Check for active blockages
    active_blockages = datasets['blockage_impact'][
        (datasets['blockage_impact']['time_interval'] == time_interval) &
        (datasets['blockage_impact']['has_blockage'] == 1)
    ]
    
    print(f"\n🚧 Active Blockages: {len(active_blockages)}")
    if len(active_blockages) > 0:
        for _, blockage in active_blockages.iterrows():
            road_name = datasets['roads'][
                datasets['roads']['road_id'] == blockage['road_id']
            ]['road_name'].values[0]
            print(f"   • {road_name}: Severity {blockage['blockage_severity']:.2f}")
    
    # Build graph WITH blockages
    print("\n📊 Building graph with blockages...")
    graph_with_blockage = build_graph(datasets, time_interval, include_blockages=True)
    
    # Build graph WITHOUT blockages (for comparison)
    print("📊 Building graph without blockages...")
    graph_no_blockage = build_graph(datasets, time_interval, include_blockages=False)
    
    # PSO Optimization
    print("\n" + "-"*70)
    print("STEP 1: PSO Parameter Optimization")
    print("-"*70)
    
    def objective_function(alpha, beta, rho):
        """Objective function for PSO (minimize average path cost)"""
        aco = AntColonyOptimizer(
            graph_with_blockage,
            alpha=alpha,
            beta=beta,
            rho=rho,
            n_ants=5,
            n_iterations=20
        )
        _, cost, _ = aco.find_path(start, end)
        return cost
    
    pso = PSOOptimizer(n_particles=15, n_iterations=20)
    best_params = pso.optimize(objective_function)
    
    results['pso_params'] = best_params
    
    # ACO with optimized parameters (WITH blockages)
    print("\n" + "-"*70)
    print("STEP 2: ACO Path Planning WITH Blockages")
    print("-"*70)
    
    aco_with_blockage = AntColonyOptimizer(
        graph_with_blockage,
        alpha=best_params['alpha'],
        beta=best_params['beta'],
        rho=best_params['rho'],
        n_ants=10,
        n_iterations=100
    )
    
    path_with, cost_with, conv_with = aco_with_blockage.find_path(start, end)
    
    print(f"\n✓ Path found (WITH blockages):")
    print(f"   Route: {' → '.join(map(str, path_with))}")
    print(f"   Total cost: {cost_with:.4f}")
    
    results['path_with_blockage'] = path_with
    results['cost_with_blockage'] = cost_with
    results['convergence_with_blockage'] = conv_with
    
    # ACO WITHOUT blockages (for comparison)
    print("\n" + "-"*70)
    print("STEP 3: ACO Path Planning WITHOUT Blockages (Comparison)")
    print("-"*70)
    
    aco_no_blockage = AntColonyOptimizer(
        graph_no_blockage,
        alpha=best_params['alpha'],
        beta=best_params['beta'],
        rho=best_params['rho'],
        n_ants=10,
        n_iterations=100
    )
    
    path_without, cost_without, conv_without = aco_no_blockage.find_path(start, end)
    
    print(f"\n✓ Path found (WITHOUT blockages):")
    print(f"   Route: {' → '.join(map(str, path_without))}")
    print(f"   Total cost: {cost_without:.4f}")
    
    results['path_without_blockage'] = path_without
    results['cost_without_blockage'] = cost_without
    results['convergence_without_blockage'] = conv_without
    
    # Analysis
    print("\n" + "="*70)
    print("RESULTS ANALYSIS")
    print("="*70)
    
    cost_increase = ((cost_with - cost_without) / cost_without) * 100
    
    print(f"\n📊 Impact of Blockages:")
    print(f"   Cost WITHOUT blockages: {cost_without:.4f}")
    print(f"   Cost WITH blockages: {cost_with:.4f}")
    print(f"   Cost increase: {cost_increase:.2f}%")
    
    print(f"\n🛣️  Path Comparison:")
    same_path = (path_with == path_without)
    if same_path:
        print("   ⚠️  Same path taken despite blockages")
    else:
        print("   ✓ Different path chosen to avoid blockages")
    
    # Detailed route analysis
    print(f"\n📍 Detailed Route Analysis:")
    print(f"\nRoute WITH blockages:")
    analyze_route(path_with, datasets, time_interval, include_blockages=True)
    
    print(f"\nRoute WITHOUT blockages:")
    analyze_route(path_without, datasets, time_interval, include_blockages=False)
    
    # Visualization
    visualize_results(results, datasets, time_interval, data_dir)
    
    # Save results
    save_results(results, datasets, time_interval, data_dir)
    
    return results


def analyze_route(path, datasets, time_interval, include_blockages=True):
    """Analyze a route in detail"""
    
    if not path or len(path) < 2:
        print("   No valid path")
        return
    
    total_distance = 0
    total_congestion = 0
    total_blockage = 0
    
    for i in range(len(path) - 1):
        from_node = path[i]
        to_node = path[i + 1]
        
        # Find road
        road = datasets['roads'][
            ((datasets['roads']['from_intersection'] == from_node) &
             (datasets['roads']['to_intersection'] == to_node)) |
            ((datasets['roads']['from_intersection'] == to_node) &
             (datasets['roads']['to_intersection'] == from_node))
        ]
        
        if len(road) == 0:
            continue
        
        road = road.iloc[0]
        road_name = road['road_name']
        distance = road['distance_km']
        
        # Get congestion and blockage
        _, congestion, blockage = calculate_road_condition_factor_with_blockage(
            road['road_id'], time_interval, datasets
        )
        
        total_distance += distance
        total_congestion += congestion
        total_blockage += blockage
        
        blockage_text = f", Blockage: {blockage:.2f}" if include_blockages else ""
        print(f"   {i+1}. {road_name}: {distance:.2f} km "
              f"(Congestion: {congestion:.2f}{blockage_text})")
    
    print(f"   Total Distance: {total_distance:.2f} km")
    if include_blockages:
        print(f"   Total Blockage Impact: {total_blockage:.2f}")


def visualize_results(results, datasets, time_interval, data_dir):
    """Create visualizations"""
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # 1. Convergence comparison
    axes[0, 0].plot(results['convergence_with_blockage'], 
                    label='With Blockages', linewidth=2, color='red')
    axes[0, 0].plot(results['convergence_without_blockage'], 
                    label='Without Blockages', linewidth=2, color='blue')
    axes[0, 0].set_xlabel('Iteration')
    axes[0, 0].set_ylabel('Best Cost')
    axes[0, 0].set_title('ACO Convergence Comparison')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # 2. Cost comparison
    axes[0, 1].bar(['Without\nBlockages', 'With\nBlockages'],
                   [results['cost_without_blockage'], results['cost_with_blockage']],
                   color=['blue', 'red'])
    axes[0, 1].set_ylabel('Total Path Cost')
    axes[0, 1].set_title('Path Cost Comparison')
    axes[0, 1].grid(True, alpha=0.3, axis='y')
    
    # 3. PSO Parameters
    params = results['pso_params']
    axes[1, 0].bar(['α (Alpha)', 'β (Beta)', 'ρ (Rho)'],
                   [params['alpha'], params['beta'], params['rho']],
                   color=['green', 'orange', 'purple'])
    axes[1, 0].set_ylabel('Parameter Value')
    axes[1, 0].set_title('Optimized ACO Parameters (PSO)')
    axes[1, 0].grid(True, alpha=0.3, axis='y')
    
    # 4. Blockage impact by road
    blockages = datasets['blockage_impact'][
        (datasets['blockage_impact']['time_interval'] == time_interval) &
        (datasets['blockage_impact']['has_blockage'] == 1)
    ]
    
    if len(blockages) > 0:
        road_names = []
        severities = []
        for _, b in blockages.iterrows():
            road_name = datasets['roads'][
                datasets['roads']['road_id'] == b['road_id']
            ]['road_name'].values[0]
            road_names.append(road_name)
            severities.append(b['blockage_severity'])
        
        axes[1, 1].barh(road_names, severities, color='coral')
        axes[1, 1].set_xlabel('Blockage Severity')
        axes[1, 1].set_title('Active Blockages at Planning Time')
        axes[1, 1].grid(True, alpha=0.3, axis='x')
    else:
        axes[1, 1].text(0.5, 0.5, 'No Active Blockages', 
                        ha='center', va='center', fontsize=14)
        axes[1, 1].set_title('Active Blockages at Planning Time')
    
    plt.suptitle('PSO-Optimized ACO Path Planning Results', 
                 fontsize=16, fontweight='bold', y=0.995)
    plt.tight_layout()
    
    # Save
    output_path = os.path.join(data_dir, 'pso_aco_results.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.show()
    
    print(f"\n✓ Visualization saved: pso_aco_results.png")


def save_results(results, datasets, time_interval, data_dir):
    """Save results to files"""
    
    # Prepare results for saving
    output = {
        'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        'time_interval': int(time_interval),
        'pso_parameters': {
            'alpha': float(results['pso_params']['alpha']),
            'beta': float(results['pso_params']['beta']),
            'rho': float(results['pso_params']['rho'])
        },
        'path_with_blockage': [int(x) for x in results['path_with_blockage']],
        'cost_with_blockage': float(results['cost_with_blockage']),
        'path_without_blockage': [int(x) for x in results['path_without_blockage']],
        'cost_without_blockage': float(results['cost_without_blockage']),
        'cost_increase_percent': float(
            ((results['cost_with_blockage'] - results['cost_without_blockage']) / 
             results['cost_without_blockage']) * 100
        )
    }
    
    # Save as JSON
    json_path = os.path.join(data_dir, 'pso_aco_results.json')
    with open(json_path, 'w') as f:
        json.dump(output, f, indent=2)
    
    print(f"✓ Results saved: pso_aco_results.json")
    
    # Create detailed CSV report
    report_data = []
    for i, node in enumerate(results['path_with_blockage']):
        inter_name = datasets['intersections'][
            datasets['intersections']['intersection_id'] == node
        ]['intersection_name'].values[0]
        
        report_data.append({
            'step': i + 1,
            'intersection_id': node,
            'intersection_name': inter_name
        })
    
    report_df = pd.DataFrame(report_data)
    csv_path = os.path.join(data_dir, 'optimal_path_report.csv')
    report_df.to_csv(csv_path, index=False)
    
    print(f"✓ Path report saved: optimal_path_report.csv")


def main():
    """Main execution function"""
    
    # Configuration
    DATA_DIR = '/content/drive/MyDrive/'
    START_NODE = 0  # Shahbag
    END_NODE = 3    # Mohakhali
    TIME_INTERVAL = 14  # 7:00 AM (morning rush)
    
    print("="*70)
    print("PSO-OPTIMIZED ACO PATH PLANNING - DHAKA TRAFFIC")
    print("="*70)
    
    # Load datasets
    datasets = load_datasets(DATA_DIR)
    
    # Run path planning
    results = run_pso_aco_path_planning(
        datasets, 
        START_NODE, 
        END_NODE, 
        TIME_INTERVAL,
        DATA_DIR
    )
    
    print("\n" + "="*70)
    print("✅ PATH PLANNING COMPLETED SUCCESSFULLY!")
    print("="*70)
    
    # Download results if in Colab
    if COLAB_ENV:
        print("\n📥 Downloading results...")
        try:
            files.download(os.path.join(DATA_DIR, 'pso_aco_results.png'))
            files.download(os.path.join(DATA_DIR, 'pso_aco_results.json'))
            files.download(os.path.join(DATA_DIR, 'optimal_path_report.csv'))
            print("✓ Results downloaded!")
        except Exception as e:
            print(f"Note: Auto-download failed ({e}). Files saved to Google Drive.")
    
    print(f"\nAll results saved to: {DATA_DIR}")
    
    return results


if __name__ == "__main__":
    main()
