import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt

# Read nodes and edges from CSV
try:
    nodes_df = pd.read_csv('nodes.csv')
    edges_df = pd.read_csv('map_data.csv')
except Exception as e:
    print(f"Error reading CSV files: {e}")
    exit(1)

# Create a directed graph
G = nx.DiGraph()

# Add nodes with their coordinates
pos = {}
for idx, row in nodes_df.iterrows():
    node_id = int(row['node_id'])
    x = float(row['x'])
    y = float(row['y'])
    G.add_node(node_id, pos=(x, y))
    pos[node_id] = (x, y)

# Add edges
for idx, row in edges_df.iterrows():
    u = int(row['from_node'])
    v = int(row['to_node'])
    length = row['length_m']
    lanes = int(row['lanes'])
    G.add_edge(u, v, length=length, lanes=lanes)

# Plotting the graph
plt.figure(figsize=(10, 8))

# Draw the nodes
nx.draw_networkx_nodes(G, pos, node_size=800, node_color='lightblue', edgecolors='black')

# Draw the labels
nx.draw_networkx_labels(G, pos, font_size=12, font_family='sans-serif', font_weight='bold')

# Draw the edges
# Using directed edges (arrows)
# Since the graph has bidirectional edges, drawing them can overlap. 
# We'll use connectionstyle to make dual edges curved
nx.draw_networkx_edges(G, pos, edgelist=G.edges(), width=2, alpha=0.7, arrowsize=20, edge_color='gray',
                      connectionstyle='arc3, rad=0.1')

# Optional: Add edge labels (e.g., length)
edge_labels = {(u, v): f"{d['length']:.1f}m" for u, v, d in G.edges(data=True)}

plt.title('Node Connectivity and Structure', fontsize=16)
plt.axis('off')
plt.grid(False)

# Save the figure
output_file = 'node_connectivity_graph.png'
plt.savefig(output_file, dpi=300, bbox_inches='tight')
print(f"Graph successfully saved as {output_file}")
