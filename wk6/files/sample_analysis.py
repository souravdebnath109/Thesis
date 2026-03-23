"""
Sample Analysis Scripts for Dhaka Traffic Dataset
Demonstrates various analyses and visualizations
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# Set style
sns.set_style("whitegrid")
plt.rcParams['figure.figsize'] = (12, 6)

def load_data():
    """Load all CSV files"""
    intersections = pd.read_csv('intersections.csv')
    roads = pd.read_csv('roads.csv')
    time_patterns = pd.read_csv('time_patterns.csv')
    traffic = pd.read_csv('traffic.csv')
    congestion = pd.read_csv('congestion_data.csv')
    
    return intersections, roads, time_patterns, traffic, congestion


def analysis_1_congestion_by_hour(congestion, time_patterns):
    """Analyze how congestion changes throughout the day"""
    print("\n" + "="*60)
    print("ANALYSIS 1: Congestion by Hour of Day")
    print("="*60)
    
    # Merge with time patterns to get hour
    congestion_with_hour = congestion.merge(
        time_patterns[['time_interval', 'hour']], 
        on='time_interval'
    )
    
    hourly_stats = congestion_with_hour.groupby('hour').agg({
        'tci': ['mean', 'std', 'min', 'max'],
        'avg_speed_kmh': 'mean',
        'congestion_coefficient': 'mean'
    }).round(2)
    
    print(hourly_stats)
    
    # Plot
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    # TCI over time
    hourly_tci = congestion_with_hour.groupby('hour')['tci'].mean()
    axes[0].plot(hourly_tci.index, hourly_tci.values, marker='o', linewidth=2, markersize=8)
    axes[0].axhspan(0, 2, alpha=0.2, color='green', label='Unobstructed')
    axes[0].axhspan(2, 4, alpha=0.2, color='yellow', label='Basically unblocked')
    axes[0].axhspan(4, 6, alpha=0.2, color='orange', label='Mild congestion')
    axes[0].axhspan(6, 8, alpha=0.2, color='red', label='Moderate congestion')
    axes[0].axhspan(8, 10, alpha=0.2, color='darkred', label='Severe congestion')
    axes[0].set_xlabel('Hour of Day')
    axes[0].set_ylabel('Average TCI')
    axes[0].set_title('Traffic Congestion Index Throughout the Day')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    
    # Speed over time
    hourly_speed = congestion_with_hour.groupby('hour')['avg_speed_kmh'].mean()
    axes[1].plot(hourly_speed.index, hourly_speed.values, marker='s', 
                 linewidth=2, markersize=8, color='green')
    axes[1].set_xlabel('Hour of Day')
    axes[1].set_ylabel('Average Speed (km/h)')
    axes[1].set_title('Average Vehicle Speed Throughout the Day')
    axes[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('analysis_1_congestion_by_hour.png', dpi=300, bbox_inches='tight')
    print("\n✓ Plot saved as: analysis_1_congestion_by_hour.png")


def analysis_2_vehicle_type_distribution(traffic):
    """Analyze traffic flow by vehicle type"""
    print("\n" + "="*60)
    print("ANALYSIS 2: Vehicle Type Distribution")
    print("="*60)
    
    vehicle_stats = traffic.groupby('vehicle_type').agg({
        'inflow': 'sum',
        'outflow': 'sum',
        'net_flow': 'sum'
    }).sort_values('inflow', ascending=False)
    
    print(vehicle_stats)
    
    # Plot
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    # Bar chart of total inflow by vehicle type
    colors = plt.cm.Set3(range(len(vehicle_stats)))
    axes[0].bar(vehicle_stats.index, vehicle_stats['inflow'], color=colors)
    axes[0].set_xlabel('Vehicle Type')
    axes[0].set_ylabel('Total Inflow')
    axes[0].set_title('Total Vehicle Inflow by Type')
    axes[0].tick_params(axis='x', rotation=45)
    
    # Pie chart
    axes[1].pie(vehicle_stats['inflow'], labels=vehicle_stats.index, 
                autopct='%1.1f%%', colors=colors, startangle=90)
    axes[1].set_title('Vehicle Type Distribution')
    
    plt.tight_layout()
    plt.savefig('analysis_2_vehicle_distribution.png', dpi=300, bbox_inches='tight')
    print("\n✓ Plot saved as: analysis_2_vehicle_distribution.png")


def analysis_3_road_comparison(congestion, roads):
    """Compare congestion across different roads"""
    print("\n" + "="*60)
    print("ANALYSIS 3: Road-wise Congestion Comparison")
    print("="*60)
    
    # Merge with road names
    congestion_with_names = congestion.merge(
        roads[['road_id', 'road_name']], 
        on='road_id'
    )
    
    road_stats = congestion_with_names.groupby('road_name').agg({
        'tci': ['mean', 'std'],
        'congestion_coefficient': 'mean',
        'avg_speed_kmh': 'mean',
        'road_condition_factor': 'mean'
    }).round(2)
    
    print(road_stats)
    
    # Plot
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    road_tci = congestion_with_names.groupby('road_name')['tci'].mean().sort_values()
    
    # Average TCI by road
    axes[0, 0].barh(road_tci.index, road_tci.values, color='coral')
    axes[0, 0].set_xlabel('Average TCI')
    axes[0, 0].set_title('Average Congestion by Road')
    
    # Average speed by road
    road_speed = congestion_with_names.groupby('road_name')['avg_speed_kmh'].mean().sort_values()
    axes[0, 1].barh(road_speed.index, road_speed.values, color='lightblue')
    axes[0, 1].set_xlabel('Average Speed (km/h)')
    axes[0, 1].set_title('Average Speed by Road')
    
    # Road condition factor
    road_factor = congestion_with_names.groupby('road_name')['road_condition_factor'].mean().sort_values()
    axes[1, 0].barh(road_factor.index, road_factor.values, color='lightgreen')
    axes[1, 0].set_xlabel('Road Condition Factor')
    axes[1, 0].set_title('Road Condition Factor by Road')
    
    # Congestion coefficient
    road_coef = congestion_with_names.groupby('road_name')['congestion_coefficient'].mean().sort_values()
    axes[1, 1].barh(road_coef.index, road_coef.values, color='plum')
    axes[1, 1].set_xlabel('Congestion Coefficient')
    axes[1, 1].set_title('Congestion Coefficient by Road')
    
    plt.tight_layout()
    plt.savefig('analysis_3_road_comparison.png', dpi=300, bbox_inches='tight')
    print("\n✓ Plot saved as: analysis_3_road_comparison.png")


def analysis_4_rush_hour_impact(congestion, time_patterns):
    """Analyze impact of rush hours"""
    print("\n" + "="*60)
    print("ANALYSIS 4: Rush Hour Impact Analysis")
    print("="*60)
    
    # Merge with time patterns
    congestion_with_time = congestion.merge(
        time_patterns[['time_interval', 'is_rush_hour']], 
        on='time_interval'
    )
    
    rush_comparison = congestion_with_time.groupby('is_rush_hour').agg({
        'tci': ['mean', 'std'],
        'avg_speed_kmh': 'mean',
        'congestion_coefficient': 'mean'
    }).round(2)
    
    rush_comparison.index = ['Non-Rush Hour', 'Rush Hour']
    print(rush_comparison)
    
    # Calculate percentage increase
    rush_tci = congestion_with_time[congestion_with_time['is_rush_hour']==1]['tci'].mean()
    non_rush_tci = congestion_with_time[congestion_with_time['is_rush_hour']==0]['tci'].mean()
    pct_increase = ((rush_tci - non_rush_tci) / non_rush_tci) * 100
    
    print(f"\n📊 Rush hour TCI is {pct_increase:.1f}% higher than non-rush hours")
    
    # Plot
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # Box plot of TCI
    congestion_with_time['Period'] = congestion_with_time['is_rush_hour'].map({
        0: 'Non-Rush Hour',
        1: 'Rush Hour'
    })
    
    sns.boxplot(data=congestion_with_time, x='Period', y='tci', ax=axes[0])
    axes[0].set_ylabel('TCI')
    axes[0].set_title('TCI Distribution: Rush vs Non-Rush Hours')
    
    # Speed comparison
    sns.boxplot(data=congestion_with_time, x='Period', y='avg_speed_kmh', ax=axes[1])
    axes[1].set_ylabel('Speed (km/h)')
    axes[1].set_title('Speed Distribution: Rush vs Non-Rush Hours')
    
    plt.tight_layout()
    plt.savefig('analysis_4_rush_hour_impact.png', dpi=300, bbox_inches='tight')
    print("✓ Plot saved as: analysis_4_rush_hour_impact.png")


def analysis_5_vehicle_congestion_contribution(congestion):
    """Analyze which vehicle types contribute most to congestion"""
    print("\n" + "="*60)
    print("ANALYSIS 5: Vehicle Type Contribution to Congestion")
    print("="*60)
    
    vehicle_congestion = congestion.groupby('vehicle_type').agg({
        'congestion_coefficient': 'sum',
        'tci': 'mean',
        'inflow': 'sum'
    }).sort_values('congestion_coefficient', ascending=False)
    
    print(vehicle_congestion)
    
    # Plot
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    # Congestion contribution
    axes[0].bar(vehicle_congestion.index, vehicle_congestion['congestion_coefficient'],
                color=plt.cm.Reds(np.linspace(0.3, 0.9, len(vehicle_congestion))))
    axes[0].set_xlabel('Vehicle Type')
    axes[0].set_ylabel('Total Congestion Coefficient')
    axes[0].set_title('Congestion Contribution by Vehicle Type')
    axes[0].tick_params(axis='x', rotation=45)
    
    # Average TCI
    axes[1].bar(vehicle_congestion.index, vehicle_congestion['tci'],
                color=plt.cm.Blues(np.linspace(0.3, 0.9, len(vehicle_congestion))))
    axes[1].set_xlabel('Vehicle Type')
    axes[1].set_ylabel('Average TCI')
    axes[1].set_title('Average TCI by Vehicle Type')
    axes[1].tick_params(axis='x', rotation=45)
    
    plt.tight_layout()
    plt.savefig('analysis_5_vehicle_congestion.png', dpi=300, bbox_inches='tight')
    print("✓ Plot saved as: analysis_5_vehicle_congestion.png")


def main():
    """Run all analyses"""
    print("\n" + "="*60)
    print("DHAKA CITY TRAFFIC DATASET - COMPREHENSIVE ANALYSIS")
    print("="*60)
    
    # Load data
    print("\n📁 Loading datasets...")
    intersections, roads, time_patterns, traffic, congestion = load_data()
    print("✓ All datasets loaded successfully")
    
    print(f"\n📊 Dataset Statistics:")
    print(f"   - Intersections: {len(intersections)}")
    print(f"   - Roads: {len(roads)}")
    print(f"   - Time intervals: {len(time_patterns)}")
    print(f"   - Traffic records: {len(traffic)}")
    print(f"   - Congestion records: {len(congestion)}")
    
    # Run analyses
    analysis_1_congestion_by_hour(congestion, time_patterns)
    analysis_2_vehicle_type_distribution(traffic)
    analysis_3_road_comparison(congestion, roads)
    analysis_4_rush_hour_impact(congestion, time_patterns)
    analysis_5_vehicle_congestion_contribution(congestion)
    
    print("\n" + "="*60)
    print("✅ ALL ANALYSES COMPLETED")
    print("="*60)
    print("\nGenerated files:")
    print("  1. analysis_1_congestion_by_hour.png")
    print("  2. analysis_2_vehicle_distribution.png")
    print("  3. analysis_3_road_comparison.png")
    print("  4. analysis_4_rush_hour_impact.png")
    print("  5. analysis_5_vehicle_congestion.png")
    print("\n")


if __name__ == "__main__":
    main()
