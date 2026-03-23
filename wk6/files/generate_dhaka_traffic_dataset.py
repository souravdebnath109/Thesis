"""
Synthetic Traffic Dataset Generator for Dhaka City
Based on: Dynamic Path Planning Based on Improved Ant Colony Algorithm in Traffic Congestion

This script generates traffic data for a small-scale road network in Dhaka
with multiple vehicle categories and realistic traffic patterns.
"""

import pandas as pd
import numpy as np
from datetime import datetime, timedelta
import random

# Set random seed for reproducibility
np.random.seed(42)
random.seed(42)

# Configuration
NUM_INTERSECTIONS = 4
NUM_ROADS = 4
VEHICLE_TYPES = ['Bus', 'Autorickshaw', 'Van', 'PrivateCar', 'Jeep', 'Truck', 'Micro']
SIMULATION_HOURS = 24
TIME_INTERVALS = 48  # 30-minute intervals

# Dhaka-specific locations (famous intersections)
DHAKA_INTERSECTIONS = [
    {'id': 0, 'name': 'Shahbag', 'lat': 23.7389, 'lon': 90.3952, 'type': 'edge'},
    {'id': 1, 'name': 'Karwan Bazar', 'lat': 23.7500, 'lon': 90.3917, 'type': 'edge'},
    {'id': 2, 'name': 'Farmgate', 'lat': 23.7561, 'lon': 90.3872, 'type': 'non-edge'},
    {'id': 3, 'name': 'Mohakhali', 'lat': 23.7808, 'lon': 90.4067, 'type': 'edge'}
]

# Road connections (from_intersection, to_intersection)
ROAD_CONNECTIONS = [
    (0, 1),  # Shahbag to Karwan Bazar
    (1, 2),  # Karwan Bazar to Farmgate
    (2, 3),  # Farmgate to Mohakhali
    (0, 2)   # Shahbag to Farmgate (alternative route)
]


def generate_intersections_csv():
    """Generate intersections.csv with Dhaka city intersection data"""
    
    data = []
    for inter in DHAKA_INTERSECTIONS:
        data.append({
            'intersection_id': inter['id'],
            'intersection_name': inter['name'],
            'latitude': inter['lat'],
            'longitude': inter['lon'],
            'type': inter['type'],
            'x_coordinate': inter['id'] * 150,  # Simplified coordinate system
            'y_coordinate': inter['id'] * 100,
            'district': 'Dhaka',
            'area': inter['name']
        })
    
    df = pd.DataFrame(data)
    return df


def generate_roads_csv():
    """Generate roads.csv with road characteristics"""
    
    data = []
    road_names = [
        'Kazi Nazrul Islam Avenue',
        'Karwan Bazar Road',
        'Farmgate Link Road',
        'Shahbag Road'
    ]
    
    for idx, (from_id, to_id) in enumerate(ROAD_CONNECTIONS):
        from_inter = DHAKA_INTERSECTIONS[from_id]
        to_inter = DHAKA_INTERSECTIONS[to_id]
        
        # Calculate distance (simplified)
        distance = np.sqrt((from_inter['lat'] - to_inter['lat'])**2 + 
                          (from_inter['lon'] - to_inter['lon'])**2) * 100  # Scale to km
        
        data.append({
            'road_id': idx,
            'road_name': road_names[idx],
            'from_intersection': from_id,
            'to_intersection': to_id,
            'distance_km': round(distance, 2),
            'num_lanes': random.randint(2, 4),
            'road_type': random.choice(['arterial', 'collector', 'main']),
            'speed_limit_kmh': random.choice([40, 50, 60]),
            'condition': random.choice(['good', 'fair', 'poor'])
        })
    
    df = pd.DataFrame(data)
    return df


def generate_time_patterns_csv():
    """Generate time_patterns.csv with hourly traffic patterns for Dhaka"""
    
    data = []
    start_time = datetime.strptime("00:00", "%H:%M")
    
    # Dhaka traffic patterns (higher during rush hours)
    for interval in range(TIME_INTERVALS):
        current_time = start_time + timedelta(minutes=interval * 30)
        hour = current_time.hour
        
        # Define traffic multipliers based on Dhaka patterns
        if 7 <= hour <= 10:  # Morning rush
            multiplier = random.uniform(2.0, 3.5)
            congestion_level = 'high'
        elif 17 <= hour <= 21:  # Evening rush
            multiplier = random.uniform(2.5, 4.0)
            congestion_level = 'severe'
        elif 11 <= hour <= 16:  # Daytime
            multiplier = random.uniform(1.2, 1.8)
            congestion_level = 'moderate'
        else:  # Night/early morning
            multiplier = random.uniform(0.3, 0.8)
            congestion_level = 'low'
        
        data.append({
            'time_interval': interval,
            'time': current_time.strftime("%H:%M"),
            'hour': hour,
            'traffic_multiplier': round(multiplier, 2),
            'congestion_level': congestion_level,
            'is_rush_hour': 1 if (7 <= hour <= 10 or 17 <= hour <= 21) else 0
        })
    
    df = pd.DataFrame(data)
    return df


def generate_traffic_csv():
    """Generate traffic.csv with vehicle flow data by type"""
    
    data = []
    
    # Base traffic flow for each vehicle type (vehicles per 30 min)
    base_flows = {
        'Bus': 50,
        'Autorickshaw': 120,
        'Van': 30,
        'PrivateCar': 200,
        'Jeep': 40,
        'Truck': 25,
        'Micro': 60
    }
    
    for road_idx in range(NUM_ROADS):
        for time_interval in range(TIME_INTERVALS):
            # Get traffic multiplier from time patterns
            hour = (time_interval * 30) // 60
            
            if 7 <= hour <= 10:
                multiplier = random.uniform(2.0, 3.5)
            elif 17 <= hour <= 21:
                multiplier = random.uniform(2.5, 4.0)
            elif 11 <= hour <= 16:
                multiplier = random.uniform(1.2, 1.8)
            else:
                multiplier = random.uniform(0.3, 0.8)
            
            for vehicle_type in VEHICLE_TYPES:
                base_flow = base_flows[vehicle_type]
                
                # Add randomness
                inflow = int(base_flow * multiplier * random.uniform(0.8, 1.2))
                outflow = int(inflow * random.uniform(0.85, 1.15))
                
                # Ensure non-negative
                inflow = max(0, inflow)
                outflow = max(0, min(outflow, inflow))
                
                data.append({
                    'road_id': road_idx,
                    'time_interval': time_interval,
                    'vehicle_type': vehicle_type,
                    'inflow': inflow,
                    'outflow': outflow,
                    'net_flow': inflow - outflow,
                    'hour': hour
                })
    
    df = pd.DataFrame(data)
    return df


def calculate_congestion_coefficient(inflow, outflow, num_lanes, distance):
    """Calculate congestion coefficient based on paper formula"""
    
    AVG_VEHICLE_LENGTH = 0.005  # 5 meters in km
    
    if inflow > outflow:
        net_vehicles = inflow - outflow
        congestion = (net_vehicles * AVG_VEHICLE_LENGTH) / (num_lanes * distance)
    else:
        congestion = 0
    
    return round(congestion, 4)


def generate_congestion_data_csv(traffic_df, roads_df):
    """Generate congestion_data.csv with congestion metrics"""
    
    data = []
    
    for _, traffic_row in traffic_df.iterrows():
        road_id = traffic_row['road_id']
        road_info = roads_df[roads_df['road_id'] == road_id].iloc[0]
        
        total_inflow = traffic_row['inflow']
        total_outflow = traffic_row['outflow']
        
        congestion_coef = calculate_congestion_coefficient(
            total_inflow, 
            total_outflow,
            road_info['num_lanes'],
            road_info['distance_km']
        )
        
        # Calculate Traffic Congestion Index (TCI) based on paper
        if congestion_coef < 0.2:
            tci = random.uniform(0, 2)
            status = 'unobstructed'
        elif congestion_coef < 0.5:
            tci = random.uniform(2, 4)
            status = 'basically_unblocked'
        elif congestion_coef < 0.8:
            tci = random.uniform(4, 6)
            status = 'mild_congestion'
        elif congestion_coef < 1.2:
            tci = random.uniform(6, 8)
            status = 'moderate_congestion'
        else:
            tci = random.uniform(8, 10)
            status = 'severe_congestion'
        
        # Calculate average speed (inversely proportional to congestion)
        base_speed = road_info['speed_limit_kmh']
        avg_speed = base_speed * (1 / (1 + congestion_coef * 0.5))
        
        data.append({
            'road_id': road_id,
            'time_interval': traffic_row['time_interval'],
            'vehicle_type': traffic_row['vehicle_type'],
            'inflow': total_inflow,
            'outflow': total_outflow,
            'congestion_coefficient': congestion_coef,
            'tci': round(tci, 2),
            'congestion_status': status,
            'avg_speed_kmh': round(avg_speed, 2),
            'road_condition_factor': round(road_info['distance_km'] * (1 + congestion_coef), 4)
        })
    
    df = pd.DataFrame(data)
    return df


def main():
    """Generate all CSV files"""
    
    print("=" * 60)
    print("Synthetic Traffic Dataset Generator for Dhaka City")
    print("=" * 60)
    print(f"Configuration:")
    print(f"  - Intersections: {NUM_INTERSECTIONS}")
    print(f"  - Roads: {NUM_ROADS}")
    print(f"  - Vehicle Types: {', '.join(VEHICLE_TYPES)}")
    print(f"  - Time Intervals: {TIME_INTERVALS} (30-min intervals)")
    print("=" * 60)
    
    # Generate datasets
    print("\n1. Generating intersections.csv...")
    intersections_df = generate_intersections_csv()
    intersections_df.to_csv('/home/claude/intersections.csv', index=False)
    print(f"   ✓ Generated {len(intersections_df)} intersections")
    
    print("\n2. Generating roads.csv...")
    roads_df = generate_roads_csv()
    roads_df.to_csv('/home/claude/roads.csv', index=False)
    print(f"   ✓ Generated {len(roads_df)} roads")
    
    print("\n3. Generating time_patterns.csv...")
    time_patterns_df = generate_time_patterns_csv()
    time_patterns_df.to_csv('/home/claude/time_patterns.csv', index=False)
    print(f"   ✓ Generated {len(time_patterns_df)} time patterns")
    
    print("\n4. Generating traffic.csv...")
    traffic_df = generate_traffic_csv()
    traffic_df.to_csv('/home/claude/traffic.csv', index=False)
    print(f"   ✓ Generated {len(traffic_df)} traffic records")
    
    print("\n5. Generating congestion_data.csv...")
    congestion_df = generate_congestion_data_csv(traffic_df, roads_df)
    congestion_df.to_csv('/home/claude/congestion_data.csv', index=False)
    print(f"   ✓ Generated {len(congestion_df)} congestion records")
    
    print("\n" + "=" * 60)
    print("Dataset Generation Complete!")
    print("=" * 60)
    
    # Display summary statistics
    print("\n📊 Dataset Summary:")
    print(f"\n1. Intersections ({len(intersections_df)} records):")
    print(intersections_df.head())
    
    print(f"\n2. Roads ({len(roads_df)} records):")
    print(roads_df.head())
    
    print(f"\n3. Time Patterns ({len(time_patterns_df)} records):")
    print(time_patterns_df.head())
    
    print(f"\n4. Traffic ({len(traffic_df)} records):")
    print(f"   - Total records: {len(traffic_df)}")
    print(f"   - Vehicle types: {traffic_df['vehicle_type'].unique()}")
    print(f"   - Average inflow: {traffic_df['inflow'].mean():.2f}")
    print(f"   - Average outflow: {traffic_df['outflow'].mean():.2f}")
    
    print(f"\n5. Congestion Data ({len(congestion_df)} records):")
    print(f"   - Congestion statuses: {congestion_df['congestion_status'].value_counts().to_dict()}")
    print(f"   - Average TCI: {congestion_df['tci'].mean():.2f}")
    print(f"   - Average speed: {congestion_df['avg_speed_kmh'].mean():.2f} km/h")
    
    print("\n" + "=" * 60)
    print("All CSV files saved to /home/claude/")
    print("=" * 60)


if __name__ == "__main__":
    main()
