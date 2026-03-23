# Dhaka City Traffic Dataset - Synthetic Data Generation

## Overview
This dataset is generated for traffic congestion analysis and dynamic path planning in Dhaka city, based on the methodology from the paper "Dynamic Path Planning Based on Improved Ant Colony Algorithm in Traffic Congestion" (Wu et al., 2020).

## Dataset Description

### Scale
- **4 Intersections**: Famous locations in Dhaka (Shahbag, Karwan Bazar, Farmgate, Mohakhali)
- **4 Roads**: Major arterial roads connecting these intersections
- **7 Vehicle Categories**: Bus, Autorickshaw, Van, PrivateCar, Jeep, Truck, Micro
- **48 Time Intervals**: 30-minute intervals covering 24 hours

### Innovation
Unlike the original paper which used only distance parameters, this dataset includes:
1. **Multiple vehicle categories** specific to Dhaka's traffic composition
2. **Time-varying traffic patterns** reflecting Dhaka's rush hour dynamics
3. **Road condition factors** instead of simple distance metrics
4. **Congestion coefficients** calculated using the paper's formula

## Files Description

### 1. intersections.csv (4 records)
Describes the road network intersections in Dhaka.

**Columns:**
- `intersection_id` (int): Unique identifier for each intersection
- `intersection_name` (str): Name of the intersection location
- `latitude` (float): Geographic latitude coordinate
- `longitude` (float): Geographic longitude coordinate
- `type` (str): 'edge' or 'non-edge' intersection
- `x_coordinate` (int): Simplified X position for visualization
- `y_coordinate` (int): Simplified Y position for visualization
- `district` (str): Administrative district (all "Dhaka")
- `area` (str): Local area name

**Example:**
```
intersection_id,intersection_name,latitude,longitude,type
0,Shahbag,23.7389,90.3952,edge
1,Karwan Bazar,23.7500,90.3917,edge
```

### 2. roads.csv (4 records)
Contains information about roads connecting intersections.

**Columns:**
- `road_id` (int): Unique identifier for each road
- `road_name` (str): Name of the road
- `from_intersection` (int): Starting intersection ID
- `to_intersection` (int): Ending intersection ID
- `distance_km` (float): Physical distance in kilometers
- `num_lanes` (int): Number of traffic lanes (2-4)
- `road_type` (str): Classification (arterial, collector, main)
- `speed_limit_kmh` (int): Legal speed limit (40-60 km/h)
- `condition` (str): Road condition (good, fair, poor)

**Example:**
```
road_id,road_name,from_intersection,to_intersection,distance_km,num_lanes
0,Kazi Nazrul Islam Avenue,0,1,1.16,4
```

### 3. time_patterns.csv (48 records)
Defines traffic patterns throughout the day specific to Dhaka.

**Columns:**
- `time_interval` (int): Interval ID (0-47)
- `time` (str): Time in HH:MM format
- `hour` (int): Hour of the day (0-23)
- `traffic_multiplier` (float): Traffic intensity multiplier
- `congestion_level` (str): Qualitative level (low, moderate, high, severe)
- `is_rush_hour` (bool): 1 if rush hour (7-10 AM or 5-9 PM), 0 otherwise

**Rush Hour Patterns:**
- Morning rush: 7:00 AM - 10:00 AM (multiplier: 2.0-3.5)
- Evening rush: 5:00 PM - 9:00 PM (multiplier: 2.5-4.0)
- Daytime: 11:00 AM - 4:00 PM (multiplier: 1.2-1.8)
- Night: 10:00 PM - 6:00 AM (multiplier: 0.3-0.8)

### 4. traffic.csv (1,344 records)
Vehicle flow data for each road, time interval, and vehicle type.

**Columns:**
- `road_id` (int): Road identifier
- `time_interval` (int): Time interval (0-47)
- `vehicle_type` (str): Type of vehicle
- `inflow` (int): Number of vehicles entering the road segment
- `outflow` (int): Number of vehicles exiting the road segment
- `net_flow` (int): Net change (inflow - outflow)
- `hour` (int): Hour of day

**Vehicle Type Distribution (base flows per 30 min):**
- Bus: 50 vehicles
- Autorickshaw: 120 vehicles (most common in Dhaka)
- Van: 30 vehicles
- PrivateCar: 200 vehicles
- Jeep: 40 vehicles
- Truck: 25 vehicles
- Micro: 60 vehicles (microbus/tempo)

**Total Records:** 4 roads × 48 intervals × 7 vehicle types = 1,344 records

### 5. congestion_data.csv (1,344 records)
Comprehensive congestion metrics calculated using the paper's methodology.

**Columns:**
- `road_id` (int): Road identifier
- `time_interval` (int): Time interval
- `vehicle_type` (str): Vehicle type
- `inflow` (int): Vehicles entering
- `outflow` (int): Vehicles exiting
- `congestion_coefficient` (float): Calculated using paper's formula
- `tci` (float): Traffic Congestion Index (0-10 scale)
- `congestion_status` (str): Status category
- `avg_speed_kmh` (float): Average vehicle speed
- `road_condition_factor` (float): Distance × (1 + congestion_coefficient)

**Congestion Formula (from paper):**
```
C_i(t) = (f_i(t-1) + f_in(t) - f_out(t)) × L / (l_i × d_i)

Where:
- f_i(t-1): Vehicles on road at previous time
- f_in(t): Inflow at current time
- f_out(t): Outflow at current time
- L: Average vehicle length (0.005 km = 5 meters)
- l_i: Number of lanes
- d_i: Road distance in km
```

**Road Condition Factor (from paper):**
```
R_i(t) = d_i × (1 + C_i(t))
```

**Traffic Congestion Index (TCI) Categories:**
- 0-2: Unobstructed
- 2-4: Basically unblocked
- 4-6: Mild congestion
- 6-8: Moderate congestion
- 8-10: Severe congestion

## Usage Examples

### Python - Load and Analyze Data

```python
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load datasets
intersections = pd.read_csv('intersections.csv')
roads = pd.read_csv('roads.csv')
time_patterns = pd.read_csv('time_patterns.csv')
traffic = pd.read_csv('traffic.csv')
congestion = pd.read_csv('congestion_data.csv')

# Example 1: Analyze congestion by time of day
hourly_congestion = congestion.groupby('hour')['tci'].mean()
plt.plot(hourly_congestion)
plt.xlabel('Hour of Day')
plt.ylabel('Average TCI')
plt.title('Congestion Pattern Throughout the Day')
plt.show()

# Example 2: Vehicle type analysis
vehicle_flows = traffic.groupby('vehicle_type')['inflow'].sum()
print(vehicle_flows)

# Example 3: Find most congested roads
road_congestion = congestion.groupby('road_id')['congestion_coefficient'].mean()
most_congested = road_congestion.idxmax()
print(f"Most congested road: {roads.loc[most_congested, 'road_name']}")

# Example 4: Rush hour analysis
rush_hour_data = congestion[congestion['hour'].isin(range(7, 10)) | 
                            congestion['hour'].isin(range(17, 21))]
avg_rush_tci = rush_hour_data['tci'].mean()
print(f"Average TCI during rush hours: {avg_rush_tci:.2f}")
```

### Path Planning Application

```python
# Calculate optimal path using road condition factors
def calculate_path_cost(path_roads, time_interval):
    """Calculate total cost using road condition factors"""
    total_cost = 0
    for road_id in path_roads:
        road_data = congestion[
            (congestion['road_id'] == road_id) & 
            (congestion['time_interval'] == time_interval)
        ]
        total_cost += road_data['road_condition_factor'].mean()
    return total_cost

# Example: Compare two paths at morning rush (time_interval=14, 7:00 AM)
path1 = [0, 1, 2]  # Shahbag -> Karwan Bazar -> Farmgate -> Mohakhali
path2 = [3, 2]     # Shahbag -> Farmgate -> Mohakhali

cost1 = calculate_path_cost(path1, 14)
cost2 = calculate_path_cost(path2, 14)

print(f"Path 1 cost: {cost1:.2f}")
print(f"Path 2 cost: {cost2:.2f}")
print(f"Optimal path: {'Path 1' if cost1 < cost2 else 'Path 2'}")
```

## Key Features

1. **Realistic Dhaka Traffic Patterns**: Based on actual traffic behavior in Dhaka
2. **Multiple Vehicle Categories**: Reflects diverse Bangladeshi urban transport
3. **Time-Dependent Congestion**: Dynamic changes throughout the day
4. **Paper-Based Methodology**: Implements formulas from Wu et al. (2020)
5. **Small Scale**: Perfect for testing and prototyping (4 nodes, 4 edges)

## Limitations

1. **Synthetic Data**: Generated algorithmically, not from real sensors
2. **Small Scale**: Only 4 intersections (real Dhaka has thousands)
3. **Simplified Geography**: Simplified coordinate system
4. **No Traffic Incidents**: Doesn't model accidents or special events
5. **Uniform Vehicle Lengths**: Assumes average 5m per vehicle

## Extensions and Improvements

To enhance this dataset for research:

1. **Scale Up**: Increase to 10-20 intersections for more realistic scenarios
2. **Add Weather**: Include weather conditions affecting traffic
3. **Special Events**: Model effects of events, accidents, road work
4. **Real GPS Data**: Integrate actual probe vehicle data if available
5. **Machine Learning**: Train prediction models on this data
6. **Validation**: Compare with real Dhaka traffic data from Google Maps API

## Citation

If you use this dataset, please cite:

```bibtex
@article{wu2020dynamic,
  title={Dynamic Path Planning Based on Improved Ant Colony Algorithm in Traffic Congestion},
  author={Wu, Chunjiang and Zhou, Shijie and Xiao, Licai},
  journal={IEEE Access},
  volume={8},
  pages={180773--180783},
  year={2020},
  publisher={IEEE}
}
```

## License

This synthetic dataset is provided for research and educational purposes.

## Contact

For questions or suggestions about this dataset, please provide feedback through your research channels.

---

**Generated**: January 30, 2026  
**Version**: 1.0  
**Format**: CSV (UTF-8 encoding)
