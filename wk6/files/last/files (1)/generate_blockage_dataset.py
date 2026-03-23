"""
Blockage Events Dataset Generator for Dhaka Traffic
Adds realistic blockage events (congestion, construction, protests, accidents)
"""

import pandas as pd
import numpy as np
import random
from datetime import datetime, timedelta

# Set random seed
np.random.seed(42)
random.seed(42)

# Configuration
NUM_ROADS = 4
TIME_INTERVALS = 48  # 30-minute intervals for 24 hours

# Blockage types and their characteristics
BLOCKAGE_TYPES = {
    'none': {
        'probability': 0.70,  # 70% chance of no blockage
        'severity': 0.0,
        'duration_intervals': 0
    },
    'heavy_congestion': {
        'probability': 0.15,  # 15% chance
        'severity': 0.5,      # Medium severity
        'duration_intervals': (2, 6)  # 1-3 hours
    },
    'road_construction': {
        'probability': 0.05,  # 5% chance
        'severity': 0.8,      # High severity
        'duration_intervals': (4, 16)  # 2-8 hours
    },
    'protest': {
        'probability': 0.05,  # 5% chance (common in Dhaka)
        'severity': 0.9,      # Very high severity
        'duration_intervals': (2, 8)   # 1-4 hours
    },
    'accident': {
        'probability': 0.05,  # 5% chance
        'severity': 0.7,      # High severity
        'duration_intervals': (1, 4)   # 0.5-2 hours
    }
}

def generate_blockage_events():
    """Generate realistic blockage events for the road network"""
    
    data = []
    blockage_id = 0
    
    for road_id in range(NUM_ROADS):
        current_interval = 0
        
        while current_interval < TIME_INTERVALS:
            # Determine if blockage occurs
            rand_val = random.random()
            cumulative_prob = 0
            selected_type = 'none'
            
            for blockage_type, props in BLOCKAGE_TYPES.items():
                cumulative_prob += props['probability']
                if rand_val <= cumulative_prob:
                    selected_type = blockage_type
                    break
            
            if selected_type == 'none':
                # No blockage, move to next interval
                current_interval += 1
                continue
            
            # Blockage occurred
            props = BLOCKAGE_TYPES[selected_type]
            
            # Determine duration
            if isinstance(props['duration_intervals'], tuple):
                duration = random.randint(*props['duration_intervals'])
            else:
                duration = props['duration_intervals']
            
            # Ensure we don't exceed time intervals
            end_interval = min(current_interval + duration, TIME_INTERVALS)
            
            # Add severity variation
            severity_variation = random.uniform(0.8, 1.2)
            actual_severity = min(1.0, props['severity'] * severity_variation)
            
            # Create blockage event
            data.append({
                'blockage_id': blockage_id,
                'road_id': road_id,
                'blockage_type': selected_type,
                'start_interval': current_interval,
                'end_interval': end_interval,
                'duration_intervals': end_interval - current_interval,
                'severity': round(actual_severity, 2),
                'lanes_affected': random.randint(1, 3),
                'description': get_blockage_description(selected_type, actual_severity)
            })
            
            blockage_id += 1
            current_interval = end_interval
    
    return pd.DataFrame(data)


def get_blockage_description(blockage_type, severity):
    """Generate human-readable description"""
    
    descriptions = {
        'heavy_congestion': [
            "Heavy traffic jam due to rush hour",
            "Severe congestion from vehicle breakdown",
            "Traffic buildup from nearby intersection"
        ],
        'road_construction': [
            "Road repair and maintenance work",
            "Bridge construction blocking lanes",
            "Underground utility installation",
            "Road expansion project"
        ],
        'protest': [
            "Political demonstration blocking road",
            "Student protest on main road",
            "Strike by transport workers",
            "Public gathering blocking traffic"
        ],
        'accident': [
            "Multi-vehicle collision",
            "Single vehicle accident",
            "Motorcycle accident blocking lane",
            "Commercial vehicle breakdown"
        ]
    }
    
    if blockage_type in descriptions:
        desc = random.choice(descriptions[blockage_type])
        severity_level = "Minor" if severity < 0.3 else "Moderate" if severity < 0.7 else "Severe"
        return f"{severity_level}: {desc}"
    return "No description"


def create_blockage_impact_matrix():
    """Create a matrix showing when and where blockages occur"""
    
    # Load blockage events
    blockages = generate_blockage_events()
    
    # Create matrix: rows = roads, columns = time intervals
    impact_matrix = pd.DataFrame(0.0, 
                                 index=range(NUM_ROADS), 
                                 columns=range(TIME_INTERVALS))
    
    # Fill matrix with severity values
    for _, event in blockages.iterrows():
        road_id = event['road_id']
        for interval in range(event['start_interval'], event['end_interval']):
            # If multiple blockages overlap, take the maximum severity
            current_severity = impact_matrix.loc[road_id, interval]
            impact_matrix.loc[road_id, interval] = max(current_severity, event['severity'])
    
    # Reshape for CSV format
    impact_data = []
    for road_id in range(NUM_ROADS):
        for interval in range(TIME_INTERVALS):
            severity = impact_matrix.loc[road_id, interval]
            if severity > 0:  # Only include intervals with blockages
                impact_data.append({
                    'road_id': road_id,
                    'time_interval': interval,
                    'blockage_severity': severity,
                    'has_blockage': 1
                })
            else:
                impact_data.append({
                    'road_id': road_id,
                    'time_interval': interval,
                    'blockage_severity': 0.0,
                    'has_blockage': 0
                })
    
    return pd.DataFrame(impact_data)


def main():
    """Generate blockage datasets"""
    
    print("="*60)
    print("Blockage Events Dataset Generator for Dhaka Traffic")
    print("="*60)
    
    # Generate blockage events
    print("\n1. Generating blockage events...")
    blockages_df = generate_blockage_events()
    blockages_df.to_csv('/home/claude/blockage_events.csv', index=False)
    print(f"   ✓ Generated {len(blockages_df)} blockage events")
    
    # Generate blockage impact matrix
    print("\n2. Generating blockage impact matrix...")
    impact_df = create_blockage_impact_matrix()
    impact_df.to_csv('/home/claude/blockage_impact.csv', index=False)
    print(f"   ✓ Generated {len(impact_df)} impact records")
    
    print("\n" + "="*60)
    print("Blockage Dataset Summary")
    print("="*60)
    
    print("\n📊 Blockage Events by Type:")
    print(blockages_df['blockage_type'].value_counts())
    
    print("\n📊 Blockage Events by Road:")
    print(blockages_df['road_id'].value_counts().sort_index())
    
    print("\n📊 Average Severity by Type:")
    print(blockages_df.groupby('blockage_type')['severity'].mean().round(2))
    
    print("\n📊 Total Blockage Duration by Road (intervals):")
    print(blockages_df.groupby('road_id')['duration_intervals'].sum())
    
    print("\n📋 Sample Blockage Events:")
    print(blockages_df.head(10))
    
    print("\n📋 Sample Impact Data:")
    print(impact_df[impact_df['has_blockage'] == 1].head(10))
    
    print("\n" + "="*60)
    print("✅ Blockage datasets generated successfully!")
    print("="*60)
    print("\nFiles created:")
    print("  1. blockage_events.csv - Detailed blockage events")
    print("  2. blockage_impact.csv - Blockage impact matrix")
    

if __name__ == "__main__":
    main()
