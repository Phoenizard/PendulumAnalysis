import json
import numpy as np
from utils.plot_status import plot_dict
from utils.tools import calculate_support_acceleration, add_jerk
import pandas as pd

# Parameters
g = 9.81  # Gravitational acceleration (m/s^2)
L, r = 18.0, 1.0  # Pendulum length and cabin radius (m)

# Read JSON file
def process_self_rotate_data(json_file_path, omega_self_0):
    """
    Load JSON data and add theta_s and omega_s fields
    
    Parameters:
    - json_file_path: path to the JSON file
    
    Returns:
    - data_dict: processed dictionary with added fields
    """
    # Read JSON
    with open(json_file_path, 'r') as f:
        data_dict = json.load(f)
    
    # First pass: calculate acc_pv using numerical differentiation
    sorted_keys = sorted(data_dict.keys(), key=int)
    n_states = len(sorted_keys)
    
    # Extract velocity array for differentiation
    times = np.array([data_dict[k]['time'] for k in sorted_keys])
    velocities = np.array([data_dict[k]['velocity'] for k in sorted_keys])
    
    # Calculate pendulum linear acceleration: dv/dt
    acc_pv_array = np.gradient(velocities, times)
    
    # Store acc_pv back to dictionary
    for i, key in enumerate(sorted_keys):
        data_dict[key]['acc_pv'] = float(acc_pv_array[i])
    
    # Second pass: process each state
    keys_to_remove = ['force_tangential', 'force_normal', 'acc_centripetal', 'acc_tangential', 'Tot', 'a_x', 'a_y', 'a_z', 'Jerk']
    
    for key in data_dict:
        # Remove unwanted keys
        for remove_key in keys_to_remove:
            if remove_key in data_dict[key]:
                del data_dict[key][remove_key]
        
        # Add new fields
        data_dict[key]['omega_s'] = omega_self_0  # self-rotation angular velocity (always positive)
        
        # Calculate customer's angular position (uniform circular motion)
        # Starting from 0 degrees at t=0, rotating with constant omega_self_0
        theta_s_raw = data_dict[key]['omega_s'] * data_dict[key]['time']
        data_dict[key]['theta_s'] = theta_s_raw % (2 * np.pi)  # Limit to [0, 2*pi)
        
        # Calculate Coriolis acceleration (scalar)
        data_dict[key]['coriolis'] = 2 * data_dict[key]['omega_s'] * data_dict[key]['velocity']
        
        # z-axis: vertical direction (head to feet), centripetal from pendulum swing
        data_dict[key]['accCorPas_z'] = (data_dict[key]['velocity'] ** 2) / L
        
        # x-axis: front direction (toward cabin center is negative)
        # Components:
        # 1. Self-rotation centripetal (negative, toward center)
        # 2. Coriolis component
        # 3. Pendulum linear acceleration component (perpendicular to arm, in cabin horizontal plane)
        data_dict[key]['accCorPas_x'] = (-data_dict[key]['omega_s']**2 * r - 
                                         data_dict[key]['coriolis'] * np.sin(data_dict[key]['theta_s']) +
                                         data_dict[key]['acc_pv'] * np.cos(-data_dict[key]['theta_s']))
        
        # y-axis: lateral direction (positive in rotation direction)
        # Components:
        # 1. Coriolis component
        # 2. Pendulum linear acceleration component
        data_dict[key]['accCorPas_y'] = (- data_dict[key]['coriolis'] * np.cos(data_dict[key]['theta_s']) -
                                         data_dict[key]['acc_pv'] * np.sin(data_dict[key]['theta_s']))
        
        # Calculate support force equivalent accelerations
        data_dict[key] = calculate_support_acceleration(data_dict[key])

    # Add jerk to the data dictionary
    data_dict = add_jerk(data_dict)
    return data_dict

def export_full_analysis(data_dict, output_path):
    """
    Export full analysis data with all fields.
    
    Parameters:
    - data_dict: processed data dictionary
    - output_path: output file path
    """
    with open(output_path, 'w') as f:
        json.dump(data_dict, f, indent=2)
    
    print(f"Full analysis data exported to: {output_path}")
    return data_dict

# Main execution
if __name__ == "__main__":
    # Specify your JSON file path here
    json_file_path = "state_s120.json"  # Replace with your actual file path
    
    # Load and process data
    data = process_self_rotate_data(json_file_path, omega_self_0=0.4)
    
    # Export data to JSON files
    # export_full_analysis(data, "AnalysisSelfRotationOverTop.json")
    # export_full_analysis(data, "AnalysisNoRotationOverTop.json")
    
    # Print first entry to verify
    print("\nFirst entry after processing:")
    print(json.dumps(data["0"], indent=2))
    
    print(f"\nTotal number of states: {len(data)}")

    # Plot the processed data
    plot_dict(data)

    # 导出CSV文件供后续分析
    df = pd.DataFrame.from_dict({k: v for k, v in data.items()}, orient='index')
    df.to_csv("AnalysisSR360_L18_os0_12s_N.csv", index_label='state_index')
    print("Data exported to csv")