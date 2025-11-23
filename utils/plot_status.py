import matplotlib.pyplot as plt
import numpy as np

def plot_dict(state_dict):
    """
    Plot physical quantities from state_dict over time (3x2 layout)
    
    Parameters:
        state_dict: State dictionary containing time and various physical quantities
    """
    # Extract data (handle string keys)
    sorted_keys = sorted(state_dict.keys(), key=int)
    n_states = len(sorted_keys)
    
    time = np.array([state_dict[k]['time'] for k in sorted_keys])
    theta = np.array([state_dict[k]['theta'] for k in sorted_keys])
    omega = np.array([state_dict[k]['omega'] for k in sorted_keys])
    velocity = np.array([state_dict[k]['velocity'] for k in sorted_keys])
    dvdt = np.array([state_dict[k]['acc_pv'] for k in sorted_keys])
    
    # Passenger coordinate accelerations
    theta_s = np.array([state_dict[k]['theta_s'] for k in sorted_keys])
    acc_N_x = np.array([state_dict[k]['acc_N_x'] for k in sorted_keys])
    acc_N_y = np.array([state_dict[k]['acc_N_y'] for k in sorted_keys])
    acc_N_z = np.array([state_dict[k]['acc_N_z'] for k in sorted_keys])
    
    # Additional quantities
    jerk = np.array([state_dict[k]['jerk'] for k in sorted_keys])
    coriolis = np.array([state_dict[k]['coriolis'] for k in sorted_keys])
    
    # Create figure (3 rows x 2 columns)
    fig, axes = plt.subplots(4, 2, figsize=(14, 10))
    
    # LEFT COLUMN - Pendulum Motion
    
    # Row 1: Angle theta
    axes[0, 0].plot(time, np.rad2deg(theta), 'b-', linewidth=1.5)
    axes[0, 0].set_ylabel('Angle θ (Degree)', fontsize=11)
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].set_title('Pendulum Motion', fontsize=13, fontweight='bold')
    axes[0, 0].axhline(0, color='gray', linestyle='--', linewidth=0.8, alpha=0.5)
    
    # Row 2: Angular velocity omega
    axes[1, 0].plot(time, omega, 'g-', linewidth=1.5)
    axes[1, 0].set_ylabel(r'Angular Velocity $\omega_p$ (rad/s)', fontsize=11)
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].axhline(0, color='gray', linestyle='--', linewidth=0.8, alpha=0.5)
    
    # Row 3: Linear velocity
    axes[2, 0].plot(time, velocity, 'r-', linewidth=1.5)
    axes[2, 0].set_ylabel(r'Velocity v_p (m/s)', fontsize=11)
    axes[2, 0].set_xlabel('Time (s)', fontsize=11)
    axes[2, 0].grid(True, alpha=0.3)
    axes[2, 0].axhline(0, color='gray', linestyle='--', linewidth=0.8, alpha=0.5)

    # Row 4: Linear acceleration dv/dt
    axes[3, 0].plot(time, dvdt, 'm-', linewidth=1.5)
    axes[3, 0].set_ylabel('Acceleration dv/dt (m/s²)', fontsize=11)
    axes[3, 0].set_xlabel('Time (s)', fontsize=11)
    axes[3, 0].grid(True, alpha=0.3)
    axes[3, 0].axhline(0, color='gray', linestyle='--', linewidth=0.8, alpha=0.5)
    
    # RIGHT COLUMN - Passenger Experience
    
    # Row 1: Support force accelerations (3 directions)
    axes[0, 1].plot(time, acc_N_x, 'r-', linewidth=1.5, label='acc_N_x (front)', alpha=0.8)
    axes[0, 1].plot(time, acc_N_y, 'g-', linewidth=1.5, label='acc_N_y (lateral)', alpha=0.8)
    axes[0, 1].plot(time, acc_N_z, 'b-', linewidth=1.5, label='acc_N_z (vertical)', alpha=0.8)
    axes[0, 1].set_ylabel('Support Acceleration (m/s²)', fontsize=11)
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].set_title('Passenger Experience', fontsize=13, fontweight='bold')
    axes[0, 1].legend(loc='best', fontsize=9)
    axes[0, 1].axhline(0, color='gray', linestyle='--', linewidth=0.8, alpha=0.5)
    
    # Row 2: Jerk
    axes[1, 1].plot(time, jerk, 'm-', linewidth=1.5)
    axes[1, 1].set_ylabel(r'Jerk $\|frac{da_N}/{dt}\|$ (m/s³)', fontsize=11)
    axes[1, 1].grid(True, alpha=0.3)
    
    # Row 3: Coriolis acceleration
    axes[2, 1].plot(time, coriolis, 'c-', linewidth=1.5)
    axes[2, 1].set_ylabel(r'Coriolis Acceleration $a_{Cor}$ (m/s²)', fontsize=11)
    axes[2, 1].set_xlabel('Time (s)', fontsize=11)
    axes[2, 1].grid(True, alpha=0.3)
    axes[2, 1].axhline(0, color='gray', linestyle='--', linewidth=0.8, alpha=0.5)
    
    # Row 4: Passage Position
    axes[3, 1].plot(time, np.rad2deg(theta_s), 'orange', linewidth=1.5)
    axes[3, 1].set_ylabel(r'Passenger Angle $\phi$ (Degree)', fontsize=11)
    axes[3, 1].set_xlabel('Time (s)', fontsize=11)
    axes[3, 1].grid(True, alpha=0.3)
    axes[3, 1].axhline(0, color='gray', linestyle='--', linewidth=0.8, alpha=0.5)

    plt.tight_layout()
    plt.show()