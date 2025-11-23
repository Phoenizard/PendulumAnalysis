import numpy as np
import matplotlib.pyplot as plt

def free_up(omega_init, theta_init, L, g=10.0):
    """
    Calculate free swing upward under gravity only until angular velocity reaches zero

    If theta_init > 0, then sgn = 1, from right to right
    If theta_init < 0, then sgn = -1, from left to left
    
    Parameters:
    omega_init: initial angular velocity (rad/s), positive = counterclockwise
    theta_init: initial angle (rad), starting position
    m: mass (kg)
    L: radius (m)
    g: gravitational acceleration (m/s²)
    
    Returns:
    theta_array: angle values from theta_init to theta_max (rad)
    omega_array: angular velocity values (rad/s)
    theta_max: maximum angle reached when omega = 0 (rad)
    """
    sgn = 1 if theta_init >= 0 else -1
    cos_theta_max = - omega_init ** 2 * L / (2 * g) + np.cos(theta_init)
    # Check if physically possible
    if cos_theta_max <= -1:
        isReach = True
        theta_max = np.pi * sgn
        # raise ValueError("Initial conditions lead to impossible swing (cos(theta_max) < -1).")  
    else:
        isReach = False
        theta_max = np.arccos(cos_theta_max) * sgn
    
    # Create angle array
    theta_array = np.linspace(theta_init, theta_max, 1000)
    
    # Analytical solution for omega(theta) under gravity only
    # ω²(θ) = ω_init² + (2mgr/I)[cos(θ) - cos(θ_init)]
    omega_squared = omega_init**2 + (2 * g / L) * (np.cos(theta_array) - np.cos(theta_init))
    
    # Handle numerical errors near theta_max
    omega_squared = np.maximum(omega_squared, 0)
    omega_array = np.sqrt(omega_squared) * sgn
    
    # Calculate time duration
    # dt = dθ/ω
    return theta_array, omega_array, theta_max, isReach


def free_down(omega_init, theta_init, DeltaAngle,L, g=10.0):
    """
    Calculate free swing downward under gravity until reach to DeltaAngle

    If theta_init > 0, then sgn = -1, from right to left
    If theta_init < 0, then sgn = 1, from left to right

    Parameters:
    omega_init: initial angular velocity (rad/s), positive = counterclockwise
    theta_init: initial angle (rad), starting position
    DeltaAngle: target angle (rad), ending position
    m: mass (kg)
    L: radius (m)
    g: gravitational acceleration (m/s²)

    Returns:
    theta_array: angle values from theta_init to DeltaAngle (rad)
    omega_array: angular velocity values (rad/s)
    final_omega: angular velocity at DeltaAngle (rad/s)
    """
    sgn = -1 if theta_init >= 0 else 1
    theta_array = np.linspace(theta_init, DeltaAngle, 1000)
    
    # Analytical solution for omega(theta) under gravity only
    # ω²(θ) = ω_init² + (2mgr/I)[cos(θ) - cos(θ_init)]
    omega_squared = omega_init**2 + (2 * g / L) * (np.cos(theta_array) - np.cos(theta_init))
    
    # Handle numerical errors
    omega_squared = np.maximum(omega_squared, 0)
    omega_array = np.sqrt(omega_squared) * sgn

    return theta_array, omega_array, omega_array[-1]


def full_circular_motion(omega_0, L, g=10.0):
    """
    Note: 默认从最高点向左下方运动 (theta_0 = π, 逆时针方向为正)即从-pi到pi

    Parameters:
    omega_0: initial angular velocity (rad/s)
    theta_0: initial angle (rad)
    L: radius (m)
    g: gravitational acceleration (m/s²)

    Returns:
    theta_array: angle values from theta_0 to theta_n (radians)
    omega_array: angular velocity values (rad/s)
    """
    theta_0, theta_n = -np.pi, np.pi
    theta_array = np.linspace(theta_0, theta_n, 1000)
    
    # Analytical solution: ω²(θ) = ω₀² + 2g/L (cosθ - cosθ₀)
    omega_array = np.sqrt(
        omega_0**2 + 
        (2 * g / L) * (np.cos(theta_array) - np.cos(theta_0))
    )
    return theta_array, omega_array


# Example usage
if __name__ == "__main__":
    # Test parameters
    L = 20    # m
    omega_init = 10 / L  # rad/s (negative for clockwise)
    theta_init = np.deg2rad(180)  # radians
    DeltaAngle = np.radians(10)
    # theta_array, omega_array, current_omega, _ = free_up(
    #     omega_init, theta_init, L, g=9.81)
    theta_array, omega_array = full_circular_motion(omega_init, L, g=9.81)
    # print(f"Initial conditions:")
    # print(f"  θ_init = {np.degrees(theta_init):.1f}°")
    # print(f"  ω_init = {omega_init:.3f} rad/s")
    # print(f"\nResults:")
    # print(f"  θ_max = {np.degrees(theta_max):.1f}°")
    # print(f"  Time duration = {time_duration:.3f} s")
    
    # # Plot

    


    plt.figure(figsize=(10, 4))
    
    plt.plot(np.degrees(theta_array), omega_array, 'b-', linewidth=2)
    plt.xlabel('Angle θ (degrees)', fontsize=12)
    plt.ylabel('Angular Velocity ω (rad/s)', fontsize=12)
    plt.title('Free Swing Upward (Gravity Only)', fontsize=14)
    plt.grid(True, alpha=0.3)

    plt.show()