import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

def simulate_pendulum_only_gravity(theta_0, theta_n , L, omega_0, g=10):
    """
    Simulate angular velocity change under gravity only (analytical solution)
    
    Parameters:
    theta_0: initial angle (radians)
    omega_0: initial angular velocity (rad/s)
    g: gravitational acceleration (m/s²)
    L: length of the pendulum (m)
    n_points: number of points in the output arrays

    Returns:
    theta_array: angle values from theta_0 to theta_n (radians)
    omega_array: angular velocity values (rad/s)
    """
    theta_array = np.linspace(theta_0, theta_n, 1000)
    
    # Analytical solution: ω²(θ) = ω₀² + 2g/L (cosθ - cosθ₀)
    omega_array = np.sqrt(
        omega_0**2 + 
        (2 * g / L) * (np.cos(theta_array) - np.cos(-theta_0))
    )
    
    return theta_array, omega_array

def simulate_pendulum_with_force(m, L, theta_0, theta_n, omega_0, tau, g=10.0, mode=1):
    """
    Simulate angular velocity change with torque AND gravity (analytical solution)
    
    Parameters:
    m: mass (kg)
    r: radius (m)
    delta_theta_deg: torque application half-angle (degrees)
    omega_0: initial angular velocity (rad/s)
    tau: applied torque (N·m)
    g: gravitational acceleration (m/s²)

    Returns:
    theta_array: angle values from theta_0 to theta_n (radians)
    omega_array: angular velocity values (rad/s)
    J: angular impulse (N·m·s)
    W: work done by torque (Joules)
    """
    I = m * L**2
    
    theta_array = np.linspace(theta_0, theta_n, 1000)
    
    sgn = 1 if omega_0 >=0 else -1

    # Analytical solution: ω²(θ) = ω₀² + (2/I)[τ(θ-Δθ) + mgr(cosθ - cos(-Δθ))]
    omega_array = np.sqrt(
        omega_0**2 + 
        (2/I) * (tau * np.abs(theta_array - theta_0) * mode + 
                 m * g * L * (np.cos(theta_array) - np.cos(theta_0)))
    ) * sgn
    
    # J = np.trapezoid(tau / omega_array, theta_array)
    J = tau * (omega_array[-1] - omega_0) / (omega_array[-1]/(theta_n - theta_0))
    W = tau * (theta_n - theta_0) 
    
    return theta_array, omega_array, J, W

# 示例调用
if __name__ == "__main__":
    # 典型大摆锤参数
    m = 5000  # kg
    r = 18    # m
    I = m * r**2  # kg·m²   
    delta_theta = np.radians(10)  # 度
    omega_0 = 20 / r# rad/s
    tau = I * 0.8  # N·m (受安全约束 I*alpha_max)

    # Simulate without applied torque
    theta_array_gravity, omega_array_gravity = simulate_pendulum_only_gravity(delta_theta, -delta_theta, r, omega_0)
    theta_array_force, omega_array_force, J_force, W_force = simulate_pendulum_with_force(m, r, -delta_theta, delta_theta, omega_0, tau, mode=-1)
    
    # Plot results
    plt.figure(figsize=(10, 6))
    plt.plot(np.degrees(theta_array_gravity), omega_array_gravity, label='Gravity Only')
    plt.plot(np.degrees(theta_array_force), omega_array_force, label='With Applied Torque')
    plt.title("Angular Velocity vs Angle for Pendulum")
    plt.xlabel("Angle (degrees)")
    plt.ylabel("Angular Velocity (rad/s)")
    plt.legend()
    plt.grid()
    plt.show()


    # Output results
    print(f"{'='*50}")
    print(f"Input Parameters:")
    print(f"  Mass m = {m} kg")
    print(f"  Radius r = {r} m")
    print(f"  Torque region = ±{np.degrees(delta_theta)}°")
    print(f"  Initial angular velocity ω₀ = {omega_0} rad/s")
    print(f"  Applied torque τ = {tau} N·m")
    print(f"{'='*50}")
    print(f"Results:")
    print(f"  Moment of inertia I = {I:.0f} kg·m²")
    print(f"  Final angular velocity ω_final = {omega_array_force[-1]:.3f} rad/s")
    print(f"  Angular velocity change Δω = {omega_array_force[-1] - omega_0:.3f} rad/s")
    print(f"{'='*50}")
    