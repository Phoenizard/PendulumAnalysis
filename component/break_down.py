import numpy as np
import matplotlib.pyplot as plt

class HalfCycleBrakeController:
    def __init__(self, theta_brake, theta_motor, n_half_cycles, tau_brake, m, L, g=9.81):
        """
        半周期刹车控制器
        
        参数:
            theta_brake: 判定刹停的角度阈值 (rad)
            theta_motor: 电机施加力矩的区间 [-theta_motor, +theta_motor] (rad)
            n_half_cycles: 严格要求的半周期数
            tau_brake: 恒定刹车力矩 (N·m)
            m: 摆锤质量 (kg)
            L: 摆长 (m)
        """
        self.theta_brake = theta_brake
        self.theta_motor = theta_motor
        self.n_half_cycles = n_half_cycles
        self.tau_brake = tau_brake
        self.m = m
        self.L = L
        self.g = g
        self.I = m * L**2
        
        # 半周期追踪
        self.half_cycle_count = 0
        self.peak_amplitudes = []
        self.last_omega_sign = 0
        self.current_peak_theta = 0
        
        # 刹车状态
        self.is_braking = False
        self.should_stop = False
        
    def calculate_brake_torque(self, theta, omega, t):
        """
        计算刹车力矩
        """
        theta_normalized = np.arctan2(np.sin(theta), np.cos(theta))
        current_omega_sign = np.sign(omega) if abs(omega) > 1e-4 else 0
        
        # 检测峰值（速度换向点）
        if self.last_omega_sign != 0 and current_omega_sign != 0:
            if current_omega_sign != self.last_omega_sign:
                # 换向点 = 一个半周期结束
                peak_amplitude = abs(self.current_peak_theta)
                self.peak_amplitudes.append(peak_amplitude)
                self.half_cycle_count += 1
                
                # 检查是否达到目标半周期数
                if self.half_cycle_count >= self.n_half_cycles:
                    if peak_amplitude < self.theta_brake:
                        self.should_stop = True
                    else:
                        self.should_stop = True  # 也要停止，但标记为失败
                
                self.current_peak_theta = 0
        
        # 更新当前峰值
        if abs(theta_normalized) > abs(self.current_peak_theta):
            self.current_peak_theta = theta_normalized
        
        self.last_omega_sign = current_omega_sign
        
        # 如果已经检测到应该停止，不再施加刹车
        if self.should_stop:
            return 0.0
        
        # 刹车逻辑：在电机区间 [-theta_motor, +theta_motor] 内施加恒定反向力矩
        in_motor_zone = abs(theta_normalized) <= self.theta_motor
        
        if in_motor_zone and abs(omega) > 1e-4:
            if not self.is_braking:
                self.is_braking = True
                
            
            # 施加反向恒定力矩
            tau = -np.sign(omega) * self.tau_brake
            return tau
        else:
            if self.is_braking:
                pass
            self.is_braking = False
            return 0.0
    
    def is_successful(self, T_max, t_final):
        """
        判断是否成功：
        1. 恰好 n_half_cycles 个半周期
        2. 第 n 个半周期的摆幅 < theta_brake
        3. 在 T_max 时间内完成
        """
        if t_final > T_max:
            # print(f"✗ 失败：超时 ({t_final:.2f}s > {T_max:.2f}s)")
            return False
        
        if self.half_cycle_count != self.n_half_cycles:
            # print(f"✗ 失败：半周期数不匹配 ({self.half_cycle_count} != {self.n_half_cycles})")
            return False
        
        if len(self.peak_amplitudes) < self.n_half_cycles:
            # print(f"✗ 失败：未完成足够的半周期")
            return False
        
        last_amplitude = self.peak_amplitudes[self.n_half_cycles - 1]
        
        if last_amplitude >= self.theta_brake:
            # print(f"✗ 失败：第 {self.n_half_cycles} 个半周期摆幅 "
            #       f"{np.degrees(last_amplitude):.2f}° >= {np.degrees(self.theta_brake):.2f}°")
            return False
        
        # print(f"✓ 成功：第 {self.n_half_cycles} 个半周期摆幅 "
        #       f"{np.degrees(last_amplitude):.2f}° < {np.degrees(self.theta_brake):.2f}°, "
        #       f"用时 {t_final:.2f}s")
        return True


def simulate_pendulum(tau_brake, theta_brake, theta_motor, n_half_cycles, m, L, 
                     T_max, g=9.81, theta0=np.pi, omega0=0.01, dt=0.01, verbose=True):
    """
    单摆仿真
    
    参数:
        T_max: 最大允许时间 (s)
        verbose: 是否打印详细信息
    
    返回:
        success: 是否成功
        controller: 控制器对象
        t_array, theta_array, omega_array, tau_array: 状态数组
    """
    controller = HalfCycleBrakeController(theta_brake, theta_motor, n_half_cycles, 
                                         tau_brake, m, L, g)
    
    # 状态数组
    t_array = [0.0]
    theta_array = [theta0]
    omega_array = [omega0]
    tau_array = [0.0]
    
    # 仿真循环
    t = 0.0
    theta = theta0
    omega = omega0
    
    while t < T_max:
        theta_normalized = np.arctan2(np.sin(theta), np.cos(theta))
        
        # 计算刹车力矩
        tau_brake_applied = controller.calculate_brake_torque(theta, omega, t)
        
        # 动力学方程
        I = m * L**2
        alpha = (-m * g * L * np.sin(theta) + tau_brake_applied) / I
        
        # 更新状态
        omega += alpha * dt
        theta += omega * dt
        t += dt
        
        # 记录
        t_array.append(t)
        theta_array.append(theta_normalized)
        omega_array.append(omega)
        tau_array.append(tau_brake_applied)
        
        # 停止条件：达到目标半周期数且速度接近0
        if controller.should_stop and abs(omega) < 1e-3:
            # if verbose:
            #     print(f"\n仿真结束")
            #     print(f"  最终位置: θ = {np.degrees(theta_normalized):.2f}°")
            #     print(f"  最终速度: ω = {omega:.6f} rad/s")
            #     print(f"  总半周期: {controller.half_cycle_count}")
            #     print(f"  用时: {t:.2f}s")
            break
    else:
        # 超时
        # if verbose:
        #     print(f"\n仿真超时 (t = {T_max:.2f}s)")
        pass
    
    # 判断成功
    t_final = t
    success = controller.is_successful(T_max, t_final)
    
    return success, controller, np.array(t_array), np.array(theta_array), np.array(omega_array), np.array(tau_array)


def find_minimum_brake_torque(theta_brake, theta_motor, n_half_cycles, m, L, T_max, g=9.81):
    """
    优化：找到最小的刹车力矩
    
    使用二分搜索
    """
    print("=" * 70)
    print("开始优化：寻找最小刹车力矩")
    print("=" * 70)
    print(f"约束条件：")
    print(f"  - 严格要求半周期数: {n_half_cycles}")
    print(f"  - 第 {n_half_cycles} 个半周期摆幅 < {np.degrees(theta_brake):.1f}°")
    print(f"  - 电机区间: ±{np.degrees(theta_motor):.1f}°")
    print(f"  - 最大时间: {T_max:.1f}s")
    print()
    
    # 估算力矩范围
    tau_min = 10.0  # N·m (下界)
    
    # 上界：基于能量估算
    E_max = m * g * L * 2
    tau_max = E_max / (2 * theta_motor) * 3
    
    print(f"初始搜索范围: [{tau_min:.0f}, {tau_max:.0f}] N·m\n")
    
    # 二分搜索
    tolerance = 100.0  # N·m
    iteration = 0
    max_iterations = 25
    
    # 首先确保上界可行
    success_upper, _, _, _, _, _ = simulate_pendulum(
        tau_max, theta_brake, theta_motor, n_half_cycles, m, L, T_max,
        g, dt=0.01, verbose=False
    )
    
    if not success_upper:
        tau_max *= 3
    else:
        pass
    
    while tau_max - tau_min > tolerance and iteration < max_iterations:
        iteration += 1
        tau_mid = (tau_min + tau_max) / 2
        
        # 测试当前力矩
        success, controller, t, theta, omega, tau = simulate_pendulum(
            tau_mid, theta_brake, theta_motor, n_half_cycles, m, L, T_max,
            g, dt=0.01, verbose=True
        )
        
        if success:
            tau_max = tau_mid
        else:
            tau_min = tau_mid
    
    tau_optimal = tau_max
    
    print("\n" + "=" * 70)
    print(f"优化完成！")
    print(f"最小刹车力矩: {tau_optimal:.0f} N·m")
    print(f"迭代次数: {iteration}")
    print("=" * 70)
    
    return tau_optimal


def run_with_optimal_torque(tau_optimal, theta_brake, theta_motor, n_half_cycles, 
                           m, L, T_max, g=9.81):
    """
    使用最优力矩运行完整仿真并绘图
    """
    print("\n" + "=" * 70)
    print(f"使用最优力矩 {tau_optimal:.0f} N·m 运行最终仿真")
    print("=" * 70 + "\n")
    
    success, controller, t, theta, omega, tau = simulate_pendulum(
        tau_optimal, theta_brake, theta_motor, n_half_cycles, m, L, T_max,
        g, dt=0.01, verbose=True
    )
    
    # 绘图
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    
    # 角度
    axes[0].plot(t, np.degrees(theta), 'b-', linewidth=1.5)
    axes[0].axhline(np.degrees(theta_brake), color='orange', linestyle='--', 
                   alpha=0.5, label=f'Brake threshold (±{np.degrees(theta_brake):.1f}°)')
    axes[0].axhline(-np.degrees(theta_brake), color='orange', linestyle='--', alpha=0.5)
    axes[0].axhline(np.degrees(theta_motor), color='r', linestyle=':', 
                   alpha=0.5, label=f'Motor zone (±{np.degrees(theta_motor):.1f}°)')
    axes[0].axhline(-np.degrees(theta_motor), color='r', linestyle=':', alpha=0.5)
    axes[0].set_ylabel('Angle (deg)', fontsize=12)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[0].set_title(f'Optimal Brake Control - {n_half_cycles} Half-Cycles (τ = {tau_optimal:.0f} N·m)', 
                     fontsize=13, fontweight='bold')
    
    # 角速度
    axes[1].plot(t, omega, 'g-', linewidth=1.5)
    axes[1].axhline(0, color='k', linestyle='--', alpha=0.3)
    axes[1].set_ylabel('Angular Velocity (rad/s)', fontsize=12)
    axes[1].grid(True, alpha=0.3)
    
    # 刹车力矩
    axes[2].plot(t, tau / 1e6, 'r-', linewidth=1.5)
    axes[2].set_ylabel('Brake Torque (MN·m)', fontsize=12)
    axes[2].set_xlabel('Time (s)', fontsize=12)
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # 打印摆幅历史
    print("\n摆幅历史:")
    for i, amp in enumerate(controller.peak_amplitudes):
        marker = " ← 目标" if i == n_half_cycles - 1 else ""
        print(f"  半周期 {i+1}: {np.degrees(amp):.2f}°{marker}")


# ============ 主程序 ============
if __name__ == "__main__":
    # 参数设置
    m = 5000.0  # kg
    L = 18.0  # m
    g = 9.81
    theta_brake = np.radians(20)  # 刹停阈值 ±20°
    theta_motor = np.radians(10)  # 电机区间 ±15°
    n_half_cycles = 8  # 严格要求：第8个半周期时刹停
    T_max = 120.0  # 最大时间 120秒
    
    # 优化：找到最小力矩
    tau_optimal = find_minimum_brake_torque(theta_brake, theta_motor, n_half_cycles, 
                                           m, L, T_max, g)
    
    # 使用最优力矩运行并绘图
    # run_with_optimal_torque(tau_optimal, theta_brake, theta_motor, n_half_cycles, 
    #                        m, L, T_max, g)
    
    # 使用最优力矩运行最终仿真
    success, controller, t_array, theta_array, omega_array, tau_array = simulate_pendulum(
        tau_optimal, theta_brake, theta_motor, n_half_cycles, m, L, T_max,
        g, dt=0.01, verbose=True
    )
    
    # ========== 导出三个数组 ==========
    time_array = t_array
    theta_array = theta_array  # 已经是归一化的角度
    omega_array = omega_array
    
    print(f"\n导出数组:")
    print(f"  time_array: {len(time_array)} 个数据点")
    print(f"  theta_array: {len(theta_array)} 个数据点")
    print(f"  omega_array: {len(omega_array)} 个数据点")
    print(success)
    print(time_array)
