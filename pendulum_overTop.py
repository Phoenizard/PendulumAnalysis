import math
import numpy as np
import matplotlib.pyplot as plt
from component import ForceSinglePeriod as fsp
from component import GravityMotion as gm
from component import break_down as bd
from utils import tools
from utils import plot_status
import json

# 所有函数输入角度都为radius

if __name__ == "__main__":
    # Parameters Setting
    T = 120 # Running Time in seconds
    T_Break = 40 # Slow Down Time in seconds
    m = 5000 # mass in kg
    L = 18.0 # length in m
    g = 9.81 # gravitational acceleration in m/s²

    target_velocity = 0.01 # target linear velocity in the Top Point in m/s
    target_omega = target_velocity / L # target angular velocity in rad/s

    theta_init = np.radians(20)
    omega_init = 0 # initial angular velocity in rad/s
    I = m * L ** 2  # moment of inertia in kg·m²
    tau_max = I * 3 * 9.81 / L# applied torque in N·m under safety constraint I*alpha_max

    # Final Energy at the top point
    E_top = 0.5 * m * target_velocity ** 2 + m * g * 2 * L
    E_needed = E_top - m * g * L *(1 - np.cos(theta_init))

    thetaN = np.radians(180) # target angle in radians
    DeltaAngle = np.radians(10) # torque application half-angle in radians
    SingleEnergyInputMax = tau_max * (2 * DeltaAngle) # maximum energy input per half period
    # Calculate required number of periods

    n_periods = max(int(np.ceil(E_needed / SingleEnergyInputMax)), 6) # at least 8 periods
    if n_periods % 2 != 0:
        n_periods += 1  # make it even number
    print(f"设定: {n_periods}次, 总能量需求: {E_needed:.2f} J，期望最终角速度: {target_omega:.2f} rad/s")
    tau = E_needed / (n_periods * 2 * DeltaAngle)  # required constant torque per half period
    print(f"每次加速所需力矩: {tau:.2f} N·m, 小于最大允许力矩 {tau_max:.2f} N·m")

    theta_n_max = [theta_init]
    theta_n_mid = []
    theta_arrays = []
    omega_arrays = []
    time_arrays = []
    # Simulate Angular Velocity with applied torque
    flagPeriod = 1
    now_omega = omega_init
    now_theta = theta_init
    now_time = 0.0


    # ========================================================================
    # 冲顶阶段
    while True: 
        tmp_omega_array_set, tmp_theta_array_set, tmp_time_array_set = [], [], []
        print(f"--- Simulating Period {flagPeriod} ---")
        # =========================================================================
        # 从最高点自由释放，执行free_down，正角度变小至加速段起点
        theta_array_free_down_1, omega_array_free_down_1, now_omega = gm.free_down(
            omega_init=0.0, theta_init=theta_n_max[-1], 
            DeltaAngle=DeltaAngle, L=L, g=g
        )
        now_theta = DeltaAngle
        time_free_down_1, duration_1 = tools.calculate_time_series(omega_array_free_down_1, theta_array_free_down_1, init_time=now_time)
        now_time += duration_1

        tmp_omega_array_set.append(omega_array_free_down_1)
        tmp_theta_array_set.append(theta_array_free_down_1)
        tmp_time_array_set.append(time_free_down_1)
        print(f"Period {flagPeriod} Fall from {np.degrees(theta_n_max[-1]):.2f}° to {np.degrees(now_theta):.2f}°")
        print(f"Omega Become: {now_omega:.4f} rad/s")
        print("Applying Torque...")
        # =========================================================================

        # =========================================================================
        # 施加力矩段，从正角度到负角度
        theta_array_force_1, omega_array_force_1, J_1, W_1 = fsp.simulate_pendulum_with_force(
            m=m, L=L, theta_0=now_theta, theta_n=-DeltaAngle, omega_0=now_omega, 
            tau=tau, g=g
        )
        now_omega = omega_array_force_1[-1]
        now_theta = -DeltaAngle
        time_force_1, duration_2 = tools.calculate_time_series(omega_array_force_1, theta_array_force_1, init_time=now_time)
        now_time += duration_2
        tmp_omega_array_set.append(omega_array_force_1)
        tmp_theta_array_set.append(theta_array_force_1)
        tmp_time_array_set.append(time_force_1)
        print(f"Period {flagPeriod} After Torque to {np.degrees(now_theta):.2f}°, Omega Become: {now_omega:.4f} rad/s")
        # =========================================================================

        # =========================================================================
        # 继续自由运动段，从负角度到最高点
        theta_array_free_up_1, omega_array_free_up_1, now_theta, isReach= gm.free_up(
            omega_init=now_omega, 
            theta_init=now_theta, 
            L=L, g=g
        )

        now_omega = 0.0
        theta_n_mid.append(now_theta)
        time_free_up_1, duration_3 = tools.calculate_time_series(
            omega_array_free_up_1,
            theta_array_free_up_1,
            init_time=now_time
        )
        now_time += duration_3
        tmp_omega_array_set.append(omega_array_free_up_1)
        tmp_theta_array_set.append(theta_array_free_up_1)
        tmp_time_array_set.append(time_free_up_1)

        if isReach:
            now_omega = omega_array_force_1[-1]
            # Test whether these array's last value equals to first value of next array
            for i in range(len(tmp_omega_array_set)-1):
                if not np.isclose(tmp_omega_array_set[i][-1], tmp_omega_array_set[i+1][0], atol=1e-6):
                    raise ValueError("Omega arrays do not connect properly!")
                if not np.isclose(tmp_theta_array_set[i][-1], tmp_theta_array_set[i+1][0], atol=1e-6):
                    raise ValueError("Theta arrays do not connect properly!")
                if not np.isclose(tmp_time_array_set[i][-1], tmp_time_array_set[i+1][0], atol=1e-6):
                    raise ValueError("Time arrays do not connect properly!")
            # Concatenate temporary arrays
            omega_arrays.append(np.concatenate(tmp_omega_array_set))
            theta_arrays.append(np.concatenate(tmp_theta_array_set))
            time_arrays.append(np.concatenate(tmp_time_array_set))
            print(f"Reached the top at {np.degrees(now_theta):.2f}°, Simulation Ends.")
            break

        print(f"Period {flagPeriod} Rise to {np.degrees(now_theta):.2f}°")
        print("---- Half Period Completed ----")
        # =========================================================================

        # =========================================================================
        # 从最高点自由释放，执行free_down，负角度变小至加速段起点
        theta_array_free_down_2, omega_array_free_down_2, now_omega = gm.free_down(
            omega_init=0.0, 
            theta_init=theta_n_mid[-1], 
            DeltaAngle=-DeltaAngle, 
            L=L, 
            g=g
        )
        now_theta = -DeltaAngle
        time_free_down_2, duration_4 = tools.calculate_time_series(
            omega_array_free_down_2,
            theta_array_free_down_2,
            init_time=now_time
        )
        now_time += duration_4
        tmp_omega_array_set.append(omega_array_free_down_2)
        tmp_theta_array_set.append(theta_array_free_down_2)
        tmp_time_array_set.append(time_free_down_2)
        print(f"Period {flagPeriod} Fall to {np.degrees(now_theta):.2f}°, Omega Become: {now_omega:.4f} rad/s")
        
        # =========================================================================
        print("Applying Torque...")
        # 施加力矩段，从负角度到正角度
        theta_array_force_2, omega_array_force_2, J_2, W_2 = fsp.simulate_pendulum_with_force(
            m=m, 
            L=L, 
            theta_0=now_theta, 
            theta_n=DeltaAngle, 
            omega_0=now_omega, 
            tau=tau, 
            g=g
        )
        now_omega = omega_array_force_2[-1]
        now_theta = DeltaAngle
        time_force_2, duration_5 = tools.calculate_time_series(
            omega_array_force_2,
            theta_array_force_2,
            init_time=now_time
        )
        now_time += duration_5
        tmp_omega_array_set.append(omega_array_force_2)
        tmp_theta_array_set.append(theta_array_force_2)
        tmp_time_array_set.append(time_force_2)
        print(f"Period {flagPeriod} After Torque to {np.degrees(now_theta):.2f}°, Omega Become: {now_omega:.4f} rad/s")
        # =========================================================================

        # =========================================================================
        # 继续自由运动段，从正角度到最高点
        theta_array_free_up_2, omega_array_free_up_2, now_theta, isReach = gm.free_up(
            omega_init=now_omega, 
            theta_init=now_theta, 
            L=L, g=g
        )
        if isReach:
            print(now_omega, now_theta)
            print(theta_array_free_up_2[-1])
        time_free_up_2, duration_6 = tools.calculate_time_series(
            omega_array_free_up_2,
            theta_array_free_up_2,
            init_time=now_time
        )
        now_time += duration_6
        tmp_omega_array_set.append(omega_array_free_up_2)
        tmp_theta_array_set.append(theta_array_free_up_2)
        tmp_time_array_set.append(time_free_up_2)

        if isReach:
            now_omega = omega_array_force_1[-1]
            # Test whether these array's last value equals to first value of next array
            for i in range(len(tmp_omega_array_set)-1):
                if not np.isclose(tmp_omega_array_set[i][-1], tmp_omega_array_set[i+1][0], atol=1e-6):
                    raise ValueError("Omega arrays do not connect properly!")
                if not np.isclose(tmp_theta_array_set[i][-1], tmp_theta_array_set[i+1][0], atol=1e-6):
                    raise ValueError("Theta arrays do not connect properly!")
                if not np.isclose(tmp_time_array_set[i][-1], tmp_time_array_set[i+1][0], atol=1e-6):
                    raise ValueError("Time arrays do not connect properly!")
            # Concatenate temporary arrays
            omega_arrays.append(np.concatenate(tmp_omega_array_set))
            theta_arrays.append(np.concatenate(tmp_theta_array_set))
            time_arrays.append(np.concatenate(tmp_time_array_set))
            print(f"Reached the top at {np.degrees(now_theta):.2f}°, Simulation Ends1.")
            break

        print(f"Period {flagPeriod} Rise to {np.degrees(now_theta):.2f}°")
        now_omega = 0.0
        theta_n_max.append(now_theta)
        # 以上完成一个完整周期
        # 记录数据
        #========================================================================
        # Test whether these array's last value equals to first value of next array
        for i in range(len(tmp_omega_array_set)-1):
            if not np.isclose(tmp_omega_array_set[i][-1], tmp_omega_array_set[i+1][0], atol=1e-6):
                raise ValueError("Omega arrays do not connect properly!")
            if not np.isclose(tmp_theta_array_set[i][-1], tmp_theta_array_set[i+1][0], atol=1e-6):
                raise ValueError("Theta arrays do not connect properly!")
            if not np.isclose(tmp_time_array_set[i][-1], tmp_time_array_set[i+1][0], atol=1e-6):
                raise ValueError("Time arrays do not connect properly!")
        #========================================================================

        time_arrays.append(np.concatenate(tmp_time_array_set))
        theta_arrays.append(np.concatenate(tmp_theta_array_set))
        omega_arrays.append(np.concatenate(tmp_omega_array_set))
        print(f"Period {flagPeriod}: from {np.degrees(theta_n_max[-2]):.2f}° to {np.degrees(theta_n_max[-1]):.2f}°")
        print(f"Time Elapsed: {now_time} s")
        flagPeriod += 1
        # break

    # 合并所有周期数据
    process_time = np.concatenate(time_arrays)
    process_theta = np.concatenate(theta_arrays)
    process_omega = np.concatenate(omega_arrays)

    now_time, now_omega, now_theta = process_time[-1], process_omega[-1], process_theta[-1]
    print(f"冲顶阶段数据状态: {now_time:.6f} s, {now_omega:.6f} rad/s, {np.degrees(now_theta):.6f}°")
    if now_time >= T - T_Break:
        print("No Stable Top State Simulation Needed, Proceeding to Break Down Phase.")
    # =======================================================
    # 稳定状态，保持在顶点以及最高速度，在不受阻力下，只受重力作用即可
    # 我们默认速度方向为正，如果为负，标记，并且在最后取反即可
    isReverse = False if now_omega > 0 else True
    print("\nStable Top State Simulation:")
    # Analysis Single Period to obtain the Time and Information
    theta_array_stable, omega_array_stable = gm.full_circular_motion(omega_0=np.abs(now_omega),L=L,g=g)
    time_array_stable, duration_stable = tools.calculate_time_series(
        omega_array_stable, theta_array_stable, init_time=0.0
    )
    print(f"Single Stable Period Duration: {duration_stable:.3f} s")
    if isReverse:
        omega_array_stable = -omega_array_stable
    # Find maximum full period within remaining time
    remaining_time = T - now_time - T_Break
    # n_stable_periods = int(remaining_time // duration_stable) 
    n_stable_periods = 2
    print(f"Remaining Time: {remaining_time:.3f} s, Can Fit {n_stable_periods} Stable Periods, Remaining Time After Stable Periods: {T- now_time - n_stable_periods * duration_stable:.3f} s to Break Down")
    T_b = T - now_time - n_stable_periods * duration_stable
    # Append Stable Periods Data
    if n_stable_periods > 0:
        stable_time_arrays, stable_theta_arrays, stable_omega_arrays = [], [], []
        
        # 第一个周期
        stable_time_arrays.append(time_array_stable + now_time)
        stable_theta_arrays.append(theta_array_stable)
        stable_omega_arrays.append(omega_array_stable)
        
        # 后续周期
        for _ in range(n_stable_periods - 1):
            # 基于上一个周期的最后时间
            last_time = stable_time_arrays[-1][-1]
            new_time_array = time_array_stable + last_time  # 使用原始模板
            
            # 跳过第一个点（与上一周期末尾重复）
            stable_time_arrays.append(new_time_array[1:])
            stable_theta_arrays.append(theta_array_stable[1:])
            stable_omega_arrays.append(omega_array_stable[1:])
            # Concatenate Stable Period Data=
    if isReverse:
        stable_omega_arrays = [-arr for arr in stable_omega_arrays]
    stable_process_time = np.concatenate(stable_time_arrays)
    stable_process_theta = np.concatenate(stable_theta_arrays)
    stable_process_omega = np.concatenate(stable_omega_arrays)
    stable_process_theta[-1] = np.pi if isReverse else -np.pi  # Ensure exact top point

    process_time = np.concatenate((process_time, stable_process_time))
    process_theta = np.concatenate((process_theta, stable_process_theta))
    process_omega = np.concatenate((process_omega, stable_process_omega))

    print(f"Stable Periods Data Points: {len(stable_process_time)}")
    print(f"Total Time after Stable Periods: {stable_process_time[-1]:.3f} s")
    print(f"Now Omega after Stable Periods: {stable_process_omega[-1]:.3f} rad/s")
    print(f"Now Theta after Stable Periods: {np.degrees(stable_process_theta[-1]):.3f}°")
    now_time = stable_process_time[-1]
    now_omega = stable_process_omega[-1]
    now_theta = stable_process_theta[-1]
    # =======================================================
    # Break Down Phase
    print(f"\nBreak Down Phase Simulation: Starting from Stable Top State with ω = {now_omega:.3f} rad/s")
    theta_brake = np.radians(20)  
    theta_motor = np.radians(10)
    n_half_cycles = 12  # 严格要求：第8个半周期时刹停
    T_max = T_b
    print("Optimal Brake Torque Calculation...")
    tau_optimal = bd.find_minimum_brake_torque(theta_brake, theta_motor, n_half_cycles, m, L, T_max, g)
    print(f"Optimal Brake Torque Found: {tau_optimal:.2f} N·m")
    success, controller, t_array_break, theta_array_break, omega_array_break, tau_array = bd.simulate_pendulum(tau_optimal, theta_brake, theta_motor, n_half_cycles, m, L, T_max, g, dt=0.01, verbose=True)
    now_time += 0.01
    now_omega = 0.01
    time_array_break = t_array_break + now_time
    for i in range(10):
        if theta_array_break[i] > 0:
            theta_array_break[i] = - theta_array_break[i]
    # 假设用0.01s系统制动然后设置试探速度为0.01

    if isReverse:
        omega_array_break = - omega_array_break
        theta_array_break = - theta_array_break
    if not np.isclose(theta_array_break[0], now_theta, atol=1e-6):
        raise ValueError("Theta arrays do not connect properly at Break Down!")
    if not np.isclose(omega_array_break[0], now_omega, atol=1e-6):
        raise ValueError("Omega arrays do not connect properly at Break Down!")
    if not np.isclose(time_array_break[0], now_time, atol=1e-6):
        raise ValueError("Time arrays do not connect properly at Break Down!")

    process_time = np.concatenate((process_time, time_array_break))
    process_theta = np.concatenate((process_theta, theta_array_break))
    process_omega = np.concatenate((process_omega, omega_array_break))
    process_velocity = tools.AngleVelocityToVelocity(process_omega, L)

    try:
        state_dict = tools.process_and_validate_data(process_time, process_theta, process_omega, process_velocity)
        print(f"成功处理 {len(state_dict)} 个状态点")
    except ValueError as e:
        print(f"数据验证错误: {e}")

    state_dict = tools.add_accelerations_to_dict(state_dict, L)
    # plot_status.plot_dict(state_dict)

    final_state = state_dict[len(state_dict) - 1]
    print("\nFinal State:")
    print(f"  Time: {final_state['time']:.3f} s")
    print(f"  Angle θ: {np.degrees(final_state['theta']):.3f}°")
    print(f"  Angular Velocity ω: {final_state['omega']:.3f} rad/s")
    print(f"  Velocity v: {final_state['velocity']:.3f} m/s")
    print(f"  Centripetal Acceleration: {final_state['acc_centripetal']:.3f} m/s²")
    print(f"  Tangential Acceleration: {final_state['acc_tangential']:.3f} m/s²")


    with open('state_s120.json', 'w', encoding='utf-8') as f:
        json.dump(state_dict, f, indent=4)

