import numpy as np

def AngleVelocityToVelocity(omega, L):
    return omega * L

def AngleToOmega(theta, t):
    """
    通过数值微分将角度转换为角速度
    """
    dtheta_dt = np.gradient(theta, t)
    return dtheta_dt

def accCentripetal(v, L):
    return v**2 / L

def accTangentialFreeMotion(theta, g=9.81):
    return -g * np.sin(theta)

def accTangentialNumical(omega_array, L, t_array):
    """
    通过数值微分将角度转换为切向加速度
    """
    voltage_array = omega_array * L
    a_tangential = np.gradient(voltage_array, t_array)
    return a_tangential

def calculate_time_series(omega_array, theta_array, init_time=0.0, threshold=1e-3):
    """
    Calculate time series from omega and theta arrays, Assuming the initial time is zero.
    
    Parameters:
    omega_array: angular velocity array (rad/s)
    theta_array: angle array (rad)
    threshold: minimum omega to avoid division by zero
    
    Returns:
    time_array: time series starting from t=0 (s)
    duration: total duration (s)
    """
    
    # 确保 omega > 0
    omega_safe = np.maximum(np.abs(omega_array), threshold)
    
    # 计算角度差分
    d_theta = np.diff(theta_array)
    
    # 计算每段的平均角速度
    omega_mid = (omega_safe[:-1] + omega_safe[1:]) / 2
    
    # 计算每段的时间增量
    d_time = np.abs(d_theta) / omega_mid
    
    # 累积求和，初始时间为0
    time_array = np.zeros(len(theta_array))
    time_array[1:] = np.cumsum(d_time)
    
    return time_array + init_time, time_array[-1]


def process_and_validate_data(process_time, process_theta, process_omega, process_velocity):
    """
    验证并处理时间序列数据
    
    参数:
        process_time: 时间数组
        process_theta: 角度数组
        process_omega: 角速度数组
        process_velocity: 速度数组
    
    返回:
        state_dict: 字典，key为状态序号，value包含time, theta, omega, velocity
    """
    # 转换为numpy数组
    process_time = np.asarray(process_time)
    process_theta = np.asarray(process_theta)
    process_omega = np.asarray(process_omega)
    process_velocity = np.asarray(process_velocity)
    
    # 1. 检查时间递减
    time_diff = np.diff(process_time)
    if np.any(time_diff < 0):
        decreasing_indices = np.where(time_diff < 0)[0]
        raise ValueError(f"时间出现递减，位置: {decreasing_indices}, "
                        f"时间值: {process_time[decreasing_indices]} -> {process_time[decreasing_indices + 1]}")
    
    # 2. 处理重复时间点
    duplicate_mask = (time_diff == 0)
    if np.any(duplicate_mask):
        duplicate_indices = np.where(duplicate_mask)[0]
        
        # 检查每个重复点
        indices_to_remove = []
        for idx in duplicate_indices:
            # 检查对应的状态量是否相同
            theta_same = np.isclose(process_theta[idx], process_theta[idx + 1], atol=1e-6)
            if process_theta[idx] == np.pi and process_theta[idx + 1] == -np.pi:
                theta_same = True
            if process_theta[idx] == -np.pi and process_theta[idx + 1] == np.pi:
                theta_same = True
            omega_same = np.isclose(process_omega[idx], process_omega[idx + 1], atol=1e-6)
            velocity_same = np.isclose(process_velocity[idx], process_velocity[idx + 1], atol=1e-5)
            
            if theta_same and omega_same and velocity_same:
                # 全部相同，标记删除第二个点
                indices_to_remove.append(idx + 1)
            else:
                raise ValueError(f"时间重复但状态不一致，位置: {idx}\n"
                               f"时间: {process_time[idx]}\n"
                               f"theta: {process_theta[idx]} vs {process_theta[idx + 1]}\n"
                               f"omega: {process_omega[idx]} vs {process_omega[idx + 1]}\n"
                               f"velocity: {process_velocity[idx]} vs {process_velocity[idx + 1]}")
        
        # 删除重复点
        if indices_to_remove:
            mask = np.ones(len(process_time), dtype=bool)
            mask[indices_to_remove] = False
            process_time = process_time[mask]
            process_theta = process_theta[mask]
            process_omega = process_omega[mask]
            process_velocity = process_velocity[mask]
            print(f"已删除 {len(indices_to_remove)} 个重复时间点")
    
    # 3. 构建字典
    state_dict = {}
    for i in range(len(process_time)):
        state_dict[i] = {
            'time': process_time[i],
            'theta': process_theta[i],
            'omega': process_omega[i],
            'velocity': process_velocity[i]
        }
    
    return state_dict

def calculate_accelerations_from_dict(state_dict, L):
    """
    从state_dict计算向心加速度和切向加速度
    
    参数:
        state_dict: 状态字典，key为索引，value包含time, theta, omega, velocity
        L: 摆长
    
    返回:
        acc_centripetal: 向心加速度数组
        acc_tangential: 切向加速度数组
    """
    # 提取数据为数组
    n_states = len(state_dict)
    t_array = np.array([state_dict[i]['time'] for i in range(n_states)])
    omega_array = np.array([state_dict[i]['omega'] for i in range(n_states)])
    velocity_array = np.array([state_dict[i]['velocity'] for i in range(n_states)])
    
    # 计算加速度
    process_Acc_Centripetal = accCentripetal(velocity_array, L)
    process_Acc_Tangential = accTangentialNumical(omega_array, L, t_array)
    
    return process_Acc_Centripetal, process_Acc_Tangential


def add_accelerations_to_dict(state_dict, L):
    """
    计算加速度并添加到state_dict中
    """
    acc_centripetal, acc_tangential = calculate_accelerations_from_dict(state_dict, L)
    
    for i in range(len(state_dict)):
        state_dict[i]['acc_centripetal'] = acc_centripetal[i]
        state_dict[i]['acc_tangential'] = acc_tangential[i]
    
    return state_dict

def calculate_support_acceleration(data_entry, g=9.81):
    """
    Calculate support force equivalent accelerations in passenger coordinate system.
    
    From force balance: Σa = g + a_N
    Therefore: a_N = Σa - g
    
    Parameters:
    - data_entry: dictionary containing state data
    
    Returns:
    - data_entry: updated dictionary with acc_N_z, acc_N_x, acc_N_y
    
    Raises:
    - ValueError: if required keys are missing
    """
    # 1. Check if required keys exist
    required_keys = ['accCorPas_x', 'accCorPas_y', 'accCorPas_z', 'theta']
    for key in required_keys:
        if key not in data_entry:
            raise ValueError(f"Missing required key: {key}")
    
    # 2. Decompose gravity in passenger coordinate system
    # theta: pendulum angle from vertical (0 at vertical, positive to right, negative to left)
    # theta ∈ [-π, π]
    theta = data_entry['theta']
    
    # Gravity components (L >> r, approximate at cabin center)
    g_x = -g * np.cos(theta)  # x-axis (front): negative when pendulum forward
    g_z = -g * np.sin(theta)  # z-axis (head): negative when pendulum to side
    g_y = 0.0                  # y-axis (lateral): no gravity component
    
    # 3. Calculate support force equivalent accelerations
    # From Σa = g + a_N  =>  a_N = Σa - g
    data_entry['acc_N_z'] = data_entry['accCorPas_z'] - g_z
    data_entry['acc_N_x'] = data_entry['accCorPas_x'] - g_x
    data_entry['acc_N_y'] = data_entry['accCorPas_y'] - g_y
    
    return data_entry

def add_jerk(data_dict):
    """
    Calculate Jerk(t) = ||da_N/dt|| for all data points.
    
    Jerk is calculated from the time derivative of support force acceleration vector.
    
    Parameters:
    - data_dict: dictionary containing all state data (keyed by string indices)
    
    Returns:
    - data_dict: updated dictionary with 'jerk' field added to each entry
    
    Raises:
    - ValueError: if required keys are missing
    """
    # Check if required keys exist
    required_keys = ['time', 'acc_N_x', 'acc_N_y', 'acc_N_z']
    
    # Get the number of data points
    n_points = len(data_dict)
    if n_points == 0:
        raise ValueError("Empty data dictionary")
    
    # Check first entry for required keys
    first_key = list(data_dict.keys())[0]
    for key in required_keys:
        if key not in data_dict[first_key]:
            raise ValueError(f"Missing required key: {key}")
    
    # Extract data into arrays (sorted by key as integer)
    sorted_keys = sorted(data_dict.keys(), key=int)
    times = np.array([data_dict[k]['time'] for k in sorted_keys])
    acc_N_x = np.array([data_dict[k]['acc_N_x'] for k in sorted_keys])
    acc_N_y = np.array([data_dict[k]['acc_N_y'] for k in sorted_keys])
    acc_N_z = np.array([data_dict[k]['acc_N_z'] for k in sorted_keys])
    
    # Calculate time derivatives of acceleration components
    da_x_dt = np.gradient(acc_N_x, times)
    da_y_dt = np.gradient(acc_N_y, times)
    da_z_dt = np.gradient(acc_N_z, times)
    
    # Calculate Jerk magnitude
    jerk_values = np.sqrt(da_x_dt**2 + da_y_dt**2 + da_z_dt**2)
    
    # Add jerk values back to dictionary
    for i, key in enumerate(sorted_keys):
        data_dict[key]['jerk'] = float(jerk_values[i])
    
    return data_dict