from re import L
from networkx import omega
import numpy as np
import sys 
from scipy.integrate import solve_ivp
from matplotlib import pyplot as plt
sys.path.append('..')
from utils import tools

def solve_pendulum(theta_0, g=9.81, L=24.0, n_points=1000):
    """
    求解单摆ODE，直到 theta <= 0
    """
    
    def ode_system(t, y):
        theta, omega = y
        dtheta_dt = omega
        domega_dt = -(g/L) * np.sin(theta)
        return [dtheta_dt, domega_dt]
    
    # 事件函数：当 theta = 0 时触发
    def theta_zero_event(t, y):
        return y[0]  # 返回 theta，当它过零时事件触发
    
    theta_zero_event.terminal = True 
    theta_zero_event.direction = -1
    
    y0 = [theta_0, 0.0]
    
    t_max = 10 * np.sqrt(L/g)
    t_span = (0, t_max)
    t_eval = np.linspace(0, t_max, n_points)
    
    solution = solve_ivp(ode_system, t_span, y0, t_eval=t_eval, 
                         events=theta_zero_event,
                         method='RK45', rtol=1e-8, atol=1e-10,
                         dense_output=True)
    
    # 
    if solution.t_events[0].size > 0:
        t_event = solution.t_events[0][0]
        # 重新生成均匀分布的时间点
        t_new = np.linspace(0, t_event, n_points)
        theta_new = solution.sol(t_new)[0]
        return t_new, theta_new
    else:
        return solution.t, solution.y[0]
    
    # return solution.t, solution.y[0]


# 示例使用
if __name__ == "__main__":
    # 输入：初始角度（60度）
    theta_0 = np.radians(120)
    
    # 求解
    t, theta = solve_pendulum(theta_0)
    print(len(t), len(theta))
    omega_v = tools.AngleToOmega(theta, t)
    v = tools.AngleVelocityToVelocity(omega_v, L=24.0)
    print(len(omega_v), len(v))
    acc_cent = tools.accCentripetal(v, L=24.0)
    acc_tan = tools.accTangential(theta, g=9.81)
    print(len(acc_cent), len(acc_tan))
    # 计算数值上速度的加速度
    a_num_line = np.gradient(v, t) # 验证一致
    print(len(a_num_line))

    # 可视化角度
    # plt.plot(t, np.degrees(theta))
    # plt.title("Theta vs Time for Simple Pendulum")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Angle (degrees)")
    # plt.grid()
    # plt.show()
    # 可视化角速度 + 线速度
    # plt.plot(t, omega_v, label='Angular Velocity (rad/s)')
    # plt.title("Angular Velocity vs Time for Simple Pendulum")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Velocity")
    # plt.legend()
    # plt.grid()
    # plt.show()

    plt.plot(t, v, label='Linear Velocity (m/s)')
    plt.title("Linear Velocity vs Time for Simple Pendulum")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity")
    plt.legend()
    plt.grid()
    plt.show()
    # 可视化加速度
    
    plt.plot(t, acc_cent / 9.81, label='Centripetal Acceleration (m/s²)')
    plt.plot(t, acc_tan / 9.81, label='Tangential Acceleration (m/s²)')
    plt.title("Acceleration vs Time for Simple Pendulum")           
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (g-scale)")
    plt.legend()
    plt.grid()
    plt.show()

