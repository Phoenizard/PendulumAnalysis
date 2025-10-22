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

def accTangential(theta, g=9.81):
    return -g * np.sin(theta)