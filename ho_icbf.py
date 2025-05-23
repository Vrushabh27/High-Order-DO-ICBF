# -*- coding: utf-8 -*-

import numpy as np
import sympy as sp
from scipy.optimize import minimize
import matplotlib.pyplot as plt

# Enable LaTeX rendering for plots\ nplt.rcParams['text.usetex'] = True

def circle_patch(x, y, r, color='g', ax=None):
    """
    Draws a filled circle patch on the given axes.
    """
    if ax is None:
        ax = plt.gca()
    circle = plt.Circle((x, y), r, color=color, fill=True, alpha=0.3)
    ax.add_patch(circle)
    return circle


def solve_qp(H, F, A, b):
    """
    Solves the quadratic program:
        minimize (1/2) x^T H x + F^T x
        subject to A x <= b
    using scipy.optimize.minimize (SLSQP).
    Returns the solution vector x.
    """
    def obj(x):
        return 0.5 * x.T.dot(H).dot(x) + F.dot(x)
    cons = ({'type': 'ineq', 'fun': lambda x: b - A.dot(x)})
    x0 = np.zeros(H.shape[1])
    res = minimize(obj, x0, constraints=cons, options={'disp': False})
    return res.x


def main():
    # Simulation parameters
    dt = 0.1
    num_steps = 700

    # Circle path
    radius = 10.0
    center = np.array([0.0, 10.0])
    theta = np.arange(0, 2*np.pi, 0.1)
    circle_x = center[0] + radius * np.cos(theta)
    circle_y = center[1] + radius * np.sin(theta)

    # Vehicle & Stanley controller
    v_val = 0.5        # constant speed
    wheelbase = 1.0
    k = 0.5            # Stanley gain

    # High-order DO-ICBF parameters
    radius1 = 1.0
    alpha1 = 0.2
    alpha2 = 2.0
    alpha3 = 2.0

    # Symbolic definitions
    X, Y, psi_sym, steering_sym = sp.symbols('X Y psi steering')
    f_sym = sp.Matrix([v_val*sp.cos(psi_sym),
                       v_val*sp.sin(psi_sym),
                       v_val*sp.tan(steering_sym)])
    b0_sym = X**2 + Y**2 - radius1**2
    b1_sym = (sp.diff(b0_sym, X)*f_sym[0] +
              sp.diff(b0_sym, Y)*f_sym[1] +
              sp.diff(b0_sym, psi_sym)*f_sym[2] +
              alpha1*b0_sym)
    b2_sym = (sp.diff(b1_sym, X)*f_sym[0] +
              sp.diff(b1_sym, Y)*f_sym[1] +
              sp.diff(b1_sym, psi_sym)*f_sym[2] +
              alpha2*b1_sym)
    p_sym = sp.diff(b2_sym, steering_sym)

    # Numeric lambdified functions
    b1_func = sp.lambdify((X, Y, psi_sym), b1_sym, 'numpy')
    b2_func = sp.lambdify((X, Y, psi_sym, steering_sym), b2_sym, 'numpy')
    p_func  = sp.lambdify((X, Y, psi_sym, steering_sym), p_sym,  'numpy')

    # Initial vehicle state [x, y, psi]
    vehicle_state = np.array([radius+2.0, center[1], np.pi/2])
    steering_angle = 0.0

    steering_angles = []
    vehicle_states = []
    checks = []

    for i in range(num_steps):
        # Estimate steering rate
        if i < 2:
            udot = 0.0
        else:
            udot = (steering_angle - steering_angles[-2]) / dt

        # QP: A * v_star <= b
        p_val = p_func(vehicle_state[0], vehicle_state[1], vehicle_state[2], steering_angle)
        A_mat = np.array([[-p_val]])
        b_val = b2_func(vehicle_state[0], vehicle_state[1], vehicle_state[2], steering_angle)
        H = np.eye(1)
        F = np.zeros((1,))
        v_star = solve_qp(H, F, A_mat, np.array([b_val]))[0]
        checks.append(v_star)

        # Stanley steering law
        dist = np.hypot(circle_x - vehicle_state[0], circle_y - vehicle_state[1])
        idx = np.argmin(dist)
        cte = dist[idx]
        if vehicle_state[1] < circle_y[idx]:
            cte = -cte
        circle_theta = np.arctan2(np.diff(circle_y), np.diff(circle_x))
        circle_theta = np.concatenate([circle_theta, circle_theta[:1]])
        yaw_path = circle_theta[idx]
        yaw_diff = np.arctan2(np.sin(yaw_path - vehicle_state[2]),
                              np.cos(yaw_path - vehicle_state[2]))
        steering_angle = yaw_diff + np.arctan2(k*cte, v_val) + v_star

        # Bicycle-model update
        x, y, psi = vehicle_state
        x   += v_val * np.cos(psi) * dt
        y   += v_val * np.sin(psi) * dt
        psi += v_val * np.tan(steering_angle) / wheelbase * dt
        vehicle_state = np.array([x, y, psi])

        steering_angles.append(steering_angle)
        vehicle_states.append(vehicle_state.copy())

    vehicle_states = np.array(vehicle_states)
    t = np.arange(1, num_steps+1) * dt

    # Plot path tracking
    fig1, ax1 = plt.subplots()
    ax1.plot(circle_x, circle_y, 'b', label='Desired Path')
    ax1.plot(vehicle_states[:,0], vehicle_states[:,1], 'r.', label='Vehicle Path')
    circle_patch(0, 0, radius1, color='g', ax=ax1)
    ax1.set_xlabel('X (m)', fontsize=14)
    ax1.set_ylabel('Y (m)', fontsize=14)
    ax1.set_title('Vehicle tracking a circle using High-order DO-ICBF', fontsize=16)
    ax1.legend()
    ax1.grid(True)
    ax1.set_aspect('equal', 'box')
    ax1.set_xlim([-12, 12])
    ax1.set_ylim([-5, 15])

    # Plot DO-ICBF signals
    fig2, ax2 = plt.subplots()
    b0_vals = vehicle_states[:,0]**2 + vehicle_states[:,1]**2 - radius1**2
    b1_vals = b1_func(vehicle_states[:,0], vehicle_states[:,1], vehicle_states[:,2])
    b2_vals = np.array([b2_func(x,y,psi, steering_angles[i])
                        for i,(x,y,psi) in enumerate(vehicle_states)])
    ax2.plot(t, b0_vals, linewidth=2, label='$b_0(x)$')
    ax2.plot(t, b1_vals, 'g', linewidth=2, label='$b_1(x)$')
    ax2.plot(t, b2_vals, 'r', linewidth=2, label='$b_2(x,u)$')
    ax2.plot(t, np.zeros_like(t), 'k', linewidth=2)
    ax2.set_xlabel('Time (s)', fontsize=14)
    ax2.set_ylabel('DO-ICBF', fontsize=14)
    ax2.set_title('High-order DO-ICBF Signals', fontsize=16)
    ax2.legend(loc='best')
    ax2.grid(True)
    ax2.set_xlim([0, t[-1]])
    ax2.set_ylim([-10, 255])

    # Plot heading
    fig3, ax3 = plt.subplots()
    ax3.plot(t, vehicle_states[:,2], 'b', label='$\psi$ (rad)')
    ax3.set_xlabel('Time (s)', fontsize=14)
    ax3.set_ylabel('$\psi$ (rad)', fontsize=14)
    ax3.set_title('Vehicle Heading over Time', fontsize=16)
    ax3.grid(True)
    ax3.set_xlim([0, t[-1]])

    plt.show()


if __name__ == '__main__':
    main()
