"""
Helper functions for TP03 Inverse Kinematics
Following the philosophy of TP02 (modernization with ETS and RTB integration)
"""

import numpy as np
from math import cos, sin, pi, atan2, sqrt, acos, asin
from scipy.linalg import pinv, norm

# Import FK helpers from TP02
try:
    from tp_02_fk_helpers import (
        dh_standard, dh_modified, fk_dh, fk_ets,
        rotx, roty, rotz, transx, transy, transz,
        extract_position, extract_rotation, homogeneous_transform
    )
except ImportError:
    print("Warning: tp_02_fk_helpers not found. Some functions may not work.")


# ============================================================================
# POSE ERROR COMPUTATION (Modern approach with spatialmath)
# ============================================================================

def pose_error(T_current, T_desired, use_spatialmath=False):
    """
    Compute pose error between current and desired transformations.
    
    Modern approach: Can use spatialmath SE3 for more robust computation.
    Traditional approach: Manual matrix operations.
    
    Parameters:
    - T_current: current end-effector transformation (4x4 or SE3)
    - T_desired: desired end-effector transformation (4x4 or SE3)
    - use_spatialmath: if True and available, use SE3 methods
    
    Returns:
    - error: 6D error vector [position_error (3), orientation_error (3)]
    """
    try:
        from spatialmath import SE3
        
        # Convert to SE3 if needed
        if use_spatialmath:
            if not isinstance(T_current, SE3):
                T_current = SE3(T_current)
            if not isinstance(T_desired, SE3):
                T_desired = SE3(T_desired)
            
            # Position error
            p_error = T_desired.t - T_current.t
            
            # Orientation error (angle-axis representation)
            R_error = T_desired.R @ T_current.R.T
            # Convert to axis-angle
            theta = np.arccos(np.clip((np.trace(R_error) - 1) / 2, -1, 1))
            if abs(theta) < 1e-6:
                o_error = np.zeros(3)
            else:
                axis = np.array([
                    R_error[2,1] - R_error[1,2],
                    R_error[0,2] - R_error[2,0],
                    R_error[1,0] - R_error[0,1]
                ]) / (2 * np.sin(theta))
                o_error = theta * axis
            
            return np.concatenate([p_error, o_error])
    except ImportError:
        pass
    
    # Traditional approach (numpy arrays)
    if hasattr(T_current, 'A'):
        T_current = T_current.A
    if hasattr(T_desired, 'A'):
        T_desired = T_desired.A
    
    # Position error
    p_error = extract_position(T_desired) - extract_position(T_current)
    
    # Orientation error (simplified axis-angle)
    R_current = extract_rotation(T_current)
    R_desired = extract_rotation(T_desired)
    R_error = R_desired @ R_current.T
    
    theta = np.arccos(np.clip((np.trace(R_error) - 1) / 2, -1, 1))
    if abs(theta) < 1e-6:
        o_error = np.zeros(3)
    else:
        axis = np.array([
            R_error[2,1] - R_error[1,2],
            R_error[0,2] - R_error[2,0],
            R_error[1,0] - R_error[0,1]
        ]) / (2 * np.sin(theta))
        o_error = theta * axis
    
    return np.concatenate([p_error, o_error])


def pose_error_simple(T_current, T_desired):
    """
    Simplified pose error (position only) for 2D/3D position-only IK.
    
    Returns:
    - error: 3D position error vector
    """
    if hasattr(T_current, 'A'):
        T_current = T_current.A
    if hasattr(T_desired, 'A'):
        T_desired = T_desired.A
    
    return extract_position(T_desired) - extract_position(T_current)


# ============================================================================
# JACOBIAN COMPUTATION
# ============================================================================

def numerical_jacobian(robot_params, q, epsilon=1e-6, method='dh', convention='standard'):
    """
    Compute numerical Jacobian (6×n) using finite differences.
    
    Modern approach: Can work with DH or ETS representations.
    
    Parameters:
    - robot_params: DH table or ETS sequence
    - q: current joint configuration
    - epsilon: finite difference step size
    - method: 'dh' or 'ets'
    - convention: 'standard' or 'modified' (for DH)
    
    Returns: 
    - J: 6×n Jacobian matrix such that dx = J * dq
    """
    n = len(q)
    J = np.zeros((6, n))
    
    # Forward kinematics function selection
    if method == 'ets':
        fk_func = lambda q_val: fk_ets(robot_params, q_val)
    else:
        fk_func = lambda q_val: fk_dh(robot_params, q_val, convention)
    
    # Reference configuration
    T_0, _ = fk_func(q)
    p_0 = extract_position(T_0)
    R_0 = extract_rotation(T_0)
    
    for i in range(n):
        q_delta = q.copy()
        q_delta[i] += epsilon
        
        T_delta, _ = fk_func(q_delta)
        p_delta = extract_position(T_delta)
        R_delta = extract_rotation(T_delta)
        
        # Linear velocity
        J[:3, i] = (p_delta - p_0) / epsilon
        
        # Angular velocity (simplified from rotation matrix derivative)
        dR = (R_delta - R_0) / epsilon
        J[3, i] = (dR[2,1] - dR[1,2]) / 2
        J[4, i] = (dR[0,2] - dR[2,0]) / 2
        J[5, i] = (dR[1,0] - dR[0,1]) / 2
    
    return J


def jacobian_from_rtb(robot, q):
    """
    Get analytical Jacobian from RTB robot model.
    
    This is the modern/industrial approach - use analytical Jacobian
    when available instead of numerical approximation.
    
    Parameters:
    - robot: RTB robot object
    - q: joint configuration
    
    Returns:
    - J: 6×n Jacobian matrix
    """
    try:
        # RTB provides analytical Jacobian
        return robot.jacob0(q)
    except:
        print("Warning: Could not compute RTB Jacobian")
        return None


# ============================================================================
# ANALYTICAL IK HELPERS
# ============================================================================

def normalize_angle(angle):
    """Normalize angle to [-π, π]."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def check_joint_limits(q, limits):
    """
    Check if joint configuration respects limits.
    
    Parameters:
    - q: joint configuration
    - limits: list of (min, max) tuples
    
    Returns: 
    - valid: True if within limits
    """
    for i, (qmin, qmax) in enumerate(limits):
        if q[i] < qmin or q[i] > qmax:
            return False
    return True


def rotation_matrix_zyz_to_euler(R):
    """
    Extract ZYZ Euler angles from rotation matrix.
    Used for spherical wrist orientation subproblem.
    
    Returns: [alpha, beta, gamma] corresponding to Rz(α)Ry(β)Rz(γ)
    """
    # Check for gimbal lock
    if abs(R[2, 2]) < 0.9999:
        beta = np.arccos(R[2, 2])
        alpha = np.arctan2(R[1, 2], R[0, 2])
        gamma = np.arctan2(R[2, 1], -R[2, 0])
    else:
        # Gimbal lock case
        if R[2, 2] > 0:  # β ≈ 0
            beta = 0
            alpha = 0
            gamma = np.arctan2(-R[1, 0], R[0, 0])
        else:  # β ≈ π
            beta = pi
            alpha = 0
            gamma = np.arctan2(R[1, 0], R[0, 0])
    
    return np.array([alpha, beta, gamma])


# ============================================================================
# NUMERICAL IK METHODS
# ============================================================================

def ik_newton_raphson(robot_params, T_target, q_init, 
                      max_iter=100, tol=1e-4, alpha=0.5,
                      method='dh', convention='standard',
                      use_rtb_jacobian=False, robot_rtb=None):
    """
    Numerical IK using Newton-Raphson with Jacobian pseudo-inverse.
    
    Modern enhancements:
    - Can use DH or ETS
    - Can use RTB analytical Jacobian if available
    - Adaptive step size
    
    Parameters:
    - robot_params: DH table or ETS sequence
    - T_target: desired end-effector transformation (4x4)
    - q_init: initial joint configuration
    - max_iter: maximum iterations
    - tol: convergence tolerance
    - alpha: step size (0 < alpha <= 1)
    - method: 'dh' or 'ets'
    - convention: 'standard' or 'modified' (for DH)
    - use_rtb_jacobian: if True, use analytical Jacobian from RTB
    - robot_rtb: RTB robot object (needed if use_rtb_jacobian=True)
    
    Returns:
    - q: solution joint angles
    - converged: True if converged
    - iterations: number of iterations
    - history: error history
    """
    q = q_init.copy()
    history = []
    
    # Select FK function
    if method == 'ets':
        fk_func = lambda q_val: fk_ets(robot_params, q_val)
    else:
        fk_func = lambda q_val: fk_dh(robot_params, q_val, convention)
    
    for iteration in range(max_iter):
        # Forward kinematics
        T_current, _ = fk_func(q)
        
        # Compute error
        error = pose_error(T_current, T_target)
        error_norm = np.linalg.norm(error)
        history.append(error_norm)
        
        # Check convergence
        if error_norm < tol:
            return q, True, iteration + 1, history
        
        # Compute Jacobian
        if use_rtb_jacobian and robot_rtb is not None:
            J = jacobian_from_rtb(robot_rtb, q)
            if J is None:
                J = numerical_jacobian(robot_params, q, method=method, convention=convention)
        else:
            J = numerical_jacobian(robot_params, q, method=method, convention=convention)
        
        # Pseudo-inverse
        J_pinv = pinv(J)
        
        # Update
        delta_q = J_pinv @ error
        q = q + alpha * delta_q
        
        # Normalize angles
        q = np.array([normalize_angle(qi) for qi in q])
    
    return q, False, max_iter, history


def ik_damped_ls(robot_params, T_target, q_init,
                 max_iter=100, tol=1e-4, alpha=0.5, lambda_damp=0.01,
                 method='dh', convention='standard'):
    """
    Damped Least Squares IK (Levenberg-Marquardt style).
    
    More robust near singularities than Newton-Raphson.
    Uses: Δq = (J^T J + λ I)^(-1) J^T e
    
    Parameters: Same as ik_newton_raphson, plus:
    - lambda_damp: damping factor (larger = more robust, slower convergence)
    
    Returns: Same as ik_newton_raphson
    """
    q = q_init.copy()
    n = len(q)
    history = []
    
    # Select FK function
    if method == 'ets':
        fk_func = lambda q_val: fk_ets(robot_params, q_val)
    else:
        fk_func = lambda q_val: fk_dh(robot_params, q_val, convention)
    
    for iteration in range(max_iter):
        T_current, _ = fk_func(q)
        error = pose_error(T_current, T_target)
        error_norm = np.linalg.norm(error)
        history.append(error_norm)
        
        if error_norm < tol:
            return q, True, iteration + 1, history
        
        J = numerical_jacobian(robot_params, q, method=method, convention=convention)
        
        # Damped pseudo-inverse
        JTJ = J.T @ J
        damped_inv = np.linalg.inv(JTJ + lambda_damp * np.eye(n))
        delta_q = damped_inv @ J.T @ error
        
        q = q + alpha * delta_q
        q = np.array([normalize_angle(qi) for qi in q])
    
    return q, False, max_iter, history


# ============================================================================
# VISUALIZATION HELPERS FOR IK
# ============================================================================

def plot_robot_2d(T_list, ax=None, title="Robot Configuration", target=None):
    """
    Visualize robot in 2D with optional target.
    Enhanced version for IK visualization.
    """
    import matplotlib.pyplot as plt
    
    if ax is None:
        fig, ax = plt.subplots(figsize=(8, 8))
    
    x = [T[0, 3] for T in T_list]
    y = [T[1, 3] for T in T_list]
    
    ax.plot(x, y, 'o-', linewidth=2, markersize=8, label='Robot', color='blue')
    ax.plot(x[0], y[0], 'gs', markersize=12, label='Base')
    ax.plot(x[-1], y[-1], 'r*', markersize=15, label='End-effector')
    
    # Target position
    if target is not None:
        ax.plot(target[0], target[1], 'mx', markersize=15, 
                markeredgewidth=3, label='Target')
        # Draw line from EE to target
        ax.plot([x[-1], target[0]], [y[-1], target[1]], 
                'r--', alpha=0.5, linewidth=1)
    
    # End-effector frame
    T_end = T_list[-1]
    scale = 0.1
    ax.arrow(T_end[0,3], T_end[1,3], scale*T_end[0,0], scale*T_end[1,0],
             head_width=0.03, head_length=0.02, fc='red', ec='red', alpha=0.7)
    ax.arrow(T_end[0,3], T_end[1,3], scale*T_end[0,1], scale*T_end[1,1],
             head_width=0.03, head_length=0.02, fc='green', ec='green', alpha=0.7)
    
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    ax.set_title(title, fontsize=12, fontweight='bold')
    ax.legend(loc='best')
    
    return ax


def compare_solutions(robot_params, solutions, target_pos, title="IK Solutions",
                     method='dh', convention='standard'):
    """
    Visualize multiple IK solutions for the same target.
    
    Parameters:
    - robot_params: DH table or ETS sequence
    - solutions: list of (q, label) tuples
    - target_pos: target position (x, y) or (x, y, z)
    - title: plot title
    - method: 'dh' or 'ets'
    - convention: 'standard' or 'modified' (for DH)
    """
    import matplotlib.pyplot as plt
    
    n_sols = len(solutions)
    fig, axes = plt.subplots(1, n_sols, figsize=(6*n_sols, 5))
    
    if n_sols == 1:
        axes = [axes]
    
    # Select FK function
    if method == 'ets':
        fk_func = lambda q_val: fk_ets(robot_params, q_val)
    else:
        fk_func = lambda q_val: fk_dh(robot_params, q_val, convention)
    
    for idx, (q, label) in enumerate(solutions):
        _, T_list = fk_func(q)
        plot_robot_2d(T_list, axes[idx], 
                      title=f"{label}\nq={np.rad2deg(q).astype(int)}°",
                      target=target_pos[:2])
    
    plt.suptitle(title, fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.show()


# ============================================================================
# RTB INTEGRATION HELPERS
# ============================================================================

def create_rtb_robot_from_dh(dh_params, name="Custom Robot", convention='standard'):
    """
    Create RTB robot from DH parameters.
    Modern approach: leverage RTB's built-in functionality.
    
    Parameters:
    - dh_params: list of (theta_off, d_off, a, alpha, jtype)
    - name: robot name
    - convention: 'standard' or 'modified'
    
    Returns:
    - robot: RTB DHRobot object
    """
    try:
        from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
        
        links = []
        for theta_off, d_off, a, alpha, jtype in dh_params:
            if convention == 'modified':
                if jtype == 'r':
                    links.append(RevoluteDH(d=d_off, a=a, alpha=alpha, offset=theta_off))
                else:
                    links.append(PrismaticDH(theta=theta_off, a=a, alpha=alpha, offset=d_off))
            else:
                if jtype == 'r':
                    links.append(RevoluteDH(d=d_off, a=a, alpha=alpha, offset=theta_off))
                else:
                    links.append(PrismaticDH(theta=theta_off, a=a, alpha=alpha, offset=d_off))
        
        robot = DHRobot(links, name=name)
        return robot
    except ImportError:
        print("RTB not available")
        return None


def create_rtb_robot_from_ets(ets_sequence, name="Custom Robot"):
    """
    Create RTB robot from ETS sequence.
    This is the MODERN approach following TP02 philosophy!
    
    Parameters:
    - ets_sequence: list of (transform_func, param, is_joint)
    - name: robot name
    
    Returns:
    - robot: RTB ERobot object with ETS
    """
    try:
        import roboticstoolbox as rtb
        
        # Map our functions to RTB ET
        func_map = {
            'rotx': lambda theta: rtb.ET.Rx(theta) if theta != 0 else rtb.ET.Rx(),
            'roty': lambda theta: rtb.ET.Ry(theta) if theta != 0 else rtb.ET.Ry(),
            'rotz': lambda theta: rtb.ET.Rz(theta) if theta != 0 else rtb.ET.Rz(),
            'transx': lambda d: rtb.ET.tx(d),
            'transy': lambda d: rtb.ET.ty(d),
            'transz': lambda d: rtb.ET.tz(d),
        }
        
        et_list = []
        for func, param, is_joint in ets_sequence:
            func_name = func.__name__
            if is_joint:
                # Variable transform
                et_list.append(func_map[func_name](0))  # Joint variable
            else:
                # Constant transform
                et_list.append(func_map[func_name](param))
        
        # Concatenate ETs
        ets = et_list[0]
        for et in et_list[1:]:
            ets *= et
        
        robot = rtb.ERobot(ets, name=name)
        return robot
    except ImportError:
        print("RTB not available")
        return None
    except Exception as e:
        print(f"Error creating RTB robot: {e}")
        return None


print("✓ IK helper functions loaded from tp_03_ik_helpers.py")
print("  Modern approach: ETS support, RTB integration, spatialmath compatibility")
