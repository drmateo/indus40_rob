"""
Helper functions for TP02 Forward Kinematics
Extracted from tp_02_fk_main.ipynb
"""

import numpy as np
from math import cos, sin, pi, atan2, sqrt


# ============================================================================
# STANDARD DH CONVENTION
# ============================================================================

def dh_standard(theta, d, a, alpha):
    """
    Standard DH transformation matrix.
    Order: Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
    
    Parameters:
    - theta: rotation about z_{i-1}
    - d: translation along z_{i-1}
    - a: translation along x_i
    - alpha: rotation about x_i
    
    Returns: 4x4 homogeneous transformation matrix
    """
    ct, st = cos(theta), sin(theta)
    ca, sa = cos(alpha), sin(alpha)
    
    T = np.array([
        [ct,           -st*ca,           st*sa,           a*ct],
        [st,            ct*ca,          -ct*sa,           a*st],
        [0,                sa,              ca,              d],
        [0,                 0,               0,              1]
    ])
    return T


# ============================================================================
# MODIFIED DH CONVENTION (Craig's convention)
# ============================================================================

def dh_modified(theta, d, a, alpha):
    """
    Modified DH transformation matrix (Craig's convention).
    Order: Rx(alpha_{i-1}) * Tx(a_{i-1}) * Rz(theta_i) * Tz(d_i)
    
    Parameters:
    - theta: rotation about z_i
    - d: translation along z_i
    - a: translation along x_{i-1}
    - alpha: rotation about x_{i-1}
    
    Returns: 4x4 homogeneous transformation matrix
    """
    ct, st = cos(theta), sin(theta)
    ca, sa = cos(alpha), sin(alpha)
    
    T = np.array([
        [ct,           -st,              0,               a],
        [st*ca,         ct*ca,          -sa,           -d*sa],
        [st*sa,         ct*sa,           ca,            d*ca],
        [0,                 0,            0,               1]
    ])
    return T


# ============================================================================
# ELEMENTARY TRANSFORM SEQUENCE (ETS)
# ============================================================================

def rotx(theta):
    """Elementary rotation about X-axis."""
    ct, st = cos(theta), sin(theta)
    return np.array([
        [1,  0,   0,  0],
        [0,  ct, -st, 0],
        [0,  st,  ct, 0],
        [0,  0,   0,  1]
    ])

def roty(theta):
    """Elementary rotation about Y-axis."""
    ct, st = cos(theta), sin(theta)
    return np.array([
        [ct,  0,  st, 0],
        [0,   1,  0,  0],
        [-st, 0,  ct, 0],
        [0,   0,  0,  1]
    ])

def rotz(theta):
    """Elementary rotation about Z-axis."""
    ct, st = cos(theta), sin(theta)
    return np.array([
        [ct, -st, 0, 0],
        [st,  ct, 0, 0],
        [0,   0,  1, 0],
        [0,   0,  0, 1]
    ])

def transx(d):
    """Elementary translation along X-axis."""
    return np.array([
        [1, 0, 0, d],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def transy(d):
    """Elementary translation along Y-axis."""
    return np.array([
        [1, 0, 0, 0],
        [0, 1, 0, d],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def transz(d):
    """Elementary translation along Z-axis."""
    return np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d],
        [0, 0, 0, 1]
    ])


# ============================================================================
# GENERIC FORWARD KINEMATICS
# ============================================================================

def fk_dh(dh_params, q, convention='standard'):
    """
    Compute forward kinematics using DH parameters.
    
    Parameters:
    - dh_params: list of tuples (theta_offset, d_offset, a, alpha, joint_type)
                 joint_type: 'r' (revolute) or 'p' (prismatic)
    - q: array of joint values [q1, q2, ..., qn]
    - convention: 'standard' or 'modified'
    
    Returns: 
    - T_0n: end-effector transformation (4x4 matrix)
    - T_list: list of intermediate transformations [T_0, T_01, T_02, ...]
    """
    dh_func = dh_standard if convention == 'standard' else dh_modified
    
    T = np.eye(4)
    T_list = [T.copy()]
    
    for i, (theta_off, d_off, a, alpha, jtype) in enumerate(dh_params):
        if jtype == 'r':  # revolute
            theta = q[i] + theta_off
            d = d_off
        else:  # prismatic
            theta = theta_off
            d = q[i] + d_off
        
        T_i = dh_func(theta, d, a, alpha)
        T = T @ T_i
        T_list.append(T.copy())
    
    return T, T_list


def fk_ets(ets_sequence, q):
    """
    Compute forward kinematics using Elementary Transform Sequence.
    
    Parameters:
    - ets_sequence: list of tuples (transform_func, param, is_joint)
                    transform_func: one of {rotx, roty, rotz, transx, transy, transz}
                    param: if is_joint=True, this is the joint index; else constant value
                    is_joint: boolean indicating if this is a joint variable
    - q: array of joint values
    
    Returns:
    - T_0n: end-effector transformation (4x4 matrix)
    - T_list: list of intermediate transformations
    """
    T = np.eye(4)
    T_list = [T.copy()]
    
    for func, param, is_joint in ets_sequence:
        if is_joint:
            value = q[param]
        else:
            value = param
        
        T_i = func(value)
        T = T @ T_i
        T_list.append(T.copy())
    
    return T, T_list


# ============================================================================
# VISUALIZATION HELPERS
# ============================================================================

def plot_robot_2d(T_list, ax, title="Robot", draw_frames=False):
    """
    Plot 2D robot configuration from transformation list.
    """
    import matplotlib.pyplot as plt
    
    # Extract positions
    x = [T[0, 3] for T in T_list]
    y = [T[1, 3] for T in T_list]
    
    # Plot robot links
    ax.plot(x, y, 'o-', linewidth=3, markersize=10, label='Robot')
    ax.plot(x[0], y[0], 'go', markersize=12, label='Base')
    ax.plot(x[-1], y[-1], 'ro', markersize=12, label='End-effector')
    
    # Draw coordinate frames if requested
    if draw_frames:
        for i, T in enumerate(T_list):
            # X-axis (red)
            ax.arrow(T[0,3], T[1,3], 0.1*T[0,0], 0.1*T[1,0], 
                    head_width=0.02, head_length=0.02, fc='red', ec='red', alpha=0.7)
            # Y-axis (green)
            ax.arrow(T[0,3], T[1,3], 0.1*T[0,1], 0.1*T[1,1], 
                    head_width=0.02, head_length=0.02, fc='green', ec='green', alpha=0.7)
            ax.text(T[0,3]+0.05, T[1,3]+0.05, f'{i}', fontsize=10)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(title)
    ax.axis('equal')
    ax.grid(True, alpha=0.3)
    ax.legend()


def plot_robot_3d(T_list, ax, title="Robot", draw_frames=True):
    """
    Plot 3D robot configuration from transformation list.
    """
    # Extract positions
    x = [T[0, 3] for T in T_list]
    y = [T[1, 3] for T in T_list]
    z = [T[2, 3] for T in T_list]
    
    # Plot robot links
    ax.plot(x, y, z, 'o-', linewidth=3, markersize=8, label='Robot')
    ax.plot([x[0]], [y[0]], [z[0]], 'go', markersize=12, label='Base')
    ax.plot([x[-1]], [y[-1]], [z[-1]], 'ro', markersize=12, label='End-effector')
    
    # Draw coordinate frames if requested
    if draw_frames:
        scale = 0.1
        for i, T in enumerate(T_list):
            origin = T[:3, 3]
            # X-axis (red)
            ax.quiver(origin[0], origin[1], origin[2],
                     T[0,0]*scale, T[1,0]*scale, T[2,0]*scale,
                     color='red', arrow_length_ratio=0.3, alpha=0.7)
            # Y-axis (green)
            ax.quiver(origin[0], origin[1], origin[2],
                     T[0,1]*scale, T[1,1]*scale, T[2,1]*scale,
                     color='green', arrow_length_ratio=0.3, alpha=0.7)
            # Z-axis (blue)
            ax.quiver(origin[0], origin[1], origin[2],
                     T[0,2]*scale, T[1,2]*scale, T[2,2]*scale,
                     color='blue', arrow_length_ratio=0.3, alpha=0.7)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title)
    ax.legend()


# ============================================================================
# COMPARISON & VALIDATION
# ============================================================================

def compare_with_rtb(T_manual, T_rtb, verbose=False):
    """
    Compare manual FK result with RTB result.
    """
    try:
        # Handle SE3 object
        if hasattr(T_rtb, 'A'):
            T_rtb_array = T_rtb.A
        else:
            T_rtb_array = np.array(T_rtb)
        
        diff = np.max(np.abs(T_manual - T_rtb_array))
        
        if verbose:
            print(f"Maximum difference: {diff:.2e}")
            if diff < 1e-10:
                print("✓ Match!")
            else:
                print("✗ Mismatch detected")
        
        return diff < 1e-10
    except:
        print("Could not compare with RTB result")
        return False


# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def homogeneous_transform(R, p):
    """Create 4x4 homogeneous transformation from rotation matrix and position."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def extract_rotation(T):
    """Extract 3x3 rotation matrix from homogeneous transform."""
    return T[:3, :3]


def extract_position(T):
    """Extract position vector from homogeneous transform."""
    return T[:3, 3]


def rotation_to_euler_zyx(R):
    """Convert rotation matrix to ZYX Euler angles (in radians)."""
    if abs(R[2,0]) != 1:
        theta_y = -np.arcsin(R[2,0])
        theta_z = np.arctan2(R[1,0]/np.cos(theta_y), R[0,0]/np.cos(theta_y))
        theta_x = np.arctan2(R[2,1]/np.cos(theta_y), R[2,2]/np.cos(theta_y))
    else:
        theta_z = 0
        if R[2,0] == -1:
            theta_y = np.pi/2
            theta_x = theta_z + np.arctan2(R[0,1], R[0,2])
        else:
            theta_y = -np.pi/2
            theta_x = -theta_z + np.arctan2(-R[0,1], -R[0,2])
    return np.array([theta_z, theta_y, theta_x])


print("✓ Helper functions loaded from tp_02_fk_helpers.py")
