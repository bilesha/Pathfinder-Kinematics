# %%

import numpy as np
import sympy


def disp(expr):
    """Displays a simplified Sympy expression."""   

    e = sympy.simplify(expr)
    e = sympy.expand(e)
    e = e.evalf()

    print(sympy.pretty(e))

    return

def rot_x(alpha):
    """Returns a homogeneous transform for just a rotation about the X axis by alpha."""

    T = sympy.Matrix([[1, 0, 0, 0],
                      [0, sympy.cos(alpha), -sympy.sin(alpha), 0],
                      [0, sympy.sin(alpha), sympy.cos(alpha), 0],
                      [0, 0, 0, 1]])

    return T

def trans_x(a):
    """Returns a homogeneous transform for just a translation along the X axis by a."""

    T = sympy.Matrix([[1, 0, 0, a],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

    return T

def rot_z(theta):
    """Returns a homogeneous transform for just a rotation about the Z axis by theta."""

    T = sympy.Matrix([[sympy.cos(theta), -sympy.sin(theta), 0, 0],
                      [sympy.sin(theta), sympy.cos(theta), 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

    return T

def trans_z(d):
    """Returns a homogeneous transform for just a rotation along the Z axis by d."""

    T = sympy.Matrix([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, d],
                      [0, 0, 0, 1]])

    return T

def h_T(alpha, a, theta, d):
    """Returns a general homogeneous transform."""

    T = rot_x(alpha) @ trans_x(a) @ rot_z(theta) @ trans_z(d)

    return T

# %%

# Forward kinematics for the given problem.

theta_1 = np.deg2rad(0.0)
T01 = h_T(0, 0, theta_1, 0)
disp(T01)

# %%

alpha_1 = np.deg2rad(90)
theta_2 = np.deg2rad(0.0)
T12 = h_T(alpha_1, 0, theta_2, 0)
disp(T12)

# %%

l1 = 100
theta_3 = np.deg2rad(45.0)
T23 = h_T(0, l1, theta_3, 0)
disp(T23)

# %%

l2 = 100
theta_4 = np.deg2rad(45.0)
T34 = h_T(0, l2, theta_4, 0)
disp(T34)

# %%

l3 = 100
theta_5 = np.deg2rad(0)
T45 = h_T(0, l3, theta_5, 0)
disp(T45)

# %%

T05 = T01 @ T12 @ T23 @ T34 @ T45
disp(T05)

x05 = float(T05[0, 3])
y05 = float(T05[1, 3])
z05 = float(T05[2, 3])

print("x05: {}".format(x05))
print("y05: {}".format(y05))
print("z05: {}".format(z05))

# %%

# Inverse kinematics for the given problem.

x = 170.711
y = 0.0
z = 170.711

theta_1 = np.arctan2(y, z)
T01 = h_T(0, 0, theta_1, 0)
disp(T01)

T45 = h_T(0, 100, 0, 0)
disp(T45)

# %%

T15 = T05 @ T01.inv()
disp(T15)

#%%

T14 = T15 @ T45.inv()
disp(T14)

x14 = float(T14[0, 3])
y14 = float(T14[1, 3])
z14 = float(T14[2, 3])

print("x14: {}".format(x14))
print("y14: {}".format(y14))
print("z14: {}".format(z14))

# %%

B = np.arctan2(z14, x14)
c2 = (l2**2 - l1**2 - x14**2 - z14**2) / (-2*l1*np.sqrt(x14**2 + z14**2))
s2 = np.sqrt(1 - c2**2)
w = np.arctan2(s2, c2)
theta_2 = B - w

print("theta_2: {}".format(np.rad2deg(theta_2)))

c3 = (x14**2 + z14**2 - l1**2 - l2**2)/(2*l1*l2)
s3 = np.sqrt(1 - c3**2)
theta_3 = np.arctan2(s3, c3)

print("theta_3: {}".format(np.rad2deg(theta_3)))

# %%

T35 = T34 @ T45
disp(T35)

x35 = float(T35[0, 3])
y35 = float(T35[1, 3])
z35 = float(T35[2, 3])

print("x35: {}".format(x35))
print("y35: {}".format(y35))
print("z35: {}".format(z35))

# %%

c4 = (x35 - l2) / l3
s4 = np.sqrt(1 - c4**2)
theta_4 = np.arctan2(s4, c4)

print("theta_4: {}".format(np.rad2deg(theta_4)))
