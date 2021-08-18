# %%

import numpy as np
import sympy
import math
from math import pi,cos,sin,tan,atan
from numpy import *
from numpy.core.umath import arctan2, deg2rad, rad2deg
from sympy.concrete import delta
from cmath import cos, sin, sqrt



# Length of links in cm
a1= 5.2
a2 = 6.9
a3 = 6.8

# Desired Position of End effector
px = -14
py = 3

phi = 90
phi = deg2rad(phi)

# Equations for Inverse kinematics
wx = px - a3*cos(phi)
wy = py - a3*sin(phi)

elta = wx**2 + wy**2
c2 = (delta - a1**2 - a2**2)/(2*a1*a2)
s2 = sqrt(1-c2**2)  # elbow down
theta_2 = arctan2(s2, c2)

s1 = ((a1+a2*c2)*wy - a2*s2*wx)/delta
c1 = ((a1+a2*c2)*wx + a2*s2*wy)/delta
theta_1 = arctan2(s1,c1)
theta_3 = phi-theta_1-theta_2

print('theta_1: ', rad2deg(theta_1))
print('theta_2: ', rad2deg(theta_2))
print('theta_3: ', rad2deg(theta_3))


# Link lengths
a1 = 6.2  # length of link a1 in cm
a2 = 5.2  # length of link a2 in cm
a3 = 0  # length of link a3 in cm
a4 = 6.9  # length of link a4 in cm
a5 = 0  # length of link a5 in cm
a6 = 6.8  # length of link a6 in cm

# Angles
theta_1 = 90  # theta 1 angle in degrees
theta_2 = 90  # theta 2 angle in degrees
theta_3 = 90  # theta 3 angle in degrees

theta_1 = (theta_1/180)*pi  # theta 1 in radians
theta_2 = (theta_2/180)*pi  # theta 2 in radians
theta_3 = (theta_3/180)*pi  # theta 3 in radians

# DH Parameter Table for 3 DOF Planar
PT = [[theta_1, 0, a2, a1],
    [theta_2, 0, a4, a3],
    [theta_3, 0, a6, a5]]

# Homogeneous Transformation Matrices
i = 0
H0_1 = [[cos(PT[i][0]), -sin(PT[i][0])*cos(PT[i][1]), sin(PT[i][0])*sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
    [sin(PT[i][0]), cos(PT[i][0])*cos(PT[i][1]), -cos(PT[i][0])*sin(PT[i][1]), PT[i][2]*sin(PT[i][0])],
    [0, sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
    [0, 0, 0, 1]]

i = 1
H1_2 = [[cos(PT[i][0]), -sin(PT[i][0])*cos(PT[i][1]), sin(PT[i][0])*sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
    [sin(PT[i][0]), cos(PT[i][0])*cos(PT[i][1]), -cos(PT[i][0])*sin(PT[i][1]), PT[i][2]*sin(PT[i][0])],
    [0, sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
    [0, 0, 0, 1]]

i = 2
H2_3 = [[cos(PT[i][0]), -sin(PT[i][0])*cos(PT[i][1]), sin(PT[i][0])*sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
    [sin(PT[i][0]), cos(PT[i][0])*cos(PT[i][1]), -cos(PT[i][0])*sin(PT[i][1]), PT[i][2]*sin(PT[i][0])],
    [0, sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
    [0, 0, 0, 1]]

print("H0_1 =")
print(matrix(H0_1))
print("H1_2 =")
print(matrix(H1_2))
print("H2_3 =")
print(matrix(H2_3))

H0_2 = dot(H0_1,H1_2)
H0_3 = dot(H0_2,H2_3)

print("H0_3 =")
print(matrix(H0_3))
