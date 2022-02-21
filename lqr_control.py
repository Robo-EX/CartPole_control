#!/usr/bin/env python3
import pybullet as p
import time
from control.matlab import *
import pybullet_data as pb
import numpy as np
import rospy


def find_lqr_control_input(pos1, vel_1, pos2, vel_2, g=-9.8, b_mass=0.5, c_mass=0.5):
    # Using LQR to find control inputs

    # The A and B matrices
    M = 0.5
    m = 0.5
    b = 0.1
    I = 1.0
    l = 1.0

    p = I*(M+m)+M*m*(l**2)
    # denominator for the A and B matrices

    A = np.matrix([[0,     1,          0,   0],
                   [0, -(I+m*(l ** 2))*b/p, ((m ** 2)*g*(l ** 2))/p,   0],
                   [0,      0,              0,           1],
                   [0, -(m*l*b)/p,       m*g*l*(M+m)/p,  0]])

    B = np.matrix([[0],
                   [(I+m*(l ** 2))/p],
                   [0],
                   [m*l/p]])

    Q = np.matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 10, 0],
        [0, 0, 0, 100]
    ])

    R = np.matrix([[0.01]])
    # The K matrix is calculated using the lqr function from the controls library
    K, S, E = lqr(A, B, Q, R)
    # np.matrix(K)

    x = np.matrix([
        [pos1],
        [vel_1],
        [pos2],
        [vel_2]
    ])
    desired = np.matrix([
        [3],
        [0],
        [0],
        [0]
    ])

    F = K*(x-desired)
    # print(np.squeeze(np.asarray(F)))
    print(F)
    return np.squeeze(np.asarray(F))


physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pb.getDataPath())
p.setRealTimeSimulation(0)  # optionally
p.setGravity(0, 0, -9.8)
startPos = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
planeId = p.loadURDF(
    "/home/bhavik/catkin_ws/src/cartpole/src/cart.urdf", startPos, startOrientation)
# print(planeId)
# print(p.getNumJoints(planeId))
p.setJointMotorControl2(planeId, 0, p.POSITION_CONTROL, force=0)
p.setJointMotorControl2(planeId, 1, p.POSITION_CONTROL, force=0)
# p.applyExternalForce(planeId, 0, (-100, 0, 0), startPos, p.WORLD_FRAME)
kp = 3
kd = 0.5
kp2 = 1000
kd2 = 20
error = 0
error_old = 0
desired_pos = np.array([10, 0])
dt = 0.001
error_cart = 0
# set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
while True:

    pos1, vel_1, _, _ = p.getJointState(planeId, 0)
    pos2, vel_2, _, _ = p.getJointState(planeId, 1)
    pos = np.array([pos1, pos2])
    error = desired_pos - pos
    error_d = (error - error_old)/dt
    # control_force = (kp * error[0]) + (kd *
    #                                    error_d[0]) + (kp2 * error[1]) + (kd2 * error_d[1])
    # # print(control_force)
    # print(pos1, vel_1, pos2, vel_2)
    control_force = find_lqr_control_input(
        pos1, vel_1, pos2, vel_2)
    error_old = error
    # print(pos2, -force)
    p.setJointMotorControl2(
        planeId, 0, p.TORQUE_CONTROL, force=control_force)

    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
