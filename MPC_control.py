#!/usr/bin/env python3
import pybullet as p
import time
import pybullet_data as pb
import numpy as np

import math
import time

import cvxpy
import numpy as np

# Model parameters


nx = 4  # number of state
nu = 1  # number of input
Q = np.diag([1.0, 1.0, 10.0, 10.0])  # state cost matrix
R = np.diag([0.01])  # input cost matrix

T = 30  # Horizon length
delta_t = 0.1  # time tick
sim_time = 5.0  # simulation time [s]

show_animation = True


def main():
    x0 = np.array([
        [1.0],
        [0.0],
        [1.3],
        [0.0]
    ])

    x = np.copy(x0)
    # time = 0.0
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pb.getDataPath())
    p.setRealTimeSimulation(0)  # optionally
    p.setGravity(0, 0, -9.8)
    startPos = [0, 0, 0]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    planeId = p.loadURDF(
        "/home/bhavik/CartPole_control/cart.urdf", startPos, startOrientation)
    # print(planeId)
    # print(p.getNumJoints(planeId))
    p.setJointMotorControl2(planeId, 0, p.POSITION_CONTROL, force=0)
    p.setJointMotorControl2(planeId, 1, p.POSITION_CONTROL, force=0)
    while True:
        # or p.DIRECT for non-graphical version

        # time += delta_t

        # calc control input
        opt_x, opt_delta_x, opt_theta, opt_delta_theta, opt_input = \
            mpc_control(x)

        # get input
        u = opt_input[0]

        # simulate inverted pendulum cart
        x = simulation(x, u)

        # desired = np.matrix([
        #     [3],
        #     [0],
        #     [0],
        #     [0]
        # ])

        # F = (x-desired)
        print(x)

        p.setJointMotorControl2(
            planeId, 0, p.POSITION_CONTROL, targetPosition=x[0])
        p.setJointMotorControl2(
            planeId, 1, p.POSITION_CONTROL, targetPosition=x[2])
        p.stepSimulation()
        time.sleep(1./240.)


def simulation(x, u):
    A, B = get_model_matrix()
    x = np.dot(A, x) + np.dot(B, u)

    return x


def mpc_control(x0):
    x = cvxpy.Variable((nx, T + 1))
    u = cvxpy.Variable((nu, T))

    A, B = get_model_matrix()

    cost = 0.0
    constr = []
    for t in range(T):
        cost += cvxpy.quad_form(x[:, t + 1], Q)
        cost += cvxpy.quad_form(u[:, t], R)
        constr += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t]]

    constr += [x[:, 0] == x0[:, 0]]
    prob = cvxpy.Problem(cvxpy.Minimize(cost), constr)

    start = time.time()
    prob.solve(verbose=False)
    elapsed_time = time.time() - start
    print(f"calc time:{elapsed_time:.6f} [sec]")

    if prob.status == cvxpy.OPTIMAL:
        ox = get_numpy_array_from_matrix(x.value[0, :])
        dx = get_numpy_array_from_matrix(x.value[1, :])
        theta = get_numpy_array_from_matrix(x.value[2, :])
        d_theta = get_numpy_array_from_matrix(x.value[3, :])

        ou = get_numpy_array_from_matrix(u.value[0, :])
    else:
        ox, dx, theta, d_theta, ou = None, None, None, None, None

    return ox, dx, theta, d_theta, ou


def get_numpy_array_from_matrix(x):
    """
    get build-in list from matrix
    """
    return np.array(x).flatten()


def get_model_matrix():
    M = 0.5
    m = 0.5
    b = 0.1
    I = 1.0
    l = 1.0
    g = 9.8
    p = I*(M+m)+M*m*(l**2)
    A = np.array([[0,     1,          0,   0],
                  [0, -(I+m*(l ** 2))*b/p, ((m ** 2)*g*(l ** 2))/p,   0],
                  [0,      0,              0,           1],
                  [0, -(m*l*b)/p,       m*g*l*(M+m)/p,  0]])

    B = np.array([[0],
                  [(I+m*(l ** 2))/p],
                  [0],
                  [m*l/p]])
    A = np.eye(nx) + delta_t * A

    B = delta_t * B

    return A, B


def flatten(a):
    return np.array(a).flatten()


if __name__ == '__main__':
    main()
