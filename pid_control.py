#!/usr/bin/env python3
import pybullet as p
import time
import pybullet_data
import numpy as np
import rospy

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setRealTimeSimulation(0)  # optionally
p.setGravity(0, 0, -9.8)
startPos = [0, 0, 0]
ALPHA = 3000
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
planeId = p.loadURDF(
    "/home/bhavik/CartPole_control/cart.urdf", startPos, startOrientation)
print(planeId)
print(p.getNumJoints(planeId))
p.setJointMotorControl2(planeId, 0, p.POSITION_CONTROL, force=0)
p.setJointMotorControl2(planeId, 1, p.POSITION_CONTROL, force=0)
# p.applyExternalForce(planeId, 0, (-100, 0, 0), startPos, p.WORLD_FRAME)
kp = 10
kd = 1.4
kp2 = 100
kd2 = 12.4
error = 0
error_old = 0
desired_pos = np.array([3, 0])
dt = 0.001
error_cart = 0
# set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
while True:

    pos1, vel_1, _, _ = p.getJointState(planeId, 0)
    pos2, vel_2, _, _ = p.getJointState(planeId, 1)
    pos = np.array([pos1, pos2])
    error = pos - desired_pos
    error_d = (error - error_old)/dt
    control_force = (kp * error[0]) + (kd *
                                       error_d[0]) + (kp2 * error[1]) + (kd2 * error_d[1])
    # print(control_force)
    error_old = error

    # print(pos2, -force)
    p.setJointMotorControl2(
        planeId, 0, p.TORQUE_CONTROL, force=control_force)

    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
