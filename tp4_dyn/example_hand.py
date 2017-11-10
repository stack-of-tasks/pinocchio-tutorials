from pinocchio.utils import *
from robot_hand import Robot
from time import sleep
from numpy import cos

robot = Robot()
robot.display(robot.q0)

q = robot.q0.copy()
for i in range(10000):
    for iq in range(3,robot.model.nq):
        q[iq] = -1+cos(i*1e-2*(1+iq/5))
    robot.display(q)
    sleep(1e-3)
