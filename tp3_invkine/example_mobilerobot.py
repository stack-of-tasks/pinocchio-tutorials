'''
This is a simple example showing how to define a mobile manipulator using the class MobileRobotWrapper.
'''

import pinocchio as se3
from mobilerobot import MobileRobotWrapper
from pinocchio.utils import *

pkg = '/home/student/models/'
urdf = pkg + 'ur_description/urdf/ur5_gripper.urdf'
                     
robot = MobileRobotWrapper(urdf,[pkg,])
robot.initDisplay(loadModel=True)
#robot.viewer.gui.addFloor('world/floor')

NQ = robot.model.nq
NV = robot.model.nv

q  = se3.randomConfiguration(robot.model,zero(NQ)-np.pi,zero(NQ)+np.pi)
vq = rand(NV)

robot.display(q)

from time import sleep
for i in range(10000):
    robot.increment(q,vq/100)
    robot.display(q)
    sleep(.01)

IDX_TOOL  = 24
IDX_BASIS = 23

se3.framesKinematics(robot.model,robot.data)
Mtool = robot.data.oMf[IDX_TOOL]
Mbasis = robot.data.oMf[IDX_BASIS]
