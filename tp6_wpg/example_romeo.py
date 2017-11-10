import pinocchio as se3
from pinocchio.romeo_wrapper import RomeoWrapper
from pinocchio.utils import *
from numpy.linalg import norm

path = '/home/nmansard/src/pinocchio/pinocchio/models/romeo/'
urdf = path + 'urdf/romeo.urdf'
pkgs = [ path, ]
robot = RomeoWrapper(urdf,pkgs,se3.JointModelFreeFlyer()) # Load urdf model
                                  # The robot is loaded with the basis fixed to the world
robot.initDisplay(loadModel=True) # setup the viewer to display the robot

NQ = robot.model.nq               # model configuration size (6)
NV = robot.model.nv               # model configuration velocity size (6)

q  = rand(NQ)                     # Set up an initial configuration
q[3:7] /= norm(q[3:7])            # Normalize the quaternion
robot.display(q)

vq = zero(NV)
vq[10] = 1                         # Set up a constant robot speed

from time import sleep
for i in range(10000):            # Move the robot with constant velocity
    q[7:] += vq[6:]/100
    robot.display(q)
    sleep(.01)
