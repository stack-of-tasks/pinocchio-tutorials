'''
This script answers to questions 1-2-3 of Tuto 2: given a reference translation <target> , 
it computes the configuration of the UR5 so that the end-effector position (3D) matches the
target. This is done using BFGS solver. While iterating to compute the optimal configuration,
the script also display the successive candidate solution, hence moving the robot from the initial 
guess configuaration to the reference target.
'''

import pinocchio as se3
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *
from pinocchio.explog import log
import time
from numpy.linalg import inv
from math import pi
from scipy.optimize import fmin_bfgs, fmin_slsqp


# --- Load robot model
pkg   = '/home/student/models/'
urdf  = pkg + 'ur_description/urdf/ur5_gripper.urdf'
robot = RobotWrapper( urdf, [ pkg, ] )
robot.initDisplay( loadModel = True)
if 'viewer' not in robot.__dict__: robot.initDisplay()

# Define an init config
robot.q0 = np.matrix([0, -pi/2, 0, 0, 0, 0])

NQ = robot.model.nq
NV = robot.model.nv

# --- Add ball to represent target
try:
     robot.viewer.gui.addSphere("world/red", .05, [1.,.2,.2,.5]) # .1 is the radius
except:
    print "Ball already exists in viewer ... skip"

try:
     robot.viewer.gui.addSphere("world/blue", .05, [.2,.2,1.,.5]) # .1 is the radius
except:
    print "Ball already exists in viewer ... skip"

# Shortcut function to convert SE3 to 7-dof vector.
M2gv      = lambda M: se3ToXYZQUAT(M)
def place(objectId,M):
     robot.viewer.gui.applyConfiguration(objectId, M2gv(M))
     robot.viewer.gui.refresh() # Refresh the window.

### 
### OPTIM 3D #########################################################
###

class Cost:
     '''Functor class computing the distance of robot effector to target.'''
     def __init__(self,pdes):
          self.pdes = pdes
     def __call__(self,q):
          '''Compute score from a configuration'''
          p = robot.position(q, 6).translation
          return np.linalg.norm(p-self.pdes)

class CallbackLogger:
     def __init__(self, sleeptime=1e-2):
          """
          nfevl: iteration number
          dt: animation time pause
          """
          self.nfeval = 1
          self.dt = sleeptime
     def __call__(self, q):
          print '===CBK=== {0:4d}   {1: 3.2f}   {2: 3.2f}'.format(self.nfeval, q[0], q[1])
          robot.display(q)
          place('world/blue',robot.position(q, 6))
          self.nfeval += 1
          time.sleep(self.dt)

## Question 1-2-3
target = np.matrix([0.5, 0.1, 0.2 ]).T  # x,y,z
place('world/red',se3.SE3(eye(3),target))
cost = Cost(target)
robot.display(robot.q0)
time.sleep(1)
print 'Let s go to pdes.'
qopt = fmin_bfgs(cost, robot.q0, callback=CallbackLogger(.5))
