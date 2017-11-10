'''
This script is a variation over the script optim_translation.py, that should be
red first.
While the initial script optimize the robot configuration to match a given position (3D),
this script optimizes the configuration of the arm so that the end-effector placement (6D, 
position+rotation) matches the reference target. 
The script also display the successive candidate solution during the optimization.
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
robot.initDisplay( loadModel = False)
if 'viewer' not in robot.__dict__: robot.initDisplay()

# Define an init config
robot.q0 = np.matrix([0, -pi/2, 0, 0, 0, 0])

NQ = robot.model.nq
NV = robot.model.nv

# --- Add ball to represent target
try:
     robot.viewer.gui.addXYZaxis("world/red", [1.,.2,.2,.5], .02, .1)
except:
    print "Ball already exists in viewer ... skip"

try:
     robot.viewer.gui.addXYZaxis("world/blue", [.2,.2,1.,.5], .02, .1)
except:
    print "Ball already exists in viewer ... skip"

# Shortcut function to convert SE3 to 7-dof vector.
M2gv      = lambda M: se3ToXYZQUAT(M)
def place(objectId,M):
     robot.viewer.gui.applyConfiguration(objectId, M2gv(M))
     robot.viewer.gui.refresh() # Refresh the window.

### 
### OPTIM 6D #########################################################
###

class Cost:
     '''Functor class computing the distance of robot effector to target.'''
     def __init__(self,oMdes):
          self.oMdes = oMdes
     def __call__(self,q):
          '''Compute score from a configuration'''
          oMe = robot.position(q, 6)
          return np.linalg.norm( se3.log(oMe.inverse()*self.oMdes).vector )

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

## Question 4
target = se3.SE3( rotate('x',.4),
                  np.matrix([0.5, 0.1, 0.2 ]).T )

place('world/red',target)
qopt = fmin_bfgs(Cost(target), robot.q0, callback=CallbackLogger(1.0))




