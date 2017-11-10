'''
This file loads 4 urdf models of the robot UR5 and places the basis of theses robots
to form a parallel robot structure.
'''

import pinocchio as se3
from pinocchio import SE3
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *
from scipy.optimize import fmin_bfgs, fmin_slsqp
from pinocchio.explog import log

# Simple helper function
M2gv = lambda M: se3ToXYZQUAT(M)
def place(objectId,M):
     gepettoViewer.applyConfiguration(objectId, M2gv(M))
     gepettoViewer.refresh() # Refresh the window.

###
### Load the robot model #############################################
###
path = '/home/student/models/'
urdf = path+ '/ur_description/urdf/ur5_gripper.urdf'

def loadRobot(M0,name):
    '''
    This function load a UR5 robot n a new model, move the basis to placement <M0>
    and add the corresponding visuals in gepetto viewer with name prefix given by string <name>.
    It returns the robot wrapper (model,data).
    '''
    robot = RobotWrapper(urdf,[path,])
    robot.model.jointPlacements[1] = M0*robot.model.jointPlacements[1]
    robot.visual_model.geometryObjects[0].placement = M0*robot.visual_model.geometryObjects[0].placement
    robot.visual_data.oMg[0] = M0*robot.visual_data.oMg[0]
    robot.initDisplay(loadModel=True,viewerRootNodeName="world/"+name)
    if 'viewer' not in robot.__dict__:  robot.initDisplay(viewerRootNodeName="world/"+name)
    return robot

robots = []
# Load 4 Ur5 robots, placed at 0.3m from origin in the 4 directions x,y,-x,-y.
Mt = SE3(eye(3),np.matrix([.3,0,0]).T)  # First robot is simply translated
for i in range(4):
    robots.append(loadRobot(SE3(rotate('z',np.pi/2*i),zero(3))*Mt,"robot%d"%i))
gepettoViewer = robots[0].viewer.gui
# Set up the robots configuration with end effector pointed upward.
q0 = np.matrix([np.pi/4,-np.pi/4,-np.pi/2,np.pi/4,np.pi/2,0]).T
for i in range(4):
    robots[i].display(q0)

###
### Set up the effector ##############################################
###

# Add a new object featuring the parallel robot tool plate.
[w,h,d] = [0.25,0.25,0.005]
color   = [red,green,blue,transparency] = [1,1,0.78,1.0]
try:
    gepettoViewer.addBox('world/toolplate', w,h,d,color)
except:
    pass
# Set and save initial positions
oMtool = SE3(rotate('z',1.268), np.matrix([0,0,.77]).T)
place('world/toolplate',oMtool)

# Store the transformation between each effector and the tool plate
toolMeff = []
for robot in robots:
    oMeff = robot.position(q0,6)
    toolMeff.append ( oMtool.inverse()*oMeff )

# Now define the reference target for the plate
oMtool_des = SE3(rotate('z',1.)*rotate('x',.2), np.matrix([0.1,0.02,.65]).T)


### 
### OPTIM 6D #########################################################
###

class Cost:
     '''Functor class computing the distance of robot effector to target.'''
     def __init__(self,robot,oMdes):
         self.robot = robot
         self.oMdes = oMdes
     def __call__(self,q):
          '''Compute score from a configuration'''
          oMe = self.robot.position(q, 6)
          return np.linalg.norm( se3.log(oMe.inverse()*self.oMdes).vector )

### Question 10
# Solve 4 times the 6D inverse-geometry problem.
place('world/toolplate',oMtool_des)
qopt = []
for i,r in enumerate(robots):
    qopt.append( fmin_bfgs(Cost(r,oMtool_des*toolMeff[i]), q0,disp=False) )
    r.display(qopt[-1])
