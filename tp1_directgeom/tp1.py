import pinocchio as se3
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *
from time import sleep
from numpy.linalg import inv
import math

pkg = '/home/student/models/'
urdf = pkg + 'ur_description/urdf/ur5_gripper.urdf'
                     
robot = RobotWrapper( urdf, [ pkg, ] )
robot.initDisplay( loadModel = True )
if 'viewer' not in robot.__dict__: robot.initDisplay()

NQ = robot.model.nq
NV = robot.model.nv

### 
### PICK #############################################################
###

# Add a red box
boxID = "world/box"
rgbt =  [ 1.0, 0.2, 0.2, 1.0 ] # red-green-blue-transparency
try:
    robot.viewer.gui.addBox( boxID, 0.05, 0.1, 0.1, rgbt ) # id, dim1, dim2, dim3, color
except:
    print "Box already exists in viewer ... skip"

# Place the box at the position ( 0.5, 0.1, 0.2 )
q_box = [ 0.5, 0.1, 0.2, 1, 0, 0, 0 ]
robot.viewer.gui.applyConfiguration( boxID, q_box )
robot.viewer.gui.refresh()

# Configuration for picking the box
q = zero( NQ )
q[ 0 ]    = -0.375
q[ 1 ]    = -1.2
q[ 2 ]    =  1.71
q[ 3 ]    = -q[ 1 ] - q[ 2 ]
q[ 4 ]    =  q[ 0 ]

robot.display(q)
print "The robot is display with end effector on the red box."
sleep( 2 )

###
### MOVE #############################################################
###

print "Let's start the movement ..."

# Random velocity of the robot driving the movement
vq = zero( NV )
vq[ 0 ]   = 2
vq[ 3 ]   = 4

idx       = robot.index( 'wrist_3_joint' )
oMeff     = robot.position( q, idx )      # Placement of end-eff wrt world at current configuration
oMbox     = XYZQUATToSe3( q_box )         # Placement of box     wrt world
effMbox   = oMeff.inverse()*oMbox         # Placement of box     wrt eff

# Shortcut function to convert SE3 to 7-dof vector.
M2gv      = lambda M: se3ToXYZQUAT(M)

for i in range(10000):
    # Chose new configuration of the robot
    q     += vq / 40
    q[ 2 ] = 1.71 + math.sin( i * 0.05 ) / 2    
    
    # Gets the new position of the box
    oMbox  = robot.position( q, idx )*effMbox
    
    # Display new configuration for robot and box
    robot.viewer.gui.applyConfiguration( boxID, M2gv(oMbox))
    robot.display( q )
    sleep( 0.1 )
