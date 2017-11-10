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

# Shortcut function to convert SE3 to 7-dof vector.
M2gv      = lambda M: se3ToXYZQUAT(M)
def place(objectId,M):
     robot.viewer.gui.applyConfiguration(objectId, M2gv(M))
     robot.viewer.gui.refresh() # Refresh the window.

### 
### Obstacle map
###
# Capsule obstacles will be placed at these XYZ-RPY parameters
oMobs = [ [ 0.40,  0.20,  0.30, 0.10,  0.30, 0.00 ],
          [-0.03, -0.23,  0.69, 0.52, -0.17, 0.86],
          [ 0.13, -0.62,  0.04, 0.53,  0.35, 0.34],
          [-0.32,  0.22, -0.08, 0.66, -0.58, 0.73]]

# Load visual objects and add them in collision/visual models
for i,xyzrpy in enumerate(oMobs):
    rad,length = .1,.5                                  # radius and length of capsules
    obs = se3.GeometryObject.CreateCapsule(rad,length)  # Pinocchio obstacle object
    obs.name = "obs%d"%i                                # Set object name
    obs.parentJoint = 0                                 # Set object parent = 0 = universe
    obs.placement = se3.SE3( rotate('x',xyzrpy[3])*rotate('y',xyzrpy[4])*rotate('z',xyzrpy[5]), 
                             np.matrix(xyzrpy[:3]).T )  # Set object placement wrt parent
    robot.collision_model.addGeometryObject(obs,robot.model,False)  # Add object to collision model
    robot.visual_model   .addGeometryObject(obs,robot.model,False)  # Add object to visual model
    # Also create a geometric object in gepetto viewer, with according name.
    try:     robot.viewer.gui.addCapsule( "world/pinocchio/"+obs.name, rad,length, [ 1.0, 0.2, 0.2, 1.0 ] )
    except:  pass

# Add all collision pairs
robot.collision_model.addAllCollisionPairs()
# Remove collision pairs that can not be relevant (forearm with upper arm, forearm with wrist, etc).
for idx in [ 56,35,23 ]:
    #print "Remove pair",robot.collision_model.collisionPairs[idx]
    robot.collision_model.removeCollisionPair(robot.collision_model.collisionPairs[idx])

# Collision/visual models have been modified => re-generate corresponding data.
robot.collision_data = se3.GeometryData(robot.collision_model)
robot.visual_data    = se3.GeometryData(robot.visual_model   )

# Display the robot will also update capsule placements.
for iloop in range(100):
    q = rand(6)*2*np.pi-np.pi

    se3.updateGeometryPlacements(robot.model,robot.data,robot.collision_model,robot.collision_data,q)
    if se3.computeCollisions(robot.collision_model,robot.collision_data,False):
        for i,p in enumerate(robot.collision_model.collisionPairs):
            if se3.computeCollision(robot.collision_model,robot.collision_data,i):
                print i,p, robot.collision_model.geometryObjects[p.first].name, \
                    robot.collision_model.geometryObjects[p.second].name
        robot.display(q)
        break
