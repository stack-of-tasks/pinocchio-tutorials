'''
Inverse kinematics (close loop / iterative) for a mobile manipulator.
Achieve two tasks: first place the end effector to a reference target. Then position
the basis to a reference line.
'''

import pinocchio as se3
from mobilerobot import MobileRobotWrapper,M2gv
from pinocchio.utils import *
import time
from numpy.linalg import pinv

### Load robot model.
pkg = '/home/student/models/'
urdf = pkg + 'ur_description/urdf/ur5_gripper.urdf'
                     
robot = MobileRobotWrapper(urdf,[pkg,])
robot.initDisplay(loadModel=False)

NQ = robot.model.nq
NV = robot.model.nv
IDX_TOOL  = 24
IDX_BASIS = 23

### Set up display environment.
def place(name,M):
    robot.viewer.gui.applyConfiguration(name,M2gv(M))
    robot.viewer.gui.refresh()

def Rquat(x,y,z,w):
    q = se3.Quaternion(x,y,z,w)
    q.normalize()
    return q.matrix()

Mgoal = se3.SE3(Rquat(0.4,0.02, -0.5,0.7),
                np.matrix([.2,-.4,.7]).T)
try:
    robot.viewer.gui.addXYZaxis('world/framegoal',[1.,0.,0.,1.],.015,.2)
    robot.viewer.gui.addCylinder('world/yaxis',.01,20,[0.1,0.1,0.1,1.])
except:
    pass
place('world/framegoal',Mgoal)
place('world/yaxis',se3.SE3(rotate('x',np.pi/2),
                            np.matrix([0,0,.1]).T))

### Question 1: servo end-effector

# Define robot initial configuration
q  = se3.randomConfiguration(robot.model,zero(NQ),zero(NQ)+np.pi)
q[:2] = 0  # Basis at the center of the world.

DT = 1e-2  # Integration step.

print 'Servo end effector placement'
# Loop on an inverse kinematics for 200 iterations.
for i in range(200):
      se3.forwardKinematics(robot.model,robot.data,q)   # Compute joint placements
      se3.framesKinematics(robot.model,robot.data)      # Also compute operational frame placements
      Mtool = robot.data.oMf[IDX_TOOL]                  # Get placement from world frame o to frame f oMf
      nu    = se3.log(Mtool.inverse()*Mgoal).vector     # Compute needed displacement
      J     = se3.frameJacobian(robot.model,robot.data,q,IDX_TOOL,True,True) # Get corresponding jacobian
      vq    = pinv(J)*nu
      robot.increment(q,vq*DT)
      robot.display(q)
      time.sleep(DT)

print "Residual error on the tool = ", np.linalg.norm(nu)

### Question 2: servo end effector and basis

print 'Servo end effector placement and basis X translation.'
# Loop on 2-objectives inverse kinematics for 1000 iterations.
for i in range(1000):
      se3.forwardKinematics(robot.model,robot.data,q)   # Compute joint placements
      se3.framesKinematics(robot.model,robot.data)      # Also compute operational frame placements

      Mtool = robot.data.oMf[IDX_TOOL]                  # Get placement from world frame o to frame f oMf
      nu1   = se3.log(Mtool.inverse()*Mgoal).vector     # Compute needed displacement
      J1    = se3.frameJacobian(robot.model,robot.data,q,IDX_TOOL,True,True) # Get corresponding jacobian

      Mbasis = robot.data.oMf[IDX_BASIS]                # Get basis placement.
      nu2    = -Mbasis.translation[0]                   # Only keep translation part.
      J2     = se3.frameJacobian(robot.model, robot.data, q, IDX_BASIS,False,True)[0,:] # Corresp. jacobian
      
      # Control law from equal-priority objectives
      #vq     = pinv( np.vstack([J1,J2]) ) * np.vstack([nu1,nu2])
      # Control law from hierarchy of objectives
      Jp1 = pinv(J1)
      vq  = Jp1*nu1
      P1  = eye(NV) - Jp1*J1
      Jp2 = pinv(J2*P1)
      vq += Jp2*(nu2 - J2*vq)
      
      robot.increment(q,vq*DT)
      robot.display(q)
      time.sleep(DT)

print "Residual error on the tool = ", np.linalg.norm(nu)
print "Residual error on the basis = ", Mbasis.translation[0] 

    




