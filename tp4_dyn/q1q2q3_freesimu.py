from robot_hand import Robot
from time import sleep
import pinocchio as se3
from pinocchio.utils import *

robot = Robot()
robot.display(robot.q0)

try:
    for i in range(10):
        robot.viewer.viewer.gui.addCylinder('world/w'+str(i), .01, .003, [ 1.0,0,0,1])
        robot.viewer.viewer.gui.setVisibility('world/w'+str(i),'OFF')
except: pass

q = robot.q0.copy()
vq = zero(robot.model.nv)


dt = 1e-3
Kf = 0e-1

qdes  = robot.q0.copy()
Kp = 10.0
Kv = 2*np.sqrt(Kp)

q=rand(robot.model.nq)

for i in range(2000,100000):

    M = se3.crba(robot.model,robot.data,q)
    b = se3.rnea(robot.model,robot.data,q,vq,zero(robot.model.nv))

    ### Q2: free dynamics with friction and gravity.
    #tauq = -Kf*vq 

    ### Q3: implement a PD tracking a reference configuration
    tauq = -Kp*(q-qdes) - Kv*vq

    aq  = np.linalg.inv(M)*(tauq-b)

    vq += aq*dt
    q   = se3.integrate(robot.model,q,vq*dt)

    if not i % 3:
        robot.display(q)
        sleep(1e-4)

