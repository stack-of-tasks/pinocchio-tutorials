'''
Simple contact simulator for normal contact (no friction allowed), hence implemented as a QP.
The main loop compute the current torque from a PD over a moving reference configuration.
Then solve a QP whose constraints are the nonpenetration.
Finally integrates it with Euler integrator.
'''

from robot_hand import Robot
from time import sleep
import pinocchio as se3
from pinocchio.utils import *
from numpy import cos,sin,pi,hstack,vstack,argmin,sqrt
from numpy.linalg import norm,pinv,inv
from quadprog import solve_qp

robot = Robot()
robot.display(robot.q0)

q  = rand(robot.model.nq)
vq = zero(robot.model.nv)

DT = 1e-3
Kp = 10.0
Kv = 2*np.sqrt(Kp)

robot.collisionPairs.append([2,8])
robot.collisionPairs.append([2,11])
robot.collisionPairs.append([2,14])
robot.collisionPairs.append([2,16])

def qdes(t):
    '''Compute a reference position for time <t>.'''
    qdes    = robot.q0.copy()
    qdes[2] = 1.5*cos(-t)-1
    qdes[3] = 1.5*cos(-t)-1
    qdes[5] = 1.5*cos(-t/1.5)-1
    qdes[6] = 1.5*cos(-t/1.5)-1
    qdes[8] = 1.5*cos(-t/2.5)-1
    qdes[9] = 1.5*cos(-t/2.5)-1
    return qdes

# Simulation loop.
for i in range(100000):

    M = se3.crba(robot.model,robot.data,q)
    b = se3.rnea(robot.model,robot.data,q,vq,zero(robot.model.nv))

    tauq = -Kp*(q-qdes(i*DT)) - Kv*vq

    # Collision test.
    active = 0
    Js = []
    dists = []
    # Check all pairs to list the current collisions.
    for idx  in range(len(robot.collisionPairs)):
        dist = robot.checkCollision(idx)
        if dist<1e-3:
            dists.append(dist)
            Js.append(robot.collisionJacobian(idx,q))
            robot.displayCollision(idx,active)
            active += 1

    # The dynamics is differently computed depending free or in-contact situation.
    if len(Js)==0:
        # If no collision: simply inverse the mass matrix.
        aq = inv(M)*(tauq-b)
    else:
        # If collision: setup a QP problem.

        # Constraint jacobian, obtained by stacking all normals.
        J = vstack(Js)
        
        # If non-zero normal velocity: cancel it with an impact model.
        if norm(J*vq)>1e-2:
            vq -= pinv(J)*J*vq
            print "*"*10+ "Impact!!!"

        # Define the QP matrices/vectors, as np.arrays.
        H = np.asarray(M)
        g = np.asarray(tauq-b).T[0]
        C = np.asarray(J).T
        d = -np.array(dists)

        # Solve the problem
        x,_,_,_,_,iact=solve_qp(H,g,C,d)
        aq = np.matrix(x).T
        print 'Contact'+str(i)

        # If a constraint is not active = contact will be broken at next time.
        # Hide the contact patches in Gepetto Viewer.
        for i in range(active):
            if i not in iact.tolist():
                robot.viewer.viewer.gui.setVisibility('world/wa'+str(active),'OFF')
                robot.viewer.viewer.gui.setVisibility('world/wb'+str(active),'OFF')

    # Integrate the resulting acceleration.
    vq += aq*DT
    q   = se3.integrate(robot.model,q,vq*DT)

    # Display.
    robot.display(q)
    sleep(1e-4)

