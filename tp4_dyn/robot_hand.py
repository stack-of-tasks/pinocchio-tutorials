from pinocchio.utils import *
from pinocchio.explog import exp,log
from numpy.linalg import pinv,norm
import pinocchio as se3
import gepetto.corbaserver
from display import Display
from numpy import pi
from numpy import cos,sin,pi,hstack,vstack,argmin
from numpy.linalg import norm,pinv

class Visual:
    '''
    Class representing one 3D mesh of the robot, to be attached to a joint. The class contains:
    * the name of the 3D objects inside Gepetto viewer.
    * the ID of the joint in the kinematic tree to which the body is attached.
    * the placement of the body with respect to the joint frame.
    This class is only used in the list Robot.visuals (see below).

    The visual are supposed mostly to be capsules. In that case, the object also contains
    radius and length of the capsule.
    The collision checking computes collision test, distance, and witness points.
    Using the supporting robot, the collision Jacobian returns a 1xN matrix corresponding
    to the normal direction.
    '''
    def __init__(self,name,jointParent,placement,radius=.1,length=None):
        '''Length and radius are used in case of capsule objects'''
        self.name = name                  # Name in gepetto viewer
        self.jointParent = jointParent    # ID (int) of the joint 
        self.placement = placement        # placement of the body wrt joint, i.e. bodyMjoint
        if length is not None:
            self.length = length
            self.radius = radius
    def place(self,display,oMjoint):
        oMbody = oMjoint*self.placement
        display.place(self.name,oMbody,False)

    def isCapsule(self):
        return 'length' in self.__dict__ and 'radius' in self.__dict__

    def collision(self,c2,data=None,oMj1=None,oMj2=None):
        if data is not None:
            oMj1 = data.oMi[self.jointParent]
            oMj2 = data.oMi[c2  .jointParent]
        M1 = oMj1*self.placement
        M2 = oMj2*c2  .placement

        assert(self.isCapsule() and c2.isCapsule())
        l1 = self.length
        r1 = self.radius
        l2 = c2  .length
        r2 = c2  .radius

        a1 = M1.act(np.matrix([0,0,-l1/2]).T)
        b1 = M2.act(np.matrix([0,0,-l2/2]).T)
        a2 = M1.act(np.matrix([0,0, l1/2]).T)
        b2 = M2.act(np.matrix([0,0, l2/2]).T)

        ab = pinv(hstack([a1-a2,b2-b1]))*(b2-a2)

        if all(ab>=0) and all(ab<=1):
            asat = bsat = False
            pa = a2 + ab[0,0]*(a1-a2)
            pb = b2 + ab[1,0]*(b1-b2)
        else:
            asat = bsat = True
            i = argmin( vstack([ ab,1-ab] ) )

            pa = a2 if i==0 else a1
            pb = b2 if i==1 else b1
            if i==0 or i==2:  # fix a to pa, search b
                b = (pinv(b1-b2)*(pa-b2))[0,0]
                if b<0:
                    pb = b2
                elif b>1:
                    pb = b1
                else:
                    pb = b2 + b*(b1-b2)
                    bsat = False
            else: # fix b
                a = (pinv(a1-a2)*(pb-a2))[0,0]
                if a<0:
                    pa = a2
                elif a>1:
                    pa = a1
                else:
                    pa = a2 + a*(a1-a2)
                    asat = False

        dist = norm(pa-pb)-(r1+r2)
        if norm(pa-pb)>1e-3:
            # Compute witness points
            ab = pa-pb
            ab /= norm(ab)
            wa = pa - ab*r1
            wb = pb + ab*r2
    
            # Compute normal matrix
            x = np.matrix([1.,0,0]).T
            r1 = cross(ab, x)
            if norm(r1)<1e-2:
                x = np.matrix([0,1.,0]).T
            r1 = cross(ab, x)
            r1 /= norm(r1)
            r2 = cross(ab,r1)
            R = hstack([r1,r2,ab])

            self.dist = dist
            c2  .dist = dist
            self.w = wa
            c2  .w = wb
            self.R = R
            c2  .R = R

        return dist

    def jacobian(self,c2,robot,q):
        Ja = se3.jacobian(robot.model,robot.data,q,self.jointParent,False,True)
        Jb = se3.jacobian(robot.model,robot.data,q,c2  .jointParent,False,True)
    
        Xa = se3.SE3(self.R,self.w).action
        Xb = se3.SE3(c2  .R,c2  .w).action

        J = (Xa*Ja)[2,:] - (Xb*Jb)[2,:]
        return J

    def displayCollision(self,viewer,name='world/wa'):
        viewer.viewer.gui.setVisibility(name,'ON')
        viewer.place(name,se3.SE3(self.R,self.w))

class Robot:
    '''
    Define a class Robot with 7DOF (shoulder=3 + elbow=1 + wrist=3). 
    The configuration is nq=7. The velocity is the same. 
    The members of the class are:
    * viewer: a display encapsulating a gepetto viewer client to create 3D objects and place them.
    * model: the kinematic tree of the robot.
    * data: the temporary variables to be used by the kinematic algorithms.
    * visuals: the list of all the 'visual' 3D objects to render the robot, each element of the list being
    an object Visual (see above).
    
    CollisionPairs is a list of visual indexes. 
    Reference to the collision pair is used in the collision test and jacobian of the collision
    (which are simply proxy method to methods of the visual class).
    '''

    def __init__(self):
        self.viewer = Display()
        self.visuals = []
        self.model = se3.Model.BuildEmptyModel()
        self.createHand()
        self.data = self.model.createData()
        self.q0 = zero(self.model.nq)
        #self.q0[3] = 1.0
        self.v0 = zero(self.model.nv)
        self.collisionPairs = []

    def createHand(self,rootId=0,prefix='',jointPlacement=None):
        color   = [red,green,blue,transparency] = [1,1,0.78,1.0]
        colorred = [1.0,0.0,0.0,1.0]

        jointId = rootId

        cm = 1e-2
        trans = lambda x,y,z: se3.SE3(eye(3),np.matrix([x,y,z]).T)
        inertia = lambda m,c: se3.Inertia(m,np.matrix(c,np.double).T,eye(3)*m**2)

        name               = prefix+"wrist"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = jointPlacement if jointPlacement!=None else se3.SE3.Identity()
        jointId = self.model.addJoint(jointId,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,inertia(3,[0,0,0]),se3.SE3.Identity())
        

        L=3*cm;W=5*cm;H=1*cm
        try:self.viewer.viewer.gui.addSphere('world/'+prefix+'wrist', .02, color)
        except: pass
 
        try: self.viewer.viewer.gui.addBox('world/'+prefix+'wpalm', L/2, W/2, H,color)
        except: pass
        self.visuals.append( Visual('world/'+prefix+'wpalm',jointId,trans(L/2,0,0) ))
        capsr = H; capsl = W
        try:self.viewer.viewer.gui.addCapsule('world/'+prefix+'wpalmb', capsr, capsl, color)
        except: pass
        self.visuals.append( Visual('world/'+prefix+'wpalmb',jointId,
                                    se3.SE3(rotate('x',pi/2),zero(3)),capsr,capsl )) 
        capsr = H; capsl = W
        try:self.viewer.viewer.gui.addCapsule('world/'+prefix+'wpalmt', capsr, capsl, color)
        except: pass
        self.visuals.append( Visual('world/'+prefix+'wpalmt',jointId,
                                    se3.SE3(rotate('x',pi/2),np.matrix([L,0,0]).T),capsr,capsl )) 
        capsr = H; capsl = L
        try:self.viewer.viewer.gui.addCapsule('world/'+prefix+'wpalml', H, L, color)
        except: pass
        self.visuals.append( Visual('world/'+prefix+'wpalml',jointId,
                                    se3.SE3(rotate('y',pi/2),np.matrix([L/2,-W/2,0]).T),capsr,capsl )) 
        capsr = H; capsl = L
        try:self.viewer.viewer.gui.addCapsule('world/'+prefix+'wpalmr', H, L, color)
        except: pass
        self.visuals.append( Visual('world/'+prefix+'wpalmr',jointId,
                                    se3.SE3(rotate('y',pi/2),np.matrix([L/2,W/2,0]).T),capsr,capsl )) 


        name               = prefix+"palm"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(eye(3), np.matrix([5*cm,0,0]).T)
        jointId = self.model.addJoint(jointId,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,inertia(2,[0,0,0]),se3.SE3.Identity())
        capsr = 1*cm; capsl = W
        try:self.viewer.viewer.gui.addCapsule('world/'+prefix+'palm2', 1*cm, W, color)
        except: pass
        self.visuals.append( Visual('world/'+prefix+'palm2',jointId,
                                    se3.SE3(rotate('x',pi/2),zero(3)),capsr,capsl )) 

        FL = 4*cm
        palmIdx = jointId

        name               = prefix+"finger11"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(eye(3), np.matrix([2*cm,W/2,0]).T)
        jointId = self.model.addJoint(palmIdx,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,inertia(.5,[0,0,0]),se3.SE3.Identity())
        capsr = H; capsl = FL-2*H
        try:self.viewer.viewer.gui.addCapsule('world/'+prefix+'finger11', H, FL-2*H, color)
        except: pass
        self.visuals.append( Visual('world/'+prefix+'finger11',jointId,
                                    se3.SE3(rotate('y',pi/2),np.matrix([FL/2-H,0,0]).T),capsr,capsl ))

        name               = prefix+"finger12"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(eye(3), np.matrix([FL,0,0]).T)
        jointId = self.model.addJoint(jointId,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,inertia(.5,[0,0,0]),se3.SE3.Identity())
        capsr = H; capsl = FL-2*H
        try:self.viewer.viewer.gui.addCapsule('world/'+prefix+'finger12', H, FL-2*H, color)
        except: pass
        self.visuals.append( Visual('world/'+prefix+'finger12',jointId,
                                    se3.SE3(rotate('y',pi/2),np.matrix([FL/2-H,0,0]).T),capsr,capsl ))

        name               = prefix+"finger13"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(eye(3), np.matrix([FL-2*H,0,0]).T)
        jointId = self.model.addJoint(jointId,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,inertia(.3,[0,0,0]),se3.SE3.Identity())
        capsr = H; capsl = 0.;
        try:self.viewer.viewer.gui.addSphere('world/'+prefix+'finger13', capsr, color)
        except: pass
        self.visuals.append( Visual('world/'+prefix+'finger13',jointId,
                                    trans(2*H,0,0),capsr,capsl ))

        name               = prefix+"finger21"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(eye(3), np.matrix([2*cm,0,0]).T)
        jointId = self.model.addJoint(palmIdx,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,inertia(.5,[0,0,0]),se3.SE3.Identity())
        capsr = H; capsl = FL-2*H
        try:self.viewer.viewer.gui.addCapsule('world/'+prefix+'finger21', H, FL-2*H, color)
        except: pass
        self.visuals.append( Visual('world/'+prefix+'finger21',jointId,
                                    se3.SE3(rotate('y',pi/2),np.matrix([FL/2-H,0,0]).T),capsr,capsl ))

        name               = prefix+"finger22"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(eye(3), np.matrix([FL,0,0]).T)
        jointId = self.model.addJoint(jointId,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,inertia(.5,[0,0,0]),se3.SE3.Identity())
        capsr = H; capsl = FL-2*H
        try:self.viewer.viewer.gui.addCapsule('world/'+prefix+'finger22', H, FL-2*H, color)
        except: pass
        self.visuals.append( Visual('world/'+prefix+'finger22',jointId,
                                    se3.SE3(rotate('y',pi/2),np.matrix([FL/2-H,0,0]).T),capsr,capsl ))

        name               = prefix+"finger23"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(eye(3), np.matrix([FL-H,0,0]).T)
        jointId = self.model.addJoint(jointId,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,inertia(.3,[0,0,0]),se3.SE3.Identity())
        capsr = H; capsl = 0.;
        try:self.viewer.viewer.gui.addSphere('world/'+prefix+'finger23', H, color)
        except: pass
        self.visuals.append( Visual('world/'+prefix+'finger23',jointId,
                                    trans(H,0,0),capsr,capsl ))


        name               = prefix+"finger31"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(eye(3), np.matrix([2*cm,-W/2,0]).T)
        jointId = self.model.addJoint(palmIdx,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,inertia(.5,[0,0,0]),se3.SE3.Identity())
        capsr = H; capsl = FL-2*H
        try:self.viewer.viewer.gui.addCapsule('world/'+prefix+'finger31', H, FL-2*H, color)
        except: pass
        self.visuals.append( Visual('world/'+prefix+'finger31',jointId,
                                    se3.SE3(rotate('y',pi/2),np.matrix([FL/2-H,0,0]).T),capsr,capsl ))

        name               = prefix+"finger32"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(eye(3), np.matrix([FL,0,0]).T)
        jointId = self.model.addJoint(jointId,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,inertia(.5,[0,0,0]),se3.SE3.Identity())
        capsr = H; capsl = FL-2*H
        try:self.viewer.viewer.gui.addCapsule('world/'+prefix+'finger32', H, FL-2*H, color)
        except: pass
        self.visuals.append( Visual('world/'+prefix+'finger32',jointId,
                                    se3.SE3(rotate('y',pi/2),np.matrix([FL/2-H,0,0]).T),capsr,capsl ))

        name               = prefix+"finger33"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(eye(3), np.matrix([FL-2*H,0,0]).T)
        jointId = self.model.addJoint(jointId,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,inertia(.3,[0,0,0]),se3.SE3.Identity())
        capsr = H; capsl = 0.
        try:self.viewer.viewer.gui.addSphere('world/'+prefix+'finger33', H, color)
        except: pass
        self.visuals.append( Visual('world/'+prefix+'finger33',jointId,
                                    trans(2*H,0,0),capsr,capsl ))

        name               = prefix+"thumb1"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(eye(3), np.matrix([1*cm,-W/2-H*1.5,0]).T)
        jointId = self.model.addJoint(1,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,inertia(.5,[0,0,0]),se3.SE3.Identity())
        capsr = H; capsl = 2*cm
        try:self.viewer.viewer.gui.addCapsule('world/'+prefix+'thumb1', H, 2*cm, color)
        except: pass
        self.visuals.append( Visual('world/'+prefix+'thumb1',jointId,
                                    se3.SE3(rotate('z',pi/3)*rotate('x',pi/2),np.matrix([1*cm,-1*cm,0]).T),
                                    capsr,capsl ))

        name               = prefix+"thumb2"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(rotate('z',pi/3)*rotate('x',pi), np.matrix([3*cm,-1.8*cm,0]).T)
        jointId = self.model.addJoint(jointId,se3.JointModelRZ(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,inertia(.4,[0,0,0]),se3.SE3.Identity())
        capsr = H; capsl = FL-2*H
        try:self.viewer.viewer.gui.addCapsule('world/'+prefix+'thumb2', H, FL-2*H, color)
        except: pass
        self.visuals.append( Visual('world/'+prefix+'thumb2',jointId,
                                    se3.SE3(rotate('x',pi/3),np.matrix([-0.7*cm,.8*cm,-0.5*cm]).T),
                                    capsr,capsl ))

        # Prepare some patches to represent collision points. Yet unvisible.
        for i in range(10):
            try:
                self.viewer.viewer.gui.addCylinder('world/wa'+str(i), .01, .003, [ 1.0,0,0,1])
                self.viewer.viewer.gui.addCylinder('world/wb'+str(i), .01, .003, [ 1.0,0,0,1])
            except: pass
            self.viewer.viewer.gui.setVisibility('world/wa'+str(i),'OFF')
            self.viewer.viewer.gui.setVisibility('world/wb'+str(i),'OFF')

  
    def checkCollision(self,pairIndex):
        ia,ib = self.collisionPairs[pairIndex]
        va = self.visuals[ia]
        vb = self.visuals[ib]
        dist = va.collision(vb,self.data)
        return dist

    def collisionJacobian(self,pairIndex,q):
        ia,ib = self.collisionPairs[pairIndex]
        va = self.visuals[ia]
        vb = self.visuals[ib]
        return va.jacobian(vb,self,q)
        
    def displayCollision(self,pairIndex,meshIndex,onlyOne=False):
        ia,ib = self.collisionPairs[pairIndex]
        va = self.visuals[ia]
        vb = self.visuals[ib]
        va.displayCollision(self.viewer,'world/wa'+str(meshIndex))
        vb.displayCollision(self.viewer,'world/wb'+str(meshIndex))
        self.viewer.viewer.gui.setVisibility('world/wa'+str(meshIndex),'ON')
        self.viewer.viewer.gui.setVisibility('world/wb'+str(meshIndex),'ON')

    def display(self,q):
        se3.forwardKinematics(self.model,self.data,q)
        for visual in self.visuals:
            visual.place( self.viewer,self.data.oMi[visual.jointParent] )
        self.viewer.viewer.gui.refresh()
        

