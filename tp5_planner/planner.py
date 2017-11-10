import pinocchio as se3
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *
from time import sleep
from numpy.linalg import inv,norm
import math
import time
import heapq
import random

pkg = '/home/student/models/'
urdf = pkg + 'ur_description/urdf/ur5_gripper.urdf'
                     
robot = RobotWrapper( urdf, [ pkg, ] )
robot.initDisplay( loadModel = True )
if 'viewer' not in robot.__dict__: robot.initDisplay()

NQ = robot.model.nq
NV = robot.model.nv

# Joint limits
QLOW = robot.model.lowerPositionLimit
QUP  = robot.model.upperPositionLimit

# Shortcut function to convert SE3 to 7-dof vector.
M2gv      = lambda M: XYZQUATToViewerConfiguration(se3ToXYZQUAT(M))
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

################################################################################
################################################################################
################################################################################


class Graph:
     def __init__(self):
          self.q        = []
          self.children = {}
          self.connex   = []    # ID of the connex component the node is belonging to
          self.nconnex = 0      # number of connex components
          self.existingConnex = [] # List of existing connex component ID
          
     def addNode(self, q=None, newConnex=False):
          '''
          Create the memory to store a new edge. Initialize all components to None.
          Create an empty list of children.
          '''
          idx = len(self.children)
          self.children[idx] = []
          self.q.append(q)
          self.connex.append(None)
          if newConnex: self.newConnex(idx)

          return idx

     def addEdge(self,first,second,orientation = 0):
          '''
          Add edge from first to second. Also add edge from second to first if orientation
          is null.
          '''
          assert( first in self.children and second in self.children )
          if orientation>=0: self.children[first].append(second)
          if orientation<=0: self.children[second].append(first)

     def newConnex(self,idx):
          '''
          Create a new connex component for node <idx>
          '''
          self.connex[idx] = self.nconnex
          self.existingConnex.append(self.nconnex)
          self.nconnex += 1
     def renameConnex(self,past,future):
          '''
          Change the index of the all the nodes belonging to a connex component.
          Useful when merging two connex components.
          '''
          try:
               self.existingConnex.remove(past)
               self.connex = [ c if c!=past else future for c in self.connex ]
          except:
               pass
     def connexIndexes(self,connex):
          '''Return the list of all node indexes belonging to connex component <connex>.'''
          return [ i for i,c in enumerate(self.connex) if c == connex ]
          
def check(q):
     ''' 
     Check that the robot constraints are satisfied. Return True if satisfied, False otherwise.
     '''
     se3.updateGeometryPlacements(robot.model,robot.data,robot.collision_model,robot.collision_data,q)
     if se3.computeCollisions(robot.collision_model,robot.collision_data,False):
          return False
     if np.any(q<QLOW): return False
     if np.any(q>QUP): return False
     return True
     
def nearestNeighbor(q1, qs, nneighbor = 1, hdistance = lambda q1,q2: norm(q1-q2)  ):
     '''
     Return the indexes <nneighbor> nearest neighbors of new configuration q1.
     q1 is the central node from which distances are computed.
     qs is the list of configuration to search in.
     nneighbor is the number of closest neighbors to return.
     condition is a function of node to filter (ex use it to select only the nodes of 
     one component of the graph).
     <hdistance> defines  the (heuristic) distance function to be used for nearest neighbor algorithm.
     '''
     if len(qs) <= nneighbor: return range(len(qs))
     return np.argpartition([ hdistance(q1,q2) for q2 in qs],
                            nneighbor)[:nneighbor]

def connect(q1,q2,stepLength = 1e-1,store = False):
     '''
     Try to connect q1 to q2, and check constraint satisfaction over a sampling of
     the path with stepLength given in parameter.
     if store, also returns the sampled trajectory
     '''

     traj = []
     res = (lambda x,y: [x,y]) if store else (lambda x,y: x)

     dq = q2-q1
     length  = norm(dq)
     # Avoid a division by small quantity
     if length<stepLength: return res(True,[q1,q2])  

     nsteps = int(length/stepLength)    # number of samples from q1 to q2
     dq /= length

     q = q1.copy()  # q will be moved from q1 to q2, following <stepLength> steps
     for step in range(nsteps):
          q += dq*stepLength            # Move q from q1 to q2
          if store: traj.append(q.copy())
          if not check(q):              # Check robot constraints satisfaction
               return res(False,traj)   # If not satisfied, return false
     
     return res(True,traj)

def simplePrm(nsamples = 1000):
     graph = Graph()
     NCONNECT = 3       # Number of nearest neighbors to try to connect with.
     for _ in range(nsamples):
          q = se3.randomConfiguration(robot.model,QLOW,QUP)
          if not check(q): continue
          idx = graph.addNode()                         # Index of the new node
          graph.q[idx] = q                              # Add a new node for configuration q
          connected = False
          for idx2 in nearestNeighbor(q,graph.q[:-1],NCONNECT):
               if connect(q,graph.q[idx2]):              # Try connect q to new neighbors
                    graph.addEdge(idx2,idx)             # Add a new edge
                    connected = True
          if idx>0 and not connected:
               graph.q = graph.q[:-1]
               del graph.children[idx]
     return graph

def visibityPrm(graph = None, qstart = None, qgoal = None, nsamples = 1000,verbose=True):
     '''
     Create a PRM using visibility heuristic, until either qgoal is connected
     qstart and qgoal are the 2 configurations to connect.
     If <graph> is not defined, a new graph is initialized. Use this argument to increment
     the graph when doing a new request.
     If <qstart> is not set, it is arbitrarily selected as the graph node #0.
     If <qgoal> is set, then algorithm ends before <nsamples> if qgoal is connected to qstart.
     <nsamples> defines how many times the algorithm must iterate before failing.
     '''
     if graph == None:   graph = Graph()
     if qstart is not None:  idxstart = graph.addNode(qstart,newConnex=True)
     else:                   idxstart = 0
     if qgoal  is not None:  idxgoal  = graph.addNode(qgoal, newConnex=True)

     success = False
     for _ in range(nsamples):
          q = se3.randomConfiguration(robot.model,
                                      robot.model.lowerPositionLimit,
                                      robot.model.upperPositionLimit)
          if not check(q): continue             # If new config is in collision, re-sample

          # Try to connect to as many connex components as possible.
          qconnect = []                         # List of possible connections
          for connex in graph.existingConnex:
               iconnex = graph.connexIndexes(connex)               # Indexes of current connex component.
               for idx2 in nearestNeighbor(q,[ graph.q[i] for i in iconnex]):
                    if connect(q,graph.q[iconnex[idx2]]):          # Try connect q to new neighbors.
                         qconnect.append([iconnex[idx2],connex])   # Store idx2.

          # Store established connection if at least one component can be created or destroyed.
          if len(qconnect)==0:                  # Create a new connex component for q alone.
               idx = graph.addNode(q,newConnex=True)
          elif len(qconnect)>1:                 # Destroy at least one component by connecting it.
               if verbose: print 'Connect',qconnect
               idx = graph.addNode(q)           # Create a new node

               # Connect it to the first connex component.
               idx2,c2 = qconnect[0]            # Recover first component of the list
               graph.connex[idx] = c2           # Add the new node to the first component
               graph.addEdge(idx,idx2)          # Create new edge

               # Connect to other connex components.
               for idx3,c3 in qconnect[1:]:
                    graph.addEdge(idx,idx3)     # Create new edge.
                    graph.renameConnex(c3,c2)   # Merge two components.

          # Test if we manage to connect goal to start
          if qgoal is not None and graph.connex[idxstart] == graph.connex[idxgoal]:
               success = True
               break

     return success,graph
          
def astar(graph, start, goal, gdistance = None, hdistance = None):
     '''
     Compute A* path for the input graph connecting start to goal.
     Edges might be oriented.
     gdistance is the distance function between two nodes of the graph.
     hdistance is the heuristic distance function typically called between 
     any arbitrary nodes of the graph and the goal.
     '''
     frontier         = [ (start, 0) ]    # frontier should be a sorted heap 
                                          # (use push and pop methods.
     cameFrom         = { start: None }   # represent a tree.
     costToHere       = { start: 0    }   # cost from start to current node.

     # Graph distance
     if gdistance is None:
          gdistance = lambda i1,i2: norm(graph.q[i1]-graph.q[i2])
     # Heuristic distance
     if hdistance is None:
          hdistance = lambda i1,i2: norm(graph.q[i1]-graph.q[i2])

     # Compute the path from leave to root in a tree
     pathFromTree = lambda tree,path,start,goal: \
         [start,]+path if goal==start \
         else pathFromTree(tree,[goal,]+path,start,tree[goal])

     # Push and pop in a sorted heap
     pop  = lambda heap: heapq.heappop(heap)[1]
     push = lambda heap,item,cost: heapq.heappush(heap,(cost,item))

     # A* make groth a set initially containing only the start while
     # maintaining a list of the nodes a the frontier of this set.
     # The set is iterativelly extended by heuristcally choosing in 
     # the frontier list, until the frontier reaches the goal.
     while len(frontier)>0:
          cur = pop(frontier)                   # Pick the (heuristic) max of the frontier
          if cur == goal:                       # If it is the goal: stop
               return pathFromTree(cameFrom,[], # Return the trajectory from tree <camFrom>
                                   start,goal)  # root to goal.

          for nex in graph.children[cur]:       # Expand the frontier to cur node childrens
               curCost = costToHere[cur] + gdistance(cur, nex)          # Exact cost to nex
               if nex not in costToHere or curCost < costToHere[nex]:
                    # If nex is not yet explored or if a shortest path to nex has been found
                    costToHere[nex] = curCost                           # Set cost-to-here.
                    push(frontier,nex, curCost + hdistance(goal, nex))  # Add nex to the sorted frontier
                    cameFrom[nex] = cur                                 # Add nex to tree.
                    
     # If arriving here: start and goal are not in the same connex component
     # of the graph. Return empty path.
     return []
     
def shortcut(path, stepLength=1e-1, subsampling = 1, niter = 10):
     '''
     Shortcut in a sequence of configurations (not indexes).
     <path> is the original sequence of configurations. Each element of the sequence
     can be connected (obstacle free) with the next one.
     <stepLength> is the length of the step when sampling a new connection for checking
     connectiviy.
     <subsampling> is a multiple of steplength defining how many new elements must
     be added to the sequence when connecting new nodes.
     <niter> defines how many random shortcuts must be tried when calling the function.
     '''
     for _ in range(niter):
          if len(path)<=2: break
          # Randomly select a nontrivial subsequence of <path>.
          first  = random.randint(0,len(path)-3)        # not last or forelast.
          second = random.randint(first+2,len(path)-1)  # subsequence length > 1
          q1,q2  = path[first],path[second]             # initial and final configurations 
          success,qs = connect(q1,q2,store=True)        # try to connect q1 to q2
          if success:                                   # If shortcut is obstacle free ...
               path = (path[:first+1]+                  # Replace previous subsquence
                       qs[::subsampling]+path[second:]) # ... with a subsampling of new connection.
     return path

def samplePath(graph,traj,stepLength=1e-1):
     ''' 
     Given a trajectory defined by key nodes of the graph, regularly sample a sequence of
     configurations along the path.
     <steplength> defines the sampling accuracy. If null, only key configurations are 
     added to the sequence.
     '''
     path = []
     cur = traj[0]
     for nex in traj[1:]:
          q0,q1 = graph.q[cur],graph.q[nex]
          path += [q0,] 
          if stepLength>0: path += connect(q0,q1,stepLength,store=True)[1] 
          path += [q1,] 
          cur = nex
     return path


### Display methods

def displayPrm(graph):
     '''Take a graph object containing a list of configurations q and 
     a dictionnary of graph relations edge. Display the configurations by the correspond 
     placement of the robot end effector. Display the graph relation by vertices connecting 
     the robot end effector positions.
     '''
     
     gui=robot.viewer.gui

     gui.deleteNode('world/prm',True)
     gui.createRoadmap('world/prm',[1.,.2,.2,.8],1e-2,1e-2,[1.,.2,.2,.8])

     for q in graph.q:
          gui.addNodeToRoadmap('world/prm',M2gv(robot.position(q,6)) )
          
     for parent,children in graph.children.items():
          for child in children:
               if child>parent:
                    q1,q2 = graph.q[parent],graph.q[child]
                    p1 = robot.position(q1,6).translation.ravel().tolist()[0]
                    p2 = robot.position(q2,6).translation.ravel().tolist()[0]
                    gui.addEdgeToRoadmap('world/prm',p1,p2)

     gui.refresh()

def displayPath(path,sleeptime=1e-2):
     ''' 
     Display a path, i.e. a sequence of robot configuration, by moving the robot
     to each list element.
     '''
     for q in path: 
          robot.display(q)
          time.sleep(sleeptime)

def displayTraj(graph,traj):  displayPath(samplePath(graph,traj))
     
### Calling the algorithms
print "Compute PRM"

q0 = np.matrix([ .1,2.,.3,.4,.5, .6 ]).T
q1 = np.matrix([ 3.1,-3.,1.3,-1.4,1.5, 2.6 ]).T
success,graph = visibityPrm(qstart = q0, qgoal = q1, nsamples = 2000)

#for n in robot.visual_model.geometryObjects: gui.setVisibility('world/pinocchio/'+n.name,'OFF')
print "Display PRM"
displayPrm(graph)
if success:
     traj = astar(graph,0,1)
     path = samplePath(graph,traj)
     optpath = shortcut(path,100)

     displayPath(optpath)

