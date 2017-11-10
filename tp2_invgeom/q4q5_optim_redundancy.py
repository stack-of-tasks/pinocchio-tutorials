'''
This script is the continuation of previous optim_translation.py.
It first samples N (=20) solutions to the position problem. THey are all very different
since the problem is under-constrained (any terminal rotation is equivalent).
It then defines another optimization problem whose cost also contains a reference configuration,
althought with a smaller weight. It then again samples N (=20) solutions. Since the problem
is now fully constraints, the solutions are all quite similar.
'''

from q1q2q3_optim_translation import *

## Question 4
solutions = []
for i in range(20):
    # Compute several configurations matching the reference position
    solutions.append( fmin_bfgs(Cost(target), 
                                x0 = rand(6)*2*np.pi-np.pi,
                                epsilon = 1e-3,
                                disp = False) )

# Display the solutions
print 'Display 20 very different solutions to the under-constraint problem.'''
for q in solutions:
    robot.display(q)
    time.sleep(.2)

print 'First 20 solution ploted ... sleep'
time.sleep(5)


### Question 5
class CostRedundancy:
     '''Functor class computing a cost function summing the distance of the effector to the target
     and the distance to a reference configuration.'''
     def __init__(self,pdes,qdes=zero(6),weight=1e-2):
          self.pdes = pdes
          self.qdes = qdes
          self.w    = weight
     def __call__(self,q):
          '''Compute score from a configuration'''
          p = robot.position(q, 6).translation
          return np.linalg.norm(p-self.pdes) + self.w*np.linalg.norm(q-self.qdes)

qopt = fmin_bfgs(CostRedundancy(target), robot.q0, callback=CallbackLogger())
robot.display(qopt)

solutions = []
for i in range(20):
    # Compute several configurations matching the reference position
    solutions.append( fmin_bfgs(CostRedundancy(target), 
                                x0 = rand(6)*2*np.pi-np.pi,
                                epsilon = 1e-6,
                                disp = False) )

# Display the solutions
print 'Display 20 solutions that are all variations over similar local minima.'''
for q in solutions:
    robot.display(q)
    time.sleep(.2)

