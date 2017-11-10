from pinocchio.utils import *
from factor import FactorGraph,Factor

f = FactorGraph(1,5)   # Define a factor of 5 variables of dimension 1

M = eye(1)             # M is simply 1 written as a 1x1 matrix.

# Add the constraints that all the variables should be equal (ie x_i = x_i+1)
for i in range(4):
    f.addFactorConstraint( [ Factor( i,M ), Factor( i+1,-M ) ], zero(1) )

# The cost is so that x_1 is as close as possible to 10, and x_N as close as possible to 20
f.addFactor( [ Factor(0,M) ], M*10 )
f.addFactor( [ Factor(4,M) ], M*20 )

# Guess what is the solution?
x = f.solve()

