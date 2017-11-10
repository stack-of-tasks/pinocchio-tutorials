'''
Solve a problem QP:
min_x  .5 x.T H x - g.T x  
       s.t. C x <= d

with x of size N, and C having M rows.

Full documentation of the QP solver is available using help(solve_qp)
'''

from pinocchio.utils import *
from quadprog import solve_qp

N = 6
M = 3

H = rand([N,N]); H = H*H.T
g = rand(N)

C = rand([M,N])
d = rand(M)

asarray = np.asarray
x,_,_,_,_,_ = solve_qp( asarray(H),asarray(g).T[0],
                        asarray(C).T,asarray(d).T[0] )
