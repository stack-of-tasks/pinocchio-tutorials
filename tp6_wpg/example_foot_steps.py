from foot_steps import FootSteps
from numpy import matrix

footsteps = FootSteps( matrix([0.0,-0.1]).T , matrix([0.0,0.1]).T )
footsteps.addPhase( .3, 'none' )
footsteps.addPhase( .7, 'left' , matrix([0.1,+0.1]).T )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'right', matrix([0.2,-0.1]).T )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'left' , matrix([0.3,+0.1]).T )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'right', matrix([0.4,-0.1]).T )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'left' , matrix([0.5,+0.1]).T )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'right', matrix([0.5,-0.1]).T )
footsteps.addPhase( .5, 'none' )

refTime = 2.0

print "*** Footstep status at time ",refTime
print "Current phase: <",footsteps.getPhaseType(refTime), "> foot is flying."
print "Foot positions: left = ",footsteps.getLeftPosition(refTime).T, \
      "  -- right = ", footsteps.getRightPosition(refTime).T
print "Next position of left foot = ", footsteps.getLeftNextPosition(refTime).T

print "Phase timings: ", [ footsteps.getPhaseStart(refTime), \
                               footsteps.getPhaseStart(refTime)+footsteps.getPhaseDuration(refTime) ] 
print "Time to impact: ", footsteps.getPhaseRemaining(refTime)

