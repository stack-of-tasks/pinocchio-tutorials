import os
import gepetto.corbaserver
from pinocchio.utils import *
import time

os.system('gepetto-gui &')

time.sleep(1)
viewer=gepetto.corbaserver.Client()
windowID = viewer.gui.createWindow ("viewtest")
viewer.gui.createSceneWithFloor("world")
viewer.gui.addSceneToWindow("world",windowID)

viewer.gui.applyConfiguration("world/floor",[0,0,0,  .7071,.7071,0,0])
viewer.gui.refresh()

N = 100
for i in range(10*N):
    viewer.gui.applyConfiguration("world/floor",[0,0,0,  .7071,.7071,i/(N+.0),0])
    viewer.gui.refresh()
    time.sleep(1e-2)
