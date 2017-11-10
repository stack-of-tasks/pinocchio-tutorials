import numpy as np
import matplotlib.pyplot as plt
# In plt, the following functions are the most useful:
#    ion,plot,draw,show,subplot,figure,title,savefig

# For use in interactive python mode (ipthyon -i)
interactivePlot = False

if interactivePlot:
    plt.ion() # Plot functions now instantaneously display, shell is not blocked


# Build numpy array for x axis
x = 1e-3 * np.array (range (100))
# Build numpy array for y axis
y = x**2

fig = plt.figure ()
ax = fig.add_subplot ('111')
ax.plot (x, y)
ax.legend (("x^2",))

if not interactivePlot:
    # Display all the plots and block the shell.
    # The script will only ends when all windows are closed.
    plt.show ()

