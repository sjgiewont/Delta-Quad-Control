import numpy as np
from numpy import pi



def piecewiseMotion(step_length, degrees, precision):
    # set the step size of the time parameter
    t = np.linspace(0, 1, precision)

    # convert the degree input to radians
    rad = degrees * (pi/180)

    # determine all the values defined by the piecewise functions
    piecewise_y = np.piecewise(t, [(t >= 0) & (t <= 0.2), t > 0.2], [lambda t: -np.cos(rad)+10*t*np.cos(rad), lambda t: 1.5*np.cos(rad)-2.5*t*np.cos(rad)])
    piecewise_x = np.piecewise(t, [(t >= 0) & (t <= 0.2), t > 0.2], [lambda t: -np.sin(rad)+10*t*np.sin(rad), lambda t: 1.5*np.sin(rad)-2.5*t*np.sin(rad)])
    piecewise_z = np.piecewise(t, [(t >= 0) & (t <= 0.1), (t > 0.1) & (t <= 0.2)], [lambda t: 100*t, lambda t: 20-100*t])

    piecewise = []

    # create matrix of all positions along trajectory
    for i in range(len(t)):
        piecewise.append([step_length*piecewise_x[i], step_length*piecewise_y[i], piecewise_z[i]-220])

    return piecewise
