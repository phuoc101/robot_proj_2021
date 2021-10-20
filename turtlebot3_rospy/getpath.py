import numpy as np

def getCircle(x0, y0, r, step):
    x1 = np.arange(x0-r, x0+r, step)
    x2 = np.flip(x1)
    y1 = (2*y0 + np.sqrt(4*y0**2 - 4*(y0**2 + (x1 - x0)**2 - r**2)))/2
    y2 = (2*y0 - np.sqrt(4*y0**2 - 4*(y0**2 + (x1 - x0)**2 - r**2)))/2
    x = np.concatenate((x1, x2))
    x = np.concatenate((x, x[np.newaxis, 0]))
    y = np.concatenate((y1, y2))
    y = np.concatenate((y, y[np.newaxis, 0]))
    x = np.flip(x)
    y = np.flip(y)
    theta = -4*np.ones(x.size)
    goal = list(zip(x, y, theta))
    return goal

def getSquare(x0, y0, l, step):
    x1 = np.arange(x0, x0+l+step, step)
    x2 = np.ones(x1.shape[0])*(x0+l)
    x3 = np.arange(x0+l, x0-step, -step)
    x4 = np.ones(x1.shape[0])*x0
    y1 = np.ones(x1.shape[0])*y0
    y2 = np.arange(y0, y0+l+step, step)
    y3 = np.ones(x1.shape[0])*(y0+l)
    y4 = np.arange(y0+l, y0-step, -step)
    x = np.concatenate((x1, x2, x3, x4))
    y = np.concatenate((y1, y2, y3, y4))
    theta = -4*np.ones(x.size)
    goal = list(zip(x, y, theta))
    return goal 
