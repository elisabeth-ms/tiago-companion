from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import warnings

#suppress warnings
warnings.filterwarnings('ignore')
eps = 0.5

def superquadricPoints(lambda1, lambda2, lambda3, lambda4, lambda5):
    X = []
    Y = []
    Z = []
    x = np.arange(-0.0125, 1.0125, 0.0025, dtype=np.float64)
    y = np.arange(-0.0125, 1.0125, 0.0025, dtype=np.float64)
    z = np.arange(-0.01, 1.015, 0.0025, dtype=np.float64)


    # print(x,y,z)
    xv, yv, zv = np.meshgrid(x, y, z)
    print("lambda5/lambda4", lambda5/lambda4)
    f= ((xv / lambda1) ** (2.0/lambda5) + (yv/ lambda2) ** (2.0/lambda5)) ** (lambda5 / lambda4) + (zv / lambda3) ** (2.0/lambda4)
    f = np.array(f, dtype=np.float64)
    # print(f)
    correct = (f > (1-eps)) & (f < (1+eps))
    # print(correct)
    # print(xv.shape)
    for i in range(0,xv.shape[0]):
        for j in range(0,xv.shape[1]):
            for k in range(0,xv.shape[2]):
                if correct[i,j,k]:
                    X.append(xv[i,j,k])
                    Y.append(yv[i,j,k])
                    Z.append(zv[i,j,k])
                    
                    X.append(-xv[i,j,k])
                    Y.append(yv[i,j,k])
                    Z.append(zv[i,j,k])

                    X.append(xv[i,j,k])
                    Y.append(-yv[i,j,k])
                    Z.append(zv[i,j,k])

                    X.append(-xv[i,j,k])
                    Y.append(-yv[i,j,k])
                    Z.append(zv[i,j,k])

                    X.append(xv[i,j,k])
                    Y.append(yv[i,j,k])
                    Z.append(-zv[i,j,k])
                    
                    X.append(-xv[i,j,k])
                    Y.append(yv[i,j,k])
                    Z.append(-zv[i,j,k])

                    X.append(xv[i,j,k])
                    Y.append(-yv[i,j,k])
                    Z.append(-zv[i,j,k])

                    X.append(-xv[i,j,k])
                    Y.append(-yv[i,j,k])
                    Z.append(-zv[i,j,k])


    # i = 0
    # for xi in np.arange(-1, 1, 0.001):    
    #     for yi in np.arange(-1,1,0.001):
    #         for zi in np.arange(-1,1,0.001):
    #             if i<len(f):
    #                 if ( f[i] > (1-eps) and f[i] < (1+eps)):
    #                     X.append(xi)
    #                     Y.append(yi)
    #                     Z.append(zi)
    #             i = i + 1
    #             print(xi, yi, zi,i)
    return np.array(X), np.array(Y), np.array(Z)

X, Y, Z = superquadricPoints(1.0,1.0,1.0,0.001,2.0)


fig = plt.figure()

ax = plt.axes(projection='3d')
# surf = ax.plot_trisurf(X, Y, Z, edgecolor='none')

ax.scatter(X, Y, Z, c=Z, cmap='viridis', linewidth=0.5);
plt.show()


