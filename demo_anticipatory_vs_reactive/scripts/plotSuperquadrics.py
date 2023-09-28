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

    mu = np.arange(-np.pi/2.0-0.05, np.pi/2.0+0.05, 0.01, dtype=np.float64)
    omega = np.arange(-np.pi-0.05, np.pi+0.05, 0.01, dtype=np.float64)
    
    muv, omegav = np.meshgrid(mu, omega)
    X.append(lambda1*np.cos(muv)**lambda4*np.cos(omegav)**lambda5)
    Y.append(lambda2*np.cos(muv)**lambda4*np.sin(omegav)**lambda5)
    Z.append(lambda3*np.sin(muv)**lambda4)

    X.append(-lambda1*np.cos(muv)**lambda4*np.cos(omegav)**lambda5)
    Y.append(lambda2*np.cos(muv)**lambda4*np.sin(omegav)**lambda5)
    Z.append(lambda3*np.sin(muv)**lambda4)

    X.append(lambda1*np.cos(muv)**lambda4*np.cos(omegav)**lambda5)
    Y.append(-lambda2*np.cos(muv)**lambda4*np.sin(omegav)**lambda5)
    Z.append(lambda3*np.sin(muv)**lambda4)

    X.append(-lambda1*np.cos(muv)**lambda4*np.cos(omegav)**lambda5)
    Y.append(-lambda2*np.cos(muv)**lambda4*np.sin(omegav)**lambda5)
    Z.append(lambda3*np.sin(muv)**lambda4)

    X.append(lambda1*np.cos(muv)**lambda4*np.cos(omegav)**lambda5)
    Y.append(lambda2*np.cos(muv)**lambda4*np.sin(omegav)**lambda5)
    Z.append(-lambda3*np.sin(muv)**lambda4)

    X.append(-lambda1*np.cos(muv)**lambda4*np.cos(omegav)**lambda5)
    Y.append(lambda2*np.cos(muv)**lambda4*np.sin(omegav)**lambda5)
    Z.append(-lambda3*np.sin(muv)**lambda4)

    X.append(lambda1*np.cos(muv)**lambda4*np.cos(omegav)**lambda5)
    Y.append(-lambda2*np.cos(muv)**lambda4*np.sin(omegav)**lambda5)
    Z.append(-lambda3*np.sin(muv)**lambda4)

    X.append(-lambda1*np.cos(muv)**lambda4*np.cos(omegav)**lambda5)
    Y.append(-lambda2*np.cos(muv)**lambda4*np.sin(omegav)**lambda5)
    Z.append(-lambda3*np.sin(muv)**lambda4)

    X = np.array(X)
    Y = np.array(Y)
    Z = np.array(Z)

    X =X.flatten('F')
    Y =Y.flatten('F')
    Z =Z.flatten('F')

    mu_, omega_ = np.meshgrid(0, omega)

    X = np.append(X,lambda1*np.cos(mu_)**lambda4*np.cos(omega_)**lambda5)
    Y= np.append(Y,lambda2*np.cos(mu_)**lambda4*np.sin(omega_)**lambda5)
    Z = np.append(Z,lambda3*np.sin(mu_)**lambda4)

    X = np.append(X,-lambda1*np.cos(mu_)**lambda4*np.cos(omega_)**lambda5)
    Y= np.append(Y,lambda2*np.cos(mu_)**lambda4*np.sin(omega_)**lambda5)
    Z = np.append(Z,lambda3*np.sin(mu_)**lambda4)

    X = np.append(X,lambda1*np.cos(mu_)**lambda4*np.cos(omega_)**lambda5)
    Y= np.append(Y,-lambda2*np.cos(mu_)**lambda4*np.sin(omega_)**lambda5)
    Z = np.append(Z,lambda3*np.sin(mu_)**lambda4)


    X = np.append(X,-lambda1*np.cos(mu_)**lambda4*np.cos(omega_)**lambda5)
    Y= np.append(Y,-lambda2*np.cos(mu_)**lambda4*np.sin(omega_)**lambda5)
    Z = np.append(Z,lambda3*np.sin(mu_)**lambda4)
    return np.array(X), np.array(Y), np.array(Z)

X, Y, Z = superquadricPoints(1.0,1.0,1.5,0.1,1.9)

X =X.flatten('F')
Y =Y.flatten('F')
Z =Z.flatten('F')
print(X.shape)
print(Y.shape)
print(Z.shape)
fig = plt.figure()

ax = plt.axes(projection='3d')
# surf = ax.plot_trisurf(X, Y, Z, edgecolor='none')

ax.scatter3D(X, Y, Z, c=Z, cmap='viridis', linewidth=0.5);
plt.show()


