from sys import path
path.append(r"/home/optimal-v/Work/casadi-linux-py39-v3.5.5-64bit")

from casadi import *

import numpy as np
import matplotlib.pyplot as plt

class Competitor1():
    """ 
    AI Agent Competitor 1: 
    Goal tracking MPC with collision avoidance constraints
    """
    def __init__(self, X_0, X_ego_obsrvd, xgoal, ygoal):
        self.dt = 0.1
        self.N = 10
        self.dsafe = 0.5
        self.alim = 5
        self.omegalim = np.pi/4

        self.X_0 = X_0
        self.xgoal = xgoal
        self.ygoal = ygoal

        self.X_ego_obsrvd = X_ego_obsrvd

    def const_vel_model(self):
        """ 
        Method used by competitor 1 to predict over the horizon trajectory of ego for collision avoidance:        
        Constant velocity model - assume velocity of ego remains at the current observed velocity over the
        horizon.
        """
        x_ego = np.zeros((1, self.N+1))
        x_ego[0] = self.X_ego_obsrvd[0]
        y_ego = np.zeros((1, self.N+1))
        y_ego[0] = self.X_ego_obsrvd[1]

        for k in range(self.N):
            x_ego[0, k+1] = x_ego[0, k] + self.X_ego_obsrvd[2] * np.cos(self.X_ego_obsrvd[3]) * self.dt
            y_ego[0, k+1] = y_ego[0, k] + self.X_ego_obsrvd[2] * np.sin(self.X_ego_obsrvd[3]) * self.dt

        # print(x_ego, y_ego)
        return x_ego, y_ego

    def MPCOpt(self):
        """
        J = (xN - xgoal)^2 + (yN - ygoal)^2
        s.t. dynamics
            const_vel_model for (xk - x_egok)^2 + (yk - y_egok)^2 >= dsafe^2
            u \in [umin, umax]
            X_0
        """
        # predict over the horizon trajectory of ego
        x_ego, y_ego = self.const_vel_model()

        # begin MPC optimization over the horizon
        opti = casadi.Opti()

        X = opti.variable(4, self.N+1)
        U = opti.variable(2, self.N)

        opti.minimize((X[0,-1] - self.xgoal) ** 2 + (X[1,-1] - self.ygoal) ** 2) # cost
        # opti.minimize((X[0,:] - self.xgoal) @ (X[0,:] - self.xgoal).T + (X[1,:] - self.ygoal) @ (X[1,:] - self.ygoal).T) # cost
        
        opti.subject_to(X[:, 0] == self.X_0) # initial condition

        for k in range(self.N):
            # dynamics
            """ 
            Unicycle model:
            X = [x y v theta]'
            U = [a omega]'
            """  
            opti.subject_to(X[0, k+1] == X[0, k] + X[2, k] * np.cos(X[3, k]) * self.dt)
            opti.subject_to(X[1, k+1] == X[1, k] + X[2, k] * np.sin(X[3, k]) * self.dt)
            opti.subject_to(X[2, k+1] == X[2, k] + U[0, k] * self.dt)
            opti.subject_to(X[3, k+1] == X[3, k] + U[1, k] * self.dt)
            # control admissibility
            opti.subject_to(opti.bounded(-self.alim, U[0, k], self.alim))
            opti.subject_to(opti.bounded(-self.omegalim, U[1, k], self.omegalim))

        # collision avoidance constraint
        for k in range(self.N):
            opti.subject_to((X[0, k] - x_ego[0, k]) ** 2 + (X[1, k] - y_ego[0, k]) ** 2 >= self.dsafe ** 2)

        opti.solver('ipopt')        
        
        try:
            sol = opti.solve()
            return sol.value(X), sol.value(U)
        except:
            return self.if_no_mpc_sol()

    def if_no_mpc_sol(self):
        """
        To handle cases where nmpc is not solvable 
        """
        X = np.zeros((4, self.N+1))
        X[:,0] = self.X_0
        U = np.zeros((2, self.N))
        
        for k in range(self.N):
            X[0, k+1] = X[0, k] + X[2, k] * np.cos(X[3, k]) * self.dt
            X[1, k+1] = X[1, k] + X[2, k] * np.sin(X[3, k]) * self.dt
            X[2, k+1] = X[2, k] + U[0, k] * self.dt
            X[3, k+1] = X[3, k] + U[1, k] * self.dt

        return X, U


# Open Loop Test
if __name__ == "__main__":
    X_0 = np.array([0, 2, 10, np.pi/4]).T
    X_ego_obsrvd = np.array([0, 0, 4, np.pi/6]).T

    AIAgent = Competitor1(X_0, X_ego_obsrvd, 5, 5)
    X, U = AIAgent.MPCOpt()    
    # print(X)
    # print(U)
    xego, yego = AIAgent.const_vel_model()
    
    plt.figure(1)
    plt.plot(X[0,:], X[1, :], "b")
    plt.plot(xego[0,:], yego[0,:], "r")

    plt.figure(2)
    plt.plot(X[2,:])
    plt.ylabel("Velocity [m/s]")

    plt.figure(3)
    plt.subplot(211)
    plt.plot(U[0,:])
    plt.ylabel("Accl [m/s^2]")
    plt.subplot(212)
    plt.plot(U[1,:])
    plt.ylabel("Omega [rad/s]")

    plt.show()