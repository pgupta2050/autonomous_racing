from sys import path
path.append(r"/home/optimal-v/Work/casadi-linux-py39-v3.5.5-64bit")

from casadi import *

import numpy as np

class Competitor1():
    """ 
    AI Agent Competitor 1: 
    Goal tracking MPC with collision avoidance constraints
    """
    def __init__(self, X_0, X_ego_obsrvd):
        self.dt = 0.1
        self.N = 10
        self.dsafe = 1
        self.amax = 5
        self.omegamax = 30

        self.X_0 = X_0

        self.X_ego_obsrvd = X_ego_obsrvd

    def dynamics(self, X_now, U_now):
        """ Unicycle model:
            X = [x y v theta]'
            U = [a omega]'
        """        
        X_next = np.zeros((4,1))
        X_next[0] = X_now[0] + X_now[2] * np.cos(X_now[3]) * self.dt
        X_next[1] = X_now[1] + X_now[2] * np.sin(X_now[3]) * self.dt
        X_next[2] = X_now[2] + U_now[0] * self.dt
        X_next[3] = X_now[3] + U_now[1] * self.dt

        return X_next

    def const_vel_model(self):
        """ 
        Method used by competitor 1 to obtain over the horizon trajectory of ego for collision avoidance:        
        Constant velocity model - assume velocity of ego remains at the current observed velocity over the
        horizon.
        """
        x_ego = np.zeros((1, self.N+1))
        x_ego[0] = self.X_ego_obsrvd[0]
        y_ego = np.zeros((1, self.N+1))
        y_ego[0] = self.X_ego_obsrvd[1]

        for k in range(self.N):
            x_ego[k+1] = x_ego[k] + self.X_ego_obsrvd[2] * np.cos(self.X_ego_obsrvd[3]) * self.dt
            y_ego[k+1] = y_ego[k] + self.X_ego_obsrvd[2] * np.sin(self.X_ego_obsrvd[3]) * self.dt

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

        opti.minimize((X[0,-1] - self.xgoal)^2 + (X[1,-1] - self.ygoal)^2)
        
        opti.subject_to(X[:, 0] == self.X_0) # initial condition

        for k in range(self.N):
            opti.subject_to(X[:, k+1] == self.dynamics(X[:, k], U[:, k]))
            opti.subject_to(np.array([-self.amax, -self.omegamax]).T <= U[:, k] <= np.array([self.amax, self.omegamax]).T)

        # collision avoidance constraint
        for k in range(self.N):
            opti.subject_to((X[0, k] - x_ego[k])^2 + (X[1, k] - y_ego[k])^2 >= self.dsafe^2)

        opti.solver('ipopt')

        sol = opti.solve()

        # print(sol.value(X))
        # print(sol.value(U))
        return sol.value(X), sol.value(U)

                 


