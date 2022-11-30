import aiagent as c1
# import pdgplanner as ego

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

dt = 0.1
Horizon = 20
# Initial states
x1_0 = 0
y1_0 = 0
v1_0 = 0
theta1_0 = 0
x2_0 = 0
y2_0 = 3
v2_0 = 0
theta2_0 = 0
# Initial augemnted control-states
a1_0 = 0
omega1_0 = 0
a2_0 = 0
omega2_0 = 0

# Goal
xgoal1 = 10
ygoal1 = 5
xgoal2 = 10
ygoal2 = 5
dsafe = 0.5

# Xsol, Usol = ego.PDGILQR(dt, Horizon, x1_0, y1_0, v1_0, theta1_0, x2_0, y2_0, v2_0, theta2_0, \
#                     xgoal1, ygoal1, xgoal2, ygoal2, dsafe, a1_0, omega1_0, a2_0, omega2_0).solve_ilqr()
Ego_data = pd.read_csv("test.txt")
Ego_data = np.array(Ego_data)

X0_ai = np.array([x2_0, y2_0, v2_0, theta2_0]).T
X0_ego = np.array([x1_0, y1_0, v1_0, theta1_0]).T

AIAgent = c1.Competitor1()

ego_traj = np.zeros((4,21))
ego_traj[:,0] = X0_ego

nv_traj = np.zeros((4,21))
nv_traj[:,0] = X0_ai

for k in range(len(Ego_data)):
    
    X, U = AIAgent.MPCOpt(X0_ai, X0_ego, xgoal2, ygoal2)

    X0_ai = X[:,1]
    X0_ego = Ego_data[k,:].T

    ego_traj[:,k+1] = X0_ego
    nv_traj[:,k+1] = X0_ai

plt.figure(1)
plt.plot(ego_traj[0,:], ego_traj[1,:], "b", label='Ego')
plt.plot(nv_traj[0,:], nv_traj[1,:], "r", label='Competitor')
plt.legend()

plt.show()