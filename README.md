# AutonomousRacing

## High leveL:

Description:
- `pdgplanner.py` is the high-level planner of the Ego vehicle
- `aiagent,py` contains competing AI agents

Key dependencies:
- `pdgplanner.py` should be in the same directory as `ilqr/` folder
- `aiagent,py` requires Casadi in the appended path

## Low Level:

Description:
- `tracking_mpc.mex` is the the controller compiled from acado
- `sim_lowlevel.m` is the low level control ros node

Key dependencies:
- Acado toolkit for matlab
