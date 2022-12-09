clc;
clear all;
close all;

EXPORT = 1;

DifferentialState x y phi v;
Control theta a;
% AlgebraicState beta;
OnlineData ox oy d_safe track_x track_y trackwidth;

%% Differential Equation
L = 2.75; % Wheelbase
lr = 1.75;
lf = 1;

% states: [x; y; yaw_angle; velocity]
% controls: [front steer angle(theta) , acc]
% beta is slip angle

% bicycle model:
beta = atan( lr/(lf+lr)*tan(theta) );
f = acado.DifferentialEquation();
f.ODE(dot(x) == v * cos(phi+beta));
f.ODE(dot(y) == v * sin(phi+beta));
f.ODE(dot(phi) == v * tan(beta)/lr);
f.ODE(dot(v) == a);
% f.add(0 == -beta + atan(lr*tan(theta)/(lr+lf)));

h = [x, y, phi, v, theta, a];
hN = [x, y, v, phi];
    
n_XD = length(hN);
n_U = length(controls);

% obstacle distances
d_ego_obstacle = (x-ox).^2 + (y-oy).^2;
d_safe = d_safe.^2;

% track distances
dist_sq = (x-track_x).^2 + (y-track_y).^2;
max_deviation_sq  = trackwidth.^2;

%% MPCexport
acadoSet('problemname', 'mpc_tc');

Ts = 0.1;
N = 5;
ocp = acado.OCP( 0.0, N*Ts, N );

W_mat = eye(n_XD+n_U,n_XD+n_U);
WN_mat = eye(n_XD,n_XD);
W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);

ocp.minimizeLSQ( W, h );
ocp.minimizeLSQEndTerm( WN, hN );
%%

% Slx = [0 0 0];
% Slu = [0];
% ocp.minimizeLSQLinearTerms( Slx, Slu );

% Mazda CX-7 steering ratio 15.8
% Steering wheel lock to lock 2.9 turns
% Max steering angle 33 degrees (0.575 rad)

% Constraints
% ocp.subjectTo(-6 <= a <= 6);
% ocp.subjectTo( -0.575 <= theta <= 0.575 );
% ocp.subjectTo(-2.5 <= phi <= 2.5 );

% order of constraints matters!
% Constraint - Collision avoidance
ocp.subjectTo( d_ego_obstacle - d_safe >= 0 );
% Contraint - track boundary
ocp.subjectTo( dist_sq - max_deviation_sq <=0 );
ocp.subjectTo( -10 <=    a    <= 10 );
ocp.subjectTo( 0 <=   v   <= 100 );

ocp.subjectTo( f );

%%

mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',        2*N                 );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
mpc.set( 'HOTSTART_QP',                 'NO'             	);
mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-10				);
mpc.set( 'GENERATE_SIMULINK_INTERFACE', 'YES'               );

if EXPORT
    mpc.exportCode( 'tracking_MPC_allConstr' );
    copyfile('/home/cralab/ACADOtoolkit/external_packages/qpoases', 'tracking_MPC_allConstr/qpoases', 'f')
    
    cd tracking_MPC_allConstr
    make_acado_solver('../tracking_MPC_allConstr')
    cd ..
end
