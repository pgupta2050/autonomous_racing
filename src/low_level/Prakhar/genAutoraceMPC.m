function [ sysStates, sysInputs, sysOde, sysParams ] = genAutoraceMPC( N, Ts, EXPORT )

% [ sysStates, sysInputs, sysOde, sysParams ] = genAutoraceMPC( N, Ts, EXPORT )
% 
% OUTPUT:
%       sysStates   := 
%       sysInputs   := 
%       sysOde*     := 
%       sysParams*  := 
% 
% *: Optional arguments
% 
% INPUT:
%       N       := 
%       Ts      := 
%       EXPORT  := 
% 
% 
% 
% Written by
% Jacky Tang
% 14-Nov-2022
% Department of Mechanical Engineering
% Clemson University
% 
% 
% 

    %% ACADO directory
    % Example:
    % { Source location/ } acado-stable/external_packages/qpoases
    srcAcado = '/home/ACADOtoolkit/';

    %% System Parameters - Kinematic Bicycle Model
    sysStates  = {'$s_x$', '$s_y$', '$\phi$', '$v$'}; % x-COM, y-COM, inertial heading, vehicle speed
    sysInputs  = {'$\delta_f$', '$a$'};               % front steering angle, acceleration
    sysOutputs = {'$s_x$', '$s_y$', '$\phi$', '$v$'}; % x-COM, y-COM, inertial heading, vehicle speed
    
    % From paper 'Kinematic and Dynamic Vehicle Models for Autonomous Driving
    % Control Design' by J.Kong, M.Pfeiffer, G.Schildbach, F.Borrelli from UC
    % Berkeley.
    % It is based on Hyundai Azera (total wheelbase of 2.843m)
    l_f = 1.05;    % [m] Distance from vehicle's CenterOfMass to front axle
    l_r = 1.75;    % [m] Distance from vehicle's CenterOfMass to rear axle

    %% Differential Equation
    DifferentialState sx sy phi v;
    % AlgebraicState beta
    Control delta_f a;
    OnlineData ox oy d_safe;    % ox     := x-pos of competitor;
                                % oy     := y-pos of competitor;
                                % d_safe := safety distance
    
    beta = atan( l_r/(l_f+l_r)*tan(delta_f) );
    
    f = acado.DifferentialEquation();
    f.add( dot(sx ) == v*cos(phi+beta) );
    f.add( dot(sy ) == v*sin(phi+beta) );
    f.add( dot(phi) == v*sin(    beta)/l_r );
    f.add( dot( v ) == a );
    % f.add(    0     == -beta + atan( l_r/(l_f+l_r)*tan(delta_f) ) );
    
    h = [diffStates; controls];
    hN = [diffStates];
    
    n_XD = length(hN);
    n_U = length(controls);

    d_ego_obstacle = (sx-ox).^2 + (sy-oy).^2;
    d_safe = d_safe.^2;
    
    %% MPC Export - Controller
    acadoSet('problemname', 'autorace');
    
    ocp = acado.OCP( 0.0, N*Ts, N );
    
    W_mat = eye(n_XD+n_U,n_XD+n_U);
    WN_mat = eye(n_XD,n_XD);
    W = acado.BMatrix(W_mat);
    WN = acado.BMatrix(WN_mat);
    
    ocp.minimizeLSQ( W, h );
    ocp.minimizeLSQEndTerm( WN, hN );
    
    % Slx = [0 0 0];
    % Slu = [0];
    % ocp.minimizeLSQLinearTerms( Slx, Slu );
    
    % Constraint - Collision avoidance
    ocp.subjectTo( d_ego_obstacle - d_safe >= 0 );   % where d_safe>=0
    
    % Constraint - Track
    % (s_x,s_y) < trackR
    % (s_x,s_y) > trackL
    
    % Constraint - Input
    % ocp.subjectTo( -10 <= delta_f <= 10 );
    ocp.subjectTo( -10 <=    a    <= 10 );
    
    % Constraint - State
    ocp.subjectTo( 0 <=   v   <= 100 );
    ocp.subjectTo( f );
    
    mpc = acado.OCPexport( ocp );
    mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
    mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
    mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
    mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       );
    mpc.set( 'NUM_INTEGRATOR_STEPS',         2*N                );
    mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
    mpc.set( 'HOTSTART_QP',                 'NO'             	);
    mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-10				);
    mpc.set( 'GENERATE_SIMULINK_INTERFACE', 'YES'               );
    
    if EXPORT
        mpc.exportCode( 'export_MPC' );
        copyfile('/home/cralab/ACADOtoolkit/external_packages/qpoases', 'export_MPC/qpoases', 'f');
        
        % Generating, compiling and exporting MPC controller as '.mexw64'
        cd export_MPC
        make_acado_solver('../autoraceMPCstep');
        cd ..
    end
    
    %% SIM Export - System ODE
    acadoSet('problemname', 'autorace_sim');
    
    numSteps = 5;
    sim = acado.SIMexport( Ts );
    % sim.setLinearInput(M1,A1,B1);
    sim.setModel(f);
    sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_RIIA5' );
    sim.set( 'NUM_INTEGRATOR_STEPS',         numSteps       );
    
    if EXPORT
        sim.exportCode( 'export_SIM' );
        
        cd export_SIM
        make_acado_integrator('../integrate_autorace');
        cd ..
    end

    %% Reset
    sysStates = diffStates;
    sysInputs = controls;
    if nargout > 2
        sysOde = f;

        if nargout > 3 
            sysParams.OCP = ocp;
            sysParams.MPC = mpc;
            sysParams.Integrator = sim;
        end
    end

    clear global
    clc

end