%% Initialisation
format compact;
close all;
clear all;
clc;

ROS_ENABLE = 1;
if ROS_ENABLE == 1
    rosshutdown
    rosinit         % Initialise rosnode

    % Subscribe/Publish
    sub_traj = rossubscriber('traj','trajectory_msgs/JointTrajectory','DataFormat','struct'); 
    pub = rospublisher('odom','nav_msgs/Odometry');
end

%% Generate MPC using ACADO
EXPORT = 0; % Before running the function 'genAutoraceMPC', change the
            % variable 'srcAcado' to the path of the installed ACADO in the
            % respective computer.

Ts = 0.1;   % Sampling time
N = 5;     % Prediction horizon [steps]

if EXPORT==1, tic; end
[ carStates, carInputs, carOde, carParams ] = genAutoraceMPC( N, Ts, EXPORT );
if EXPORT==1, tGenMPC = toc; disp(['Time(genAutoraceMPC) = ',num2str(tGenMPC,'%.6f'),' seconds.']); return; end

%% Simulation - Initialisation
sysStates  = {'$s_x$', '$s_y$', '$\phi$', '$v$'}; % x-COM, y-COM, inertial heading, vehicle speed
sysInputs  = {'$\delta_f$', '$a$'};               % front steering angle, acceleration
sysOutputs = {'$s_x$', '$s_y$', '$\phi$', '$v$'}; % x-COM, y-COM, inertial heading, vehicle speed

n = length(carStates);
m = length(carInputs);
n_U = m;

% R = 50;                                     % Track radius
% [xTrack,yTrack] = genTrackRef(R,1000);      % Circular track

% xTrack = mapTrack(:,1);                     % Recorded map
% yTrack = mapTrack(:,2)-mapTrack(1,2);

% [xTrack,yTrack] = genSineCurve();           % Sine curve

% xTrack = ones(1000,1);                      % Straight line
% yTrack = 1.*linspace(1,1000,1000)';

% mapHighLevel = readcell('high_level_traj.txt'); % sx, sy, v, phi
% mapHighLevel = cell2mat(mapHighLevel);
% idxMapInit = 1;
% xTrack = mapHighLevel(idxMapInit:end,1);
% yTrack = mapHighLevel(idxMapInit:end,2);
% vTrack = mapHighLevel(idxMapInit:end,3);
% pTrack = mapHighLevel(idxMapInit:end,4);

Tf = 900*.1;
% X0 = [xTrack(1)-.1, -0.1, deg2rad(90), 0];   % sx, sy, phi, v    % Init conditions
X0 = [0, 0, deg2rad(0), 0];   % sx, sy, phi, v    % Init conditions

% xTrack = xTrack(floor(logspace(0.5,2,40)));
% yTrack = yTrack(floor(logspace(0.5,2,40)));
% [xTrack,yTrack,pTrack] = genVehicleRef([X0(1:2);xTrack(1:end-1),yTrack(1:end-1)]',[xTrack,yTrack]');
% xTrack = xTrack';
% yTrack = yTrack';
% pTrack = pTrack';
% vTrack = 1.*ones(size(xTrack));
% vTrack = 1.*logspace(0.1,0.9,length(xTrack))';

d_safe = 0.0;

% xObs = 1.75;    yObs = 1.0;
% xObs = 2.0 ;    yObs = 1.0;
% xObs = 2.5 ;    yObs = 1.0;
% xObs = 3.0 ;    yObs = 1.0;
% xObs = 3.5 ;    yObs = 1.0;
xObs = 4.0 ;    yObs = 1.0;
% xObs = 2.0 ;    yObs = 0.5;

xObs = xObs.*ones(size(xTrack));    xObs = linspace(2,3,length(xTrack))';
yObs = yObs.*ones(size(xTrack));
dSafe = d_safe.*ones(size(xObs));

Uref = zeros(N,n_U);
input.u = Uref;

% input.W = diag([10,10,0.5,0,10,0.5]); % sx, sy, phi, v, delta_f, a
% input.WN = diag([1,1,0.1,0]);

input.W = diag([10,10,0,2,1,0.1]); % sx, sy, phi, v, delta_f, a
input.WN = diag([1,1,0,1]);

%% Simulation - Running
disp('------------------------------------------------------------------');
disp('               Simulation Loop'                                    );
disp('------------------------------------------------------------------');

k = 1;
iter = 0; time = 0;
KKT_MPC = []; INFO_MPC = [];
controls_MPC = [];
state_sim = X0;
input_sim = [];

if ROS_ENABLE == 1
    % Let high level know controller is alove
    get_new_segment = 1;
    msg =  rosmessage(pub);
    msg.Pose.Pose.Position.X = X0(1);
    msg.Pose.Pose.Position.Y = X0(2);
    msg.Twist.Twist.Linear.X = X0(4);
    msg.Pose.Pose.Orientation.Z = X0(3);
    msg.Pose.Pose.Position.Z = get_new_segment;
    send(pub, msg);
    
    % set flag to begin loop
    get_new_segment = 1;
    
    ref_traj_list = [];
end

while time(end) < Tf
%     tic

    if ROS_ENABLE == 1 && get_new_segment == 1 
        % Get the ref for this step and wait for next high level command
        traj1 = receive(sub_traj,70);
        
        % Consume the subscriber data
        x_ilqrRef = GetTraj(traj1);
        xy_ref_traj = x_ilqrRef(1:2,:)';
        phi_ref_traj = x_ilqrRef(3,:)';
        v_ref_traj = x_ilqrRef(4,:)';

        xTrack = xy_ref_traj(:,1);
        yTrack = xy_ref_traj(:,2);
        pTrack = phi_ref_traj;
        vTrack = v_ref_traj;

%         xy_AI_predict_traj = __;      % <<<<< pass data from ROS to here
        xObs = xy_AI_predict_traj(:,1); % Column vector, Obstacle x-position
        yObs = xy_AI_predict_traj(:,2); % Column vector, Obstacle y-position
        dSafe = d_safe.*ones(size(xObs));

        msg.Pose.Pose.Position.Z = get_new_segment;
        % wait to get new traj till segment end is reached
        get_new_segment = 0;
        
        ref_traj_list = [ref_traj_list; xy_ref_traj];
    end
    
    % Time Varying Reference Update
    Xref =    [xTrack(k:k+N-1),yTrack(k:k+N-1),1.*pTrack(k:k+N-1),1.*vTrack(k:k+N-1)]; %repmat(Xref,N,  1);
    input.x = [xTrack(k:k+N),  yTrack(k:k+N),  1.*pTrack(k:k+N),  1.*vTrack(k:k+N)];   %repmat(Xref,N+1,1);
    
    input.y = [Xref(1:N,:), Uref];
    input.yN = Xref(N,:);

    % Obstacle(s)
    obstacle = [xObs(k:k+N),yObs(k:k+N),dSafe(k:k+N)];
    input.od = obstacle;

    % Solve NMPC OCP
    input.x0 = state_sim(end,:);
    output = autoraceMPCstep(input);

    % Save the MPC Step
    INFO_MPC = [INFO_MPC; output.info];
    KKT_MPC = [KKT_MPC; output.info.kktValue];
    controls_MPC = [controls_MPC; output.u(1,:)];
%     input.x = output.x;
    input.u = output.u;
    
    % Simulate System
    sim_input.x = state_sim(end,:).';
    sim_input.u = output.u(1,:).';
    states = integrate_autorace(sim_input);     % Vehicle ODE
    state_sim = [state_sim; states.value'];
    input_sim = [input_sim; output.u(1,:)];
    
    iter = iter+1;
    nextTime = iter*Ts;
    time = [time, nextTime];
    disp(['Time: ',num2str(nextTime,'%06.1f'),char(9), ...
          'RTI step: ',num2str(output.info.cpuTime*1e3,'%.3f'),' ms ',char(9), ...
          'xRef=',num2str(Xref(1,1),'%07.4f'),char(9), ...
          'yRef=',num2str(Xref(1,2),'%07.4f'),char(9), ...
          'hRef=',num2str(rad2deg(Xref(1,3)),'%07.4f'),char(9), ...
          'err_d=',num2str(norm(Xref(1,[1,2])'-state_sim(end,[1,2])'),'%.4f'),char(9), ...
          'k=',num2str(k)]);

    k = k + 1;
%     if k+N-1 >= length(xTrack), disp('End of Track'); break; end    % Only used for offline single segment simulation
    if k+N-1 >= length(xTrack)
        disp('End of Track segment');
        get_new_segment = 1;
        point_of_segment = 1;
        iter = 0;
        k = 1;
        
        if ROS_ENABLE == 1
            % Publish to high level
            msg.Pose.Pose.Position.X = states.value(1);
            msg.Pose.Pose.Position.Y = states.value(2);
            msg.Twist.Twist.Linear.X = states.value(4);
            msg.Pose.Pose.Orientation.Z = states.value(3);
            msg.Pose.Pose.Position.Z = get_new_segment;
            send(pub, msg);
        end
        
%         break;
        
    else
%             point_of_segment = point_of_segment +1 ;
        if ROS_ENABLE == 1
            msg.Pose.Pose.Position.X = states.value(1);
            msg.Pose.Pose.Position.Y = states.value(2);
            msg.Twist.Twist.Linear.X = states.value(4);
            msg.Pose.Pose.Orientation.Z = states.value(3);
            msg.Pose.Pose.Position.Z = get_new_segment;
            send(pub, msg);
        end
    end

end

%% Simulation - Plot
isSave = 0;
fWidth = 560;  %480;1122
fHeight = 335; %240;420

tk = time';
XSim = state_sim';
% XSim(3,:) = rad2deg(XSim(3,:));
% XSim(4,:) = 3.6.*XSim(4,:);
USim = input_sim';
% USim(1,:) = rad2deg(USim(1,:));

% State Trajectories
% figure;
% title('\bf State Trajectories','Interpreter','latex');
% for i=1:n
%     subplot(n,1,i);
%     plot(tk,XSim(i,:)');
%     if i==1, title(['$N($sec$) = ',num2str(N*Ts),'$'],'Interpreter','latex'); end
%     if i==n, xlabel('$t$','Interpreter','latex'); end
%     ylabel(sysStates(i),'Interpreter','latex');
%     grid on;
% end

% Input Trajectories
% figure;
% title('\bf Input Trajectories','Interpreter','latex');
% for i=1:m
%     subplot(m,1,i);
%     plot(tk(1:end-1),USim(i,:)');
%     if i==1, title(['$N($sec$) = ',num2str(N*Ts),'$'],'Interpreter','latex'); end
%     if i==m, xlabel('$t$','Interpreter','latex'); end
%     ylabel(sysInputs(i),'Interpreter','latex');
%     grid on;
% end

% Vehicle Trajectory Plot with Input Plots
% figure;
% subplot(2,2,[1,3]); hold on; grid on; axis equal; axis padded;
% plot(xTrack,yTrack,'-.');
% plot(XSim(1,:)',XSim(2,:)');
% plot(XSim(1,1), XSim(2,1),'ko');
% plot(Xref(:,1), Xref(:,2),'ro');
% title('\bf Vehicle Trajectory','Interpreter','latex');
% xlabel(sysStates(1),'Interpreter','latex');
% ylabel(sysStates(2),'Interpreter','latex');
% for i=1:m
%     subplot(2,2,i*2);
%     plot(tk(1:end-1),USim(i,:)');
%     if i==1, title(['$N($sec$) = ',num2str(N*Ts),'$'],'Interpreter','latex'); end
%     if i==m, xlabel('$t$','Interpreter','latex'); end
%     ylabel(sysInputs(i),'Interpreter','latex');
%     grid on;
% end

% Vehicle Trajectory Plot
wLine = 1;
figure; hold on; grid on; axis equal; axis padded;
title('\bf Vehicle Trajectory','Interpreter','latex');
title(['\bf Vehicle Trajectory \rm$(N=',num2str(N),', T_s=',num2str(Ts),')$'], ...
      ['$x_0=[',num2str(X0(1)),',',num2str(X0(2)),',',num2str(rad2deg(X0(3))),'^{\circ},',num2str(X0(4)),']$'], ...
      'Interpreter','latex');
plot(xTrack,yTrack,'--*','LineWidth',wLine,'MarkerSize',3);
plot(XSim(1,:)',XSim(2,:)','-*','LineWidth',wLine,'MarkerSize',3);
plot(XSim(1,1) ,XSim(2,1),'ko');
plot(Xref(:,1) ,Xref(:,2),'ro');
[marginX, marginY] = genTrackRef(d_safe,1000);
N_Obs = length(obstacle(:,1));
for i=1:N_Obs, plot(marginX+obstacle(i,1),marginY+obstacle(i,2),'m:','LineWidth',wLine); end
plot(obstacle(:,1),obstacle(:,2),'m.');
xlabel('$s_x$','Interpreter','latex');
ylabel('$s_y$','Interpreter','latex');
lgdLabel = {'Planner','$s_k$','$s_0$','Horizon','$d_{safe}$','$[o_x,o_y]$'};
lgdLabel(5+N_Obs) = lgdLabel(end); lgdLabel(N_Obs:5+N_Obs-1) = {''};
legend(lgdLabel,'Interpreter','latex','Location','southeast');
txtDim1 = [0.175, 0.75, 0.3, 0.1];
txtStr1 = {['$W\matrix{ s_x & ',num2str(input.W(1,1)),'&',num2str(input.WN(1,1)), ...
                  ' \cr s_y & ',num2str(input.W(2,2)),'&',num2str(input.WN(2,2)), ...
                 ' \cr \phi & ',num2str(input.W(3,3)),'&',num2str(input.WN(3,3)), ...
                    ' \cr v & ',num2str(input.W(4,4)),'&',num2str(input.WN(4,4)), ...
             ' \cr \delta_f & ',num2str(input.W(5,5)), ...
                    ' \cr a & ',num2str(input.W(6,6)),'}$']};
annotation("textbox",txtDim1,'String',txtStr1,'Interpreter','latex','FitBoxToText','on','BackgroundColor','white');
% txtDim2 = [0.35, 0.75, 0.3, 0.1];
% txtStr2 = {['$W_N\matrix{ s_x & ',num2str(input.WN(1,1)), ...
%                     ' \cr s_y & ',num2str(input.WN(2,2)), ...
%                    ' \cr \phi & ',num2str(input.WN(3,3)), ...
%                       ' \cr v & ',num2str(input.WN(4,4)),'}$']};
% annotation("textbox",txtDim2,'String',txtStr2,'Interpreter','latex','FitBoxToText','on','BackgroundColor','white');

% Combination Plot
% figure('Position',[230,240,1122,420]);
% for i=1:n
%     subplot(4,4,[i*4-3,i*4-3+1]);
%     plot(tk,XSim(i,:)');
%     if i==1, title(['$N($sec$) = ',num2str(N*Ts),'$'],'Interpreter','latex'); end
%     if i==n, xlabel('$t$','Interpreter','latex'); end
%     ylabel(sysStates(i),'Interpreter','latex');
%     grid on;
% end
% 
% subplot(4,4,[3,7,11,15]); hold on; grid on; axis equal; axis padded;
% plot(xTrack,yTrack,'-.');
% plot(XSim(1,:)',XSim(2,:)');
% plot(XSim(1,1) ,XSim(2,1),'ko');
% plot(Xref(:,1) ,Xref(:,2),'ro');
% title('\bf Vehicle Trajectory','Interpreter','latex');
% xlabel(sysStates(1),'Interpreter','latex');
% ylabel(sysStates(2),'Interpreter','latex');
% 
% for i=1:m
%     subplot(4,4,8*(i-1)+[4,8]);
%     plot(tk(1:end-1),USim(i,:)');
%     if i==1, title(['$N($sec$) = ',num2str(N*Ts),'$'],'Interpreter','latex'); end
%     if i==m, xlabel('$t$','Interpreter','latex'); end
%     ylabel(sysInputs(i),'Interpreter','latex');
%     grid on;
% end

%%
figure(2)
plot(ref_traj_list(:,1),ref_traj_list(:,2),'--*','LineWidth',wLine,'MarkerSize',3);
hold on
plot(XSim(1,:)',XSim(2,:)','-*','LineWidth',wLine,'MarkerSize',3);
plot(comptraj4.e00, comptraj4.e1,'-+','LineWidth',wLine,'MarkerSize',3)
legend("planned", "actual ego", "comp")


figure(5)
plot(ref_traj_list(1:21,1),ref_traj_list(1:21,2),'b')
hold on
% plot(ref_traj_list(:,1),ref_traj_list(:,2),'--*','LineWidth',wLine,'MarkerSize',3);
plot(ref_traj_list(22:42,1),ref_traj_list(22:42,2))
% plot(ref_traj_list(43:63,1),ref_traj_list(43:63,2),'*')
legend("1","2","3","4")

%% Helper functions
function [ X, Y ] = genTrackRef( R, n )

    theta = linspace(0,2*pi(),n)';
    X = R.*cos(theta);
    Y = R.*sin(theta);

end

function x = GetTraj(JointTrajectory)
    poses = reshape([JointTrajectory.points(:).positions],[3,21]);
%     poses = reshape(poses,[3,21])
%     x = poses(1,2:end);
%     y = poses(2,2:end);
%     phi = poses(3,2:end);
    v = [JointTrajectory.points(:).velocities];
%     v = v(2:end);
    x = [poses;v];
    % x is [x,y,phi,v] horizon sequence
end
