close all; clc; clear
rosshutdown

%% initialize rosnode
rosinit

%% load evaluation driving data
load('/home/cralab/catkin_optimalProj/src/autonomous_racing/src/tracks/MPC_test_map.mat')
x_map_list = map(:,1);
y_map_list = map(:,2);


%% Simulation - Initialisation
sysStates  = {'$s_x$', '$s_y$', '$\phi$', '$v$'}; % x-COM, y-COM, inertial heading, vehicle speed
sysInputs  = {'$\delta_f$', '$a$'};               % front steering angle, acceleration
sysOutputs = {'$s_x$', '$s_y$', '$\phi$', '$v$'}; % x-COM, y-COM, inertial heading, vehicle speed

n = length(carStates);
m = length(carInputs);
n_U = m;

R = 50;     % Track radius
[xTrack,yTrack] = genTrackRef(R,1000);      % Circular track
xTrack1 = xTrack; yTrack1 = yTrack;

Tf = 900*.5;
X0 = [xTrack(1), -2, deg2rad(90), 0];   % sx, sy, phi, v    % Init conditions

% Time Varying Reference
vRef = 0;
[sxRef,syRef,phiRef] = genVehicleRef([X0(1:2)',[xTrack(1:N)';  yTrack(1:N)']], ...
                                     [xTrack(1:N+1)';         yTrack(1:N+1)']);
Xref = [sxRef(1:N)', syRef(1:N)', phiRef(1:N)', vRef.*ones(size(sxRef(1:N)'))];

input.x = [sxRef', syRef', phiRef', vRef.*ones(size(sxRef'))];

input.od = [];

Uref = zeros(N,n_U);
input.u = Uref;

input.y = [Xref(1:N,:), Uref];
input.yN = Xref(N,:);

input.W = diag([10,10,0.5,0,10,0.5]); % sx, sy, phi, v, delta_f, a
input.WN = diag([1,1,0.4,0]);


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

% visualize_learn;
while time(end) < Tf
%     tic
    
    % Solve NMPC OCP
    input.x0 = state_sim(end,:);
    output = autoraceMPCstep(input);
    % Save the MPC Step
    INFO_MPC = [INFO_MPC; output.info];
    KKT_MPC = [KKT_MPC; output.info.kktValue];
    controls_MPC = [controls_MPC; output.u(1,:)];
    input.x = output.x;
    input.u = output.u;
    
    
    % Simulate System
    sim_input.x = state_sim(end,:).';
    sim_input.u = output.u(1,:).';
    states = integrate_autorace(sim_input);
    state_sim = [state_sim; states.value'];
    input_sim = [input_sim; output.u(1,:)];
    
    iter = iter+1;
    nextTime = iter*Ts;
    time = [time, nextTime];
    disp(['Time: ',num2str(nextTime,'%06.1f'),char(9), ...
          'RTI step: ',num2str(output.info.cpuTime*1e3,'%.3f'),' ms ',char(9), ...
          'xRef=',num2str(Xref(end,1),'%07.4f'),char(9), ...
          'yRef=',num2str(Xref(end,2),'%07.4f'),char(9), ...
          'hRef=',num2str(rad2deg(Xref(end,3)),'%07.4f'),char(9), ...
          'err_d=',num2str(norm(Xref(end,[1,2])'-state_sim(end,[1,2])')),char(9), ...
          'k=',num2str(k)]);

    % Time Varying Reference Update
    if time(end)>Ts && mod(time(end),floor(time(end)))==0 && mod(time(end),1)==0
        k = k + 1;

        [sxRef,syRef,phiRef] = genVehicleRef([xTrack(k-1:k-1+N)'; yTrack(k-1:k-1+N)'], ...
                                             [    xTrack(k:k+N)';     yTrack(k:k+N)']);
        Xref = [sxRef(1:N)', syRef(1:N)', phiRef(1:N)', vRef.*ones(size(sxRef(1:N)'))];
        
        input.x = [sxRef', syRef', phiRef', vRef.*ones(size(sxRef'))];

        input.y = [Xref(1:N,:), Uref];
        input.yN = Xref(N,:);
    end
    
end

%% Simulation - Plot
isSave = 0;
fWidth = 560;%480;1122
fHeight = 335;%240;420

tk = time';
XSim = state_sim';
XSim(3,:) = rad2deg(XSim(3,:));
XSim(4,:) = 3.6.*XSim(4,:);
USim = input_sim';
USim(1,:) = rad2deg(USim(1,:));

% Trajectory Plot
wLine = 1;
figure; hold on; grid on; axis equal; axis padded;
title('Vehicle Trajectory','Interpreter','latex');
plot(xTrack,yTrack,'-.','LineWidth',wLine);
plot(XSim(1,:)',XSim(2,:)','LineWidth',wLine);
plot(XSim(1,1) ,XSim(2,1),'ko');

% Combination Plot
figure('Position',[230,240,1122,420]);
for i=1:n
    subplot(4,4,[i*4-3,i*4-3+1]);
    plot(tk,XSim(i,:)');
    if i==1, title(['$N(sec) = ',num2str(N*Ts),'$'],'Interpreter','latex'); end
    if i==n, xlabel('$t$','Interpreter','latex'); end
    ylabel(sysStates(i),'Interpreter','latex');
    grid on;
end

subplot(4,4,[3,7,11,15]); hold on; grid on; axis equal; axis padded;
plot(xTrack,yTrack,'-.');
plot(XSim(1,:)',XSim(2,:)');
plot(XSim(1,1) ,XSim(2,1),'ko');
plot(Xref(:,1) ,Xref(:,2),'ro');
title('Vehicle Trajectory','Interpreter','latex');
xlabel(sysStates(1),'Interpreter','latex');
ylabel(sysStates(2),'Interpreter','latex');

for i=1:m
    subplot(4,4,8*(i-1)+[4,8]);
    plot(tk(1:end-1),USim(i,:)');
    if i==1, title(['$N(sec) = ',num2str(N*Ts),'$'],'Interpreter','latex'); end
    if i==m, xlabel('$t$','Interpreter','latex'); end
    ylabel(sysInputs(i),'Interpreter','latex');
    grid on;
end

%% Helper functions
function [ X, Y ] = genTrackRef( R, n )

    theta = linspace(0,2*pi(),n)';
    X = R.*cos(theta);
    Y = R.*sin(theta);

end


