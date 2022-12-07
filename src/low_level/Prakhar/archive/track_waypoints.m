%% Initialisation
format compact;
% close all;
clear all;
clc;

%% Track & Waypoints
ave_v_x = 5.0;  % [m/s]
ave_v_y = 2.5;  % [m/s]

ave_v_weight = 0.5;     % percentage of x-component
ave_v = ave_v_weight*ave_v_x + (1-ave_v_weight)*ave_v_y;
disp(['v_x = ',num2str(ave_v_x,'%.2f'),'m/s']);
disp(['v_y = ',num2str(ave_v_y,'%.2f'),'m/s']);
disp(['Weighted average speed = ',num2str(ave_v,'%.2f'),'m/s (',num2str(ave_v_weight*100),'% of v_x)']);

R = 50;                                         % Track radius
[xTrack,yTrack] = genCircularRef(R,[R,0],1000); % Circular track

numWaypt = 20;
[xWaypt,yWaypt] = genCircularRef(R,[R,0],numWaypt);
distWaypt = (pi()*R) / (numWaypt-1);
disp(['Arc length = ',num2str(distWaypt),'m']);
disp(['Average time travel = ',num2str(distWaypt/ave_v), ...
      's based on average speed of ',num2str(ave_v,'%.2f'),'m/s']);

initPos = [0;0];

%% Plot
wLine = 1;

figure; hold on; grid on; axis equal; axis padded;
plot(xTrack,yTrack,'k--','LineWidth',wLine/2);
plot(xWaypt,yWaypt,'r*');
plot(initPos(1),initPos(2),'ro','LineWidth',wLine);

%% Helper function
function [ X, Y ] = genCircularRef( R, c, n )

    theta = linspace(pi/2,-1/2*pi(),n)';
    X = R.*cos(theta) + c(1);
    Y = R.*sin(theta) + c(2);

end



