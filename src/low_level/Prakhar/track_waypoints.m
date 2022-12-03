%% Initialisation
format compact;
% close all;
clear all;
clc;

%% Track & Waypoints
ave_v_x = 5.0;  % [m/s]
ave_v_y = 2.5;  % [m/s]

% ave_v_weight = 0.5;     % percentage of x-component
% ave_v = ave_v_weight*ave_v_x + (1-ave_v_weight)*ave_v_y;
ave_v = sqrt(ave_v_x^2 + ave_v_y^2);
disp(['v_x = ',num2str(ave_v_x,'%.2f'),'m/s']);
disp(['v_y = ',num2str(ave_v_y,'%.2f'),'m/s']);
% disp(['Weighted average speed = ',num2str(ave_v,'%.2f'),'m/s (',num2str(ave_v_weight*100),'% of v_x)']);
disp(['Resultant speed = ',num2str(ave_v,'%.2f'),'m/s']);


R = 50;                                         % Track radius
[xTrack,yTrack] = genCircularRef(R,[0,R],1000); % Circular track

% [xTrack,yTrack] = genEllipseRefV1([R*2,R],[0,R],100); % Ellipse track

Rx = R;     Ry = R/2;   % Fat ellipse
% Rx = R/2;   Ry = R;     % Tall ellipse
e = sqrt( 1-(Ry^2)/(Rx^2) );    % Rx:=axisX; Ry:=axisY;
[xTrack,yTrack] = genEllipseRef([-Rx,0],[Rx,0],[0,Ry],e,100);   % Ellipse track


numWaypt = 10;
[xWaypt,yWaypt] = genCircularRef(R,[0,R],numWaypt);
% [xWaypt,yWaypt] = genEllipseRefV1([R*2,R],[0,R],numWaypt);
[xWaypt,yWaypt] = genEllipseRef([-Rx,0],[Rx,0],[0,Ry],e,numWaypt);
distWaypt = (pi()*R) / (numWaypt-1);
disp(['Arc length = ',num2str(distWaypt),'m']);
disp(['Average time travel = ',num2str(distWaypt/ave_v), ...
      's based on average/resultant speed of ',num2str(ave_v,'%.2f'),'m/s']);

initPos = [0;0];

%% Plot
wLine = 1;

figure; hold on; grid on; axis equal; axis padded;
plot(xTrack,yTrack,'k--','LineWidth',wLine/2);
plot(xWaypt,yWaypt,'r*');
plot(initPos(1),initPos(2),'ro','LineWidth',wLine);

%% Helper function
function [ X, Y ] = genCircularRef( R, C, n )

    theta_offset = -pi()/2;
    theta = linspace(0,1*pi(),n)' + theta_offset;
    X = R.*cos(theta) + C(1); X(abs(X)<1e-9) = 0;
    Y = R.*sin(theta) + C(2); Y(abs(Y)<1e-9) = 0;

end

function [ X, Y ] = genEllipseRefV1( R, C, n )

    Rx = R(1); Ry = R(2);
    Cx = C(1); Cy = C(2);
    
    X = [linspace(-Rx,Rx,n/2+1),linspace(Rx,-Rx,n/2+1)]'; X(n/2+2) = []; X(end) = [];
    Y = ( 1 - ((X-Cx).^2)./(Rx.^2) ) .* Ry.^2;
    Y = sqrt(Y);
    Y = Y + Cy;
    Y(ceil(length(Y)/2)+1:end) = -Y(ceil(length(Y)/2)+1:end) + Ry+Cy;

end

function [ X, Y ] = genEllipseRef( f1, f2, C, e, n )

    x1 = f1(1); y1 = f1(2);
    x2 = f2(1); y2 = f2(2);

    a = 1/2*sqrt((x2-x1)^2+(y2-y1)^2);
    b = a*sqrt(1-e^2);

    theta_offset = -pi()/2;
    t = linspace(0,1*pi,n)' + theta_offset;
    x = a*cos(t);
    y = b*sin(t);
    w = atan2(y2-y1,x2-x1);

    X = (x1+x2)/2 + x*cos(w) - y*sin(w); X = X + C(1); X(abs(X)<1e-9) = 0;
    Y = (y1+y2)/2 + x*sin(w) + y*cos(w); Y = Y + C(2); Y(abs(Y)<1e-9) = 0;

end






