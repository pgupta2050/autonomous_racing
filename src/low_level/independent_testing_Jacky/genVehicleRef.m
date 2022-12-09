function [ x, y, phi ] = genVehicleRef( p0, p1 )

% [ x, y, phi ] = genVehicleRef( p0, p1 )
% 
% OUTPUT:
%       x   := [x1, y1, ... , xN]
%       y   := [y1, y2, ... , yN]
%       phi := w.r.t global x-axis ([1;0]) [rad]
%
% INPUT:
%       p0  := [x0, x1, ... , x(N-1);
%               y0, y1, ... , y(N-1)]
%       p1  := [x1, x2, ... , xN;
%               y1, y2, ... , yN]
% 
% Sample command to test and verify this function:
% [~,~,pp] = genVehicleRef(zeros(2,4),[1,1;-1,1;-1,-1;1,-1]'); rad2deg(pp)
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

    x = p1(1,:);
    y = p1(2,:);

    v_x = [1;0];
    v = p1-p0;
    [~,nRef] = size(v);

    phi = acos( dot(v,repmat(v_x,1,nRef)) ./ vecnorm(v) );
%     if v(2) < 0, phi = 2*pi() - phi; end
    phi(v(2,:)<0) = 2*pi()-phi(v(2,:)<0);

end