close all; clc; clear
rosshutdown

%% initialize rosnode
rosinit

%% load evaluation driving data
load('/home/cralab/catkin_optimalProj/src/autonomous_racing/src/tracks/MPC_test_map.mat')
x_map_list = map(:,1);
y_map_list = map(:,2);
map_length = length(map);
phi_map_list = [atan2((y_map_list(2) - y_map_list(end)),(x_map_list(2) - x_map_list(end)))];
for i = 2:length(x_map_list) - 1 
    phi_map_list = [phi_map_list; atan2((y_map_list(i+1) - y_map_list(i-1)),(x_map_list(i+1) - x_map_list(i-1)))];
end
phi_map_list = [phi_map_list; atan2((y_map_list(1) - y_map_list(end-1)),(x_map_list(1) - x_map_list(end-1)))];
map_step = 0.2;

%% States

x = 0; 
y = 0;
v_ref = 15;
v = 0;
phi = 0; %-2.87; %-2.87; %1.5; %-1.3689;
theta = 0;

L = 2.75;
lf = 1.75;
lr = 1;

x_list = [];
y_list = [];
v_list = [];
ax_list = [];
ay_list = [];
phi_list = [];
phi_ref_traj = [];
theta_list = [];
d_theta_list = [];

%% Obstacle info
xo1 = 0;
yo1 = 100;
wo1 = 4;
ho1 = 5;

%% Subscribe/Publish
sub_traj = rossubscriber('traj', 'trajectory_msgs/JointTrajectory', 'DataFormat', 'struct'); 
pub = rospublisher('odom','nav_msgs/Odometry');

%% Initialize MPC
predict_horizon = 10;
delta_t_MPC = 0.5;
eva_no = predict_horizon/delta_t_MPC + 1;

x0 = [0, 0, 0, 0];
Xref = [0 0 0 v_ref];
input.x = repmat(x0,eva_no,1);
Xref = repmat(Xref,eva_no-1,1);

z = [.1];
input.z = repmat(z,eva_no-1,1);
Zref = repmat(z,eva_no-1,1);

Uref = zeros(eva_no-1,2);
input.u = Uref;

od = [xo1, yo1, 0, 0];
input.od = repmat(od,eva_no,1);
input.y = [Xref(1:eva_no-1,:) Zref];
input.yN = Xref(eva_no-1,:);

% Weights: 
input.W = diag([5 5 1 1 0]);
% Terminal Weights:
input.WN = diag([2 2 10]);

error_combined_list = [];

v_ref_vehFrame = ones(eva_no,1) * v_ref;

%% Visualization
figure(1)
plot(map(:,1),map(:,2),'LineWidth',1);
xlim([-50,100])
ylim([-50,450])
set(gcf,'Position',[100 100 300 800])
hold on
% rectangle('Position',[xo1-wo1/2 yo1 wo1 ho1])
h1 = animatedline('LineWidth',0.5,'MaximumNumPoints',11,'Color','red','LineStyle','--');
h2 = animatedline('LineWidth',0.5,'MaximumNumPoints',10000,'Color','red','LineStyle','-');
h3 = annotation('arrow');
set(h3,'parent', gca, ...
    'HeadLength', 10, 'HeadWidth', 5, 'HeadStyle', 'cback1',...
    'Color','blue','LineWidth',0.5);
h4 = annotation('arrow');
set(h4,'parent', gca, ...
    'HeadLength', 5, 'HeadWidth', 2.5, 'HeadStyle', 'cback1',...
    'Color','green','LineWidth',0.5);

s1 = scatter(0,0,'Filled');
s2 = scatter(0,0,'x','LineWidth',2, 'MarkerEdgeColor', 'red');

%% Initialize video
myVideo = VideoWriter('MPC','Uncompressed AVI'); %open video file
myVideo.FrameRate = 20;
open(myVideo)
frame = getframe(gcf); % get frame
writeVideo(myVideo, frame);

%% Simulation
delta_t = 0.05;
current_t = 0;
% r = rateControl(1/delta_t);
% reset(r);

coordinates = 2;
              % case 1 Coordinates in world frame    
              % case 2 Coordinates in vehicle frame
              
[near_index1_pre,near_dis1_pre] = dsearchn(map, [x,y]);
while(current_t <= 2000)   
    tic
    [near_index1,near_dis1] = dsearchn(map, [x,y]);
    
    if (near_index1_pre > near_index1 && near_index1_pre - near_index1 < 10) ||...
        (near_index1 - near_index1_pre > 300)
        near_index1 = near_index1_pre;
    end
    near_index1_pre = near_index1;
    
    %% Create raw reference points using future vehicle speed and map
%     v_low = max(v_ref, 0.1);
%     v_ref_list = ones(eva_no,1) * v_low;
%     ref_indices_raw = [];
%     ref_dis = 0;
%     for i = 2:eva_no
%         ref_dis = ref_dis + v_ref_list(i-1) * delta_t_MPC;
%         ref_indices_raw(i - 1, 1) = near_index1 + ref_dis/map_step;
%     end
%     
%     xy_ref_traj = [map(near_index1,:)];
%     phi_ref_traj = [phi_map_list(near_index1)];
%     for k = 1:length(ref_indices_raw) 
%         index_low = floor(ref_indices_raw(k));
%         index_fract = ref_indices_raw(k) - index_low;
%         index_low = loop_index(index_low, map_length);
%         index_high = loop_index(index_low + 1, map_length);
%         xy_ref = map(index_low,:) + (map(index_high,:) - map(index_low,:)) * index_fract;
%         xy_ref_traj = [xy_ref_traj; xy_ref];
%         phi_ref = phi_map_list(index_low,:) + (phi_map_list(index_high,:) - phi_map_list(index_low,:)) * index_fract;
%         phi_ref_traj = [phi_ref_traj; phi_ref];
%     end
%     
    trans_matrix = [cos(phi), sin(phi); -sin(phi), cos(phi)];
%     xy_ref_traj_vehFrame =  (trans_matrix*[xy_ref_traj(:,1)' - x ;xy_ref_traj(:,2)' - y])';
%     phi_ref_traj_vehFrame = wrapToPi(phi_ref_traj - phi);
    o1_worldFrame = [xo1, yo1];
    o1_vehFrame = (trans_matrix*[xo1 - x ;yo1 - y])';

%     pause(24);
    
    %% wait for next high level command
    i =1;
    while isempty(sub_traj.LatestMessage)
        i = i+1;
    end
    
    disp("here")
    %% Consume the subscriber data
    if ~isempty(sub_traj.LatestMessage)
        x_ilqrRef = GetTraj(sub_traj.LatestMessage);
        % got the  traj sequence
        xy_ref_traj_vehFrame = x_ilqrRef(1:2,:)';
        phi_ref_traj_vehFrame = x_ilqrRef(3,:)';
        v_ref_vehFrame = x_ilqrRef(4,:)';
    end

    disp("here1")
    %% Create reference to pass to mpc just from high level info

    %% Update MPC inputs
    switch coordinates
        case 1 % Coordinates in world frame
            Xref = [xy_ref_traj, phi_ref_traj, ones(eva_no,1) * 0, ones(eva_no,1) * v_ref, ones(eva_no,1) * 0, ones(eva_no,1) * 0];
            x0 = [x y phi v];
            o1 = o1_worldFrame;
        case 2 % Coordinates in vehicle frame
%             Xref = [xy_ref_traj_vehFrame, phi_ref_traj_vehFrame, ones(eva_no,1) * v_ref];
            Xref = [xy_ref_traj_vehFrame, phi_ref_traj_vehFrame, v_ref_vehFrame];
            x0 = [x y phi v];
            o1 = o1_vehFrame;
    end 
    
    input.x = repmat(x0,eva_no,1);
    input.od = repmat([o1, 0, 0],eva_no,1);
    input.y = [Xref(1:eva_no-1,:) Zref];
    input.yN = Xref(eva_no,1:3);
    input.x0 = x0;
    
    output = tracking_MPC(input);
    output.info.objValue;
    t1 = toc;
    
    switch coordinates
        case 1 % Coordinates in world frame
            xy_predict = output.x(:,1:2);
        case 2 % Coordinates in vehicle frame
            xy_predict = ([cos(-phi), sin(-phi); -sin(-phi), cos(-phi)] * [output.x(:,1)';output.x(:,2)'] + [x;y])';
    end
    
    %% Update vehicle states
    theta = output.u(1, 1);
    ax = output.u(1, 2);
    beta = output.z(1,1);
    beta = atan2(lr*tan(theta),(lr+lf));
    x = x + v * cos(phi) * delta_t;
    y = y + v * sin(phi) * delta_t;
    phi = phi + v * tan(beta) / L * delta_t;
    phi = wrapToPi(phi);
    v = v + ax * delta_t;
    theta = min(max(theta, -0.575), 0.575);
    ay = v^2/L*tan(theta);

    [x y phi v theta ax beta]

    x_list = [x_list; x];
    v_list = [v_list; v];
    y_list = [y_list; y];
    phi_list = [phi_list; phi];
    theta_list = [theta_list; theta];
    ax_list = [ax_list;  ax];
    ay_list = [ay_list; ay];
%     d_theta_list = [d_theta_list; d_theta];

    %% Publish to high level
    msg =  rosmessage(pub);
    msg.Pose.Pose.Position.X = x;
    msg.Pose.Pose.Position.Y = y;
    msg.Twist.Twist.Linear.X = v;
    msg.Pose.Pose.Orientation.Z = phi;
    send(pub, msg);
        
    %% Visualization
%     addpoints(h1,output.x(:,1),output.x(:,2));
    addpoints(h1, xy_predict(:,1),xy_predict(:,2));
    addpoints(h2,x,y);
%     set(s1,'XData',xy_ref_traj(:,1),'YData',xy_ref_traj(:,2));
    set(s1,'XData',xy_ref_traj_vehFrame(:,1),'YData',xy_ref_traj_vehFrame(:,2));

    set(s2,'XData',xy_predict(:,1),'YData',xy_predict(:,2)) ;
    dx = cos(phi) * L;
    dy = sin(phi) * L;
    dxx = cos(phi + theta) * 0.5 * L;
    dyy = sin(phi + theta) * 0.5 * L;
    set(h3,'position', [x,y,dx,dy]);
    set(h4,'position', [x,y,dxx,dyy]);
    
    drawnow
    
    pause(0.01) %Pause and grab frame
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
    
    t2 = toc;
    current_t = current_t + delta_t;
    [t1 t2 v ay];
end

close(myVideo)


function index = loop_index(index_raw, list_length)
    index = index_raw;
    
    if index_raw > list_length
        index = rem(index_raw,list_length);
    end
    
    if index_raw < 1
        index = rem(index_raw,list_length);
        index = list_length + index;
    end
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