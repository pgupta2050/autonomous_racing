close all; clc; clear
rosshutdown

load('/home/cralab/catkin_optimalProj/src/autonomous_racing/src/tracks/test_segment.mat')

%% initialize rosnode
rosinit

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

%% Subscribe/Publish
sub_traj = rossubscriber('traj', 'trajectory_msgs/JointTrajectory', 'DataFormat', 'struct'); 
pub = rospublisher('odom','nav_msgs/Odometry');

%% Initialize MPC
predict_horizon = 10; % seconds
delta_t_MPC = 0.5;
eva_no = predict_horizon/delta_t_MPC + 1;

% init X
x0 = [0, 0, 0, 0];
input.x = repmat(x0,eva_no,1);

Xref = [0 0 0 0];
Xref = repmat(Xref,eva_no-1,1);

% init algebraic state
z = [0];
input.z = repmat(z,eva_no-1,1);
Zref = repmat(z,eva_no-1,1);

Uref = zeros(eva_no-1,2);
input.u = Uref;

input.y = [Xref(1:eva_no-1,:) Uref];
input.yN = Xref(eva_no-1,:);

% Weights: 
input.W = diag([5 5 0 0 0 0]);
% Terminal Weights:
input.WN = diag([5 5 0 0]);

error_combined_list = [];

%% Visualization
figure(1)
% plot(map(:,1),map(:,2),'LineWidth',1);
xlim([-30,30])
ylim([-30,30])
% set(gcf,'Position',[100 100 300 300])
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
% open(myVideo)
% frame = getframe(gcf); % get frame
% writeVideo(myVideo, frame);

%% Simulation

% Let high level know controller is alove
get_new_segment = 1;

msg = rosmessage(pub);
msg.Pose.Pose.Position.X = x0(1);
msg.Pose.Pose.Position.Y = x0(2);
msg.Pose.Pose.Orientation.Z = x0(3);
msg.Twist.Twist.Linear.X = x0(4);
msg.Pose.Pose.Position.Z = get_new_segment;
send(pub, msg);

% set flag to begin loop
get_new_segment = 1;

ref_traj_list = [];


delta_t = 0.1;
current_t = 0;

while(current_t <= 2000)   
    tic
    
    %% wait for next high level command
    traj1 = receive(sub_traj,40);
    
    %% Consume the subscriber data
    x_ilqrRef = GetTraj(traj1);
    xy_ref_traj = x_ilqrRef(1:2,:)';
    phi_ref_traj = x_ilqrRef(3,:)';
    v_ref_traj = x_ilqrRef(4,:)';
    xTrack = xy_ref_traj(:,1);
    yTrack = xy_ref_traj(:,2);
    msg.Pose.Pose.Position.Z = get_new_segment;
    % wait to get new traj till segment end is reached
    get_new_segment = 0;

    ref_traj_list = [ref_traj_list; xy_ref_traj];
   
    %% Update MPC inputs   
    Xref = [xy_ref_traj, phi_ref_traj, v_ref_traj];
    
    x0 = [x y phi v];
    input.x0 = x0;
    
    input.x = repmat(x0,eva_no,1);
    
    input.y = [Xref(1:eva_no-1,:) Uref];
    input.yN = Xref(eva_no-1,:);
    
    
    output = tracking_MPC(input);
    output.info.objValue;
    t1 = toc;
    
    xy_predict = output.x(:,1:2);
    
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

    %% Publish to high level
    msg = rosmessage(pub);
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
    set(s1,'XData',xy_ref_traj(:,1),'YData',xy_ref_traj(:,2));

    set(s2,'XData',xy_predict(:,1),'YData',xy_predict(:,2)) ;
    dx = cos(phi) * L;
    dy = sin(phi) * L;
    dxx = cos(phi + theta) * 0.5 * L;
    dyy = sin(phi + theta) * 0.5 * L;
    set(h3,'position', [x,y,dx,dy]);
    set(h4,'position', [x,y,dxx,dyy]);
    
    drawnow
    
%     pause(0.01) %Pause and grab frame
%     frame = getframe(gcf); %get frame
%     writeVideo(myVideo, frame);
    
    t2 = toc;
    current_t = current_t + delta_t;
    [t1 t2 v ay];
end

% close(myVideo)


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
    v = [JointTrajectory.points(:).velocities];
    x = [poses;v];
end
