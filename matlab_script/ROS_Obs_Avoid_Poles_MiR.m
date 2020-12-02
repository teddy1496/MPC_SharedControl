% point stabilization + Multiple shooting + obstacle avoidance
clear all
close all
clc

%% import CasADi 

% CasADi v3.4.5
addpath('/home/devalla/SharedControlMir/matlab_files/casadi_matlab')
import casadi.*


%% Initialise ROS and required Subscribers and Publishers

try
    rosinit
catch ME
    fprintf('ROS Node is already initialised');
end

velocity_pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
robot_vel = rosmessage(velocity_pub);

pub_initial = rospublisher('/initial_pose', 'geometry_msgs/Pose');
initial_pose = rosmessage(pub_initial);

current_state_sub = rossubscriber('/gazebo/model_states',...
    'gazebo_msgs/ModelStates');

if_quit_loop = rossubscriber('/end_sim', 'std_msgs/Bool');
% near_obs = rossubscriber('/slow_robot', 'std_msgs/Bool');
goal_pose = rossubscriber('/leader_point', 'geometry_msgs/PoseStamped');

pause(1);


%% Define the problem as CasADi symbolics
% Optimization parameters
N =15; % prediction horizon
T = 0.4; % dt in Seconds
% Weighting Matrices
Q = zeros(3,3); Q(1,1) = 6;Q(2,2) = 6;Q(3,3) = 4; 
R = zeros(2,2); R(1,1) = 0.3; R(2,2) = 0.15; 

% rob_diam = sqrt((0.89/2)^2 + (0.58/2)^2);
rob_diam = 0.95;

v_max = 1; v_min = -v_max;
omega_max = 1; omega_min = -omega_max;

x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta');
states = [x;y;theta]; n_states = length(states);

v = SX.sym('v'); omega = SX.sym('omega');
controls = [v;omega]; n_controls = length(controls);

% Kinematic System Model r.h.s
rhs = [v*cos(theta);v*sin(theta);omega]; 

f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)
% Decision variables (controls)
U = SX.sym('U',n_controls,N); 
% Decision Variables (states)
X = SX.sym('X',n_states,(N+1));

% parameters (which include at the initial state of the robot and the reference state)
P = SX.sym('P',n_states + n_states);


obj = 0; % Objective function
g = [];  % constraints vector
% Symbolically compute constraints and Objective Function
st  = X(:,1); % initial state
g = [g;st-P(1:3)]; % initial condition constraints

% Define the equality constraints as symbolics
for k = 1:N
    st = X(:,k);  con = U(:,k);
    obj = obj+(st-P(4:6))'*Q*(st-P(4:6)) + con'*R*con; % calculate obj
    st_next = X(:,k+1);
    f_value = f(st,con);
    st_next_euler = st+ (T*f_value);
    g = [g;st_next-st_next_euler]; % compute constraints
    
end


% Add constraints for collision avoidance (Inequality contraint)

obs_x = [-4 -2 0.5 3 -2.5 0 2 -4 -1 3 -2 1 4 -1 1 3]; % meters
obs_y = [3.5 3 3.5 4 1.5 1.5 2 0 0 0.5 -2 -1 -1 -4 -3 -3]; % meters

obs_diam = 0.45; % meters


for k = 1:N+1
    for num_obs = 1:length(obs_x)
        g = [g ; -sqrt((X(1,k)-obs_x(num_obs))^2+(X(2,k)-...
            obs_y(num_obs))^2) + (rob_diam/2 + obs_diam/2)];
    end
end

vp_lims = 0.1;
vn_lims = 0.1;
wp_lims = 0.1;
wn_lims = 0.1;
for k = 1:N-1
    g = [g ; (U(1,k)-U(1,k+1))];
end

for k = 1:N-1
    g = [g ; (U(2,k)-U(2,k+1))];
end
%% Create the problem with contraint limits

% make the decision variable one column  vector
OPT_variables = [reshape(X,3*(N+1),1);reshape(U,2*N,1)];

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);
opts = struct;
opts.ipopt.max_iter = 10000;
opts.ipopt.print_level =0;
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-7;
solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

args = struct;
% equality constraints
args.lbg(1:3*(N+1)) = 0; 
args.ubg(1:3*(N+1)) = 0;

% obstacle constraints
length_obs_const = 3*(N+1)+length(obs_x)*(N+1);
args.lbg(3*(N+1)+1 : length_obs_const) = -inf;
args.ubg(3*(N+1)+1 : length_obs_const) = 0; 

% linear velocity decomposition constraints
args.lbg(length_obs_const+1 : length_obs_const+(N-1)) = -vn_lims;
args.ubg(length_obs_const+1 : length_obs_const+(N-1)) = vp_lims;

length_vp = length_obs_const+(N-1);
% angular velocity decomposition constraints
args.lbg(length_vp+1 : length_vp+(N-1)) = -wn_lims;
args.ubg(length_vp+1 : length_vp+(N-1)) = wp_lims;

% Bounds on the state variables (Boundary Conditions/Outer walls)
args.lbx(1:3:3*(N+1),1) = -4.45; %state x lower bound
args.ubx(1:3:3*(N+1),1) = 4.45; %state x upper bound
args.lbx(2:3:3*(N+1),1) = -4.45; %state y lower bound
args.ubx(2:3:3*(N+1),1) = 4.45; %state y upper bound
args.lbx(3:3:3*(N+1),1) = -inf; %state theta lower bound
args.ubx(3:3:3*(N+1),1) = inf; %state theta upper bound

% Bounds on the control Variables (Linear and angular velocity)
args.lbx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_min; %v lower bound
args.ubx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_max; %v upper bound
args.lbx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_min; %omega lower bound
args.ubx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_max; %omega upper bound

%% Initialise robot pose on Gazebo and decision variables

prompt = {'Enter x-Value:','Enter y-Value:', 'Enter theta-Value (Rad)'};
dlgtitle = 'Initial estimate';
dims = [1 35];
definput = {'-4.0','-4.0', '0.0'};
answer = inputdlg(prompt,dlgtitle,dims,definput);

x_initial = str2double(answer{1,1}); 
y_initial = str2double(answer{2,1}); 
theta_initial = str2double(answer{3,1});
% Initial condition
x0 = [x_initial ; y_initial ; theta_initial];

eul = [theta_initial 0 0];
quat = eul2quat(eul);
unpuase_gazebo = call(rossvcclient('/gazebo/unpause_physics'));

% Set robot to desired start pose on gazebo
set_pose_gazebo = rossvcclient('/gazebo/set_model_state');
set_pose_msg = rosmessage(set_pose_gazebo);
set_pose_msg.ModelState.ModelName = 'mir';
set_pose_msg.ModelState.Pose.Position.X = x_initial;
set_pose_msg.ModelState.Pose.Position.Y = y_initial;
set_pose_msg.ModelState.Pose.Position.Z = 0;
set_pose_msg.ModelState.Pose.Orientation.W = quat(1);
set_pose_msg.ModelState.Pose.Orientation.X = quat(2);
set_pose_msg.ModelState.Pose.Orientation.Y = quat(3);
set_pose_msg.ModelState.Pose.Orientation.Z = quat(4);

initial_pose.Position.X = x_initial;
initial_pose.Position.Y = y_initial;
initial_pose.Orientation.W = quat(1);
initial_pose.Orientation.X = quat(2);
initial_pose.Orientation.Y = quat(3);
initial_pose.Orientation.Z = quat(4);

set_position_gazebo = call(set_pose_gazebo, set_pose_msg);
send(pub_initial, initial_pose);

% Recieve user desired goal pose
final_pose = receive(goal_pose, 3);
x_final = final_pose.Pose.Position.X;
y_final = final_pose.Pose.Position.Y;
theta_quat_w = final_pose.Pose.Orientation.W;
theta_quat_x = final_pose.Pose.Orientation.X;
theta_quat_y = final_pose.Pose.Orientation.Y;
theta_quat_z = final_pose.Pose.Orientation.Z;
theta_eul = quat2eul([theta_quat_w, theta_quat_x, theta_quat_y, theta_quat_z]);
theta_final = theta_eul(1);
% Goal pose.
xs = [x_final ; y_final ; theta_final];

t0 = 0;
t(1) = t0;
% initialization of the control decision variables
u0 = zeros(N,2);        
% initialization of the states decision variables
X0 = repmat(x0,1,N+1)'; 

xx(:,1) = x0;
xs1(:,1) = xs; 
% Start MPC
mpciter = 0;
xs1 = [];
u_cl=[];

quit_while = receive(if_quit_loop, 10);

%%

% the main simulaton loop... it works as long as the error is greater
% than 10^-6 and the number of mpc steps is less than its maximum
% value.
main_loop = tic;
while (quit_while.Data == 0)
    args.p   = [x0;xs]; % set the values of the parameters vector
    % initial value of the optimization variables
    args.x0  = [reshape(X0',3*(N+1),1);reshape(u0',2*N,1)];
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    % extract control values for N=1
    u = reshape(full(sol.x(3*(N+1)+1:end))',2,N)'; 
    xx1(:,1:3,mpciter+1)= reshape(full(sol.x(1:3*(N+1)))',3,N+1)';
    u_cl= [u_cl ; u(1,:)];
    t(mpciter+1) = t0;
     
    % Send the control values to the MiR Gazebo Simulation
    robot_vel.Linear.X = u(1,1);
    robot_vel.Angular.Z = u(1,2);
    send(velocity_pub, robot_vel);    
    
    % Recieve current robot position from Gazebo
    rob_state = receive(current_state_sub,1);

    isMir = cellfun(@(x)isequal(x,'mir'),rob_state.Name);
    [row,col] = find(isMir);
    robot_pose_x = rob_state.Pose(row,1).Position.X;
    robot_pose_y = rob_state.Pose(row,1).Position.Y;

    eulers = quat2eul([rob_state.Pose(row,1).Orientation.W,...
        rob_state.Pose(row,1).Orientation.X, ...
        rob_state.Pose(row,1).Orientation.Y,...
        rob_state.Pose(row,1).Orientation.Z]);
    
    robot_orientation = eulers(1);
    
    % propogate values for next prediction
    x0 = [robot_pose_x; robot_pose_y; robot_orientation];
    t0 = t0 + T;
    u0 = [u(2:size(u,1),:);u(size(u,1),:)];
    
    xx(:,mpciter+2) = x0;
    xs1(:,mpciter+2) = xs;
    X0 = reshape(full(sol.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY
    
    % Shift trajectory to initialize the next step
    X0 = [X0(2:end,:);X0(end,:)];
    
    % Recieve user desired goal pose
    final_pose = receive(goal_pose, 3);
    x_final = final_pose.Pose.Position.X;
    y_final = final_pose.Pose.Position.Y;
    theta_quat_w = final_pose.Pose.Orientation.W;
    theta_quat_x = final_pose.Pose.Orientation.X;
    theta_quat_y = final_pose.Pose.Orientation.Y;
    theta_quat_z = final_pose.Pose.Orientation.Z;
    theta_eul = quat2eul([theta_quat_w, theta_quat_x, ...
        theta_quat_y, theta_quat_z]);
    theta_final = theta_eul(1);
    
    % Reference posture.
    xs = [x_final ; y_final ; theta_final];

    mpciter = mpciter + 1;
    quit_while = receive(if_quit_loop,1);
    
end


main_loop_time = toc(main_loop);
ss_error = norm((x0-xs),2)
average_mpc_time = main_loop_time/(mpciter+1)
xs1(:,1) = []; xs1 = [xs1 xs1(:,length(xs1))];
    
% Draw_MPC_PS_Obstacles_Dynamic (t,xx,xx1,xs1,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam)





