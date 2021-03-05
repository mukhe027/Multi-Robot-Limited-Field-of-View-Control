% Framework developed for simulating multi-agent systems
function [MAS] = RunMAS(n,ros,gazebo,AerialIDs,GroundIDs)

%% Basic setup
close all

% Input arguments
if nargin == 0
    error('Not enough input arguments');
elseif nargin == 1
    fprintf('ROS and Gazebo flag not selected, running in MATLAB.\n\n\n')
    ros = false;
    gazebo = false;
    AerialIDs = [];
    GroundIDs = [];
elseif nargin == 2
    if ros
        fprintf('Gazebo flag not selected, running without it.\n\n\n')
    else
        fprintf('Running simulation in MATLAB.\n\n\n')
    end
    gazebo = false;
    if ros && isempty(AerialIDs) && isempty(GroundIDs)
        error('Can not run in ROS without any agents!\n\n\n');
    end
    AerialIDs = [];
    GroundIDs = [];
elseif nargin == 3
    fprintf('Running using ROS and Gazebo.\n\n\n');
    if lenght(AerialIDs)+length(GroundIDs)>0
        fprintf('Warning: using Gazebo and real agents.\n\n\n');
    end
elseif nargin == 4
    fprintf('Using only Crazyflies.')
    if ~ros
        error('If you want to use CF, you need to select ROS flag');
    end
    if length(AerialIDs)~=n
        error('Number of agents and CFs do not match');
    end
    GroundIDs = [];
elseif nargin == 5
    if length(AerialIDs)+length(GroundIDs)~=n
        error('Number of agents n and actual agents do not match.\n\n\n');
    end
elseif nargin > 5
    error('Too many input arguments.\n\n\n');
end


%% Fundamental Parameters
Parameters.n = n;                               % Number of agents
Parameters.d = 2;                               % Dimension of the simulation [It can be either 2 or 3]
Parameters.kin = 2;                             % Kinematics [0: Single Integrator; 1: Unicycle; 2: Quadrotor]
Parameters.s = 0;                               % Angular Dimension [It can be either 0, 1, 2 or 3]
Parameters.dt = 1/100;                          % Sampling time [s]
Parameters.t = 30.0;                               % Simulation time [s]
Parameters.l = 5;                               % Environment One Dimension Size
Parameters.rho = 10;                             % Agent's Visibility
Parameters.rho0 = 3.5;                            % Agent's Collision Radius
Parameters.ROS = ros;                           % ROS Interaction
Parameters.GAZEBO = gazebo;                     % Gazebo
Parameters.Aerial_HW = 'CF';                    % Select between 'CF' and 'DJI'
Parameters.AerialIDs = AerialIDs;                     % Aerial Agents ID
Parameters.Ground_HW = 'Saetta';                % Only 'Saetta' available now
Parameters.GroundIDs = GroundIDs;                     % Ground Agents ID
Parameters.robot_name = 'ardrone';              % Robot model in Gazebo

%% Control data
if n==3
    G_desired = [ 0 1 1
        0 0 0
        0 0 0 ];
elseif n==4
%     G_desired = [ 0 0 1 0
%         0 0 1 0
%         0 0 0 1
%         0 0 0 0 ];
%         G_desired = [ 0 1 0 0
%         0 0 1 0
%         0 0 0 1
%         0 0 0 0 ];
   G_desired = [ 0 0 1 1
        0 0 1 0
        0 0 0 0 
        0 0 0 0];
elseif n==5
%     G_desired = [ 0 0 1 0 0
%         0 0 0 1 0
%         0 0 0 0 1
%         0 0 0 0 1
%         0 0 0 0 0 ];
%     error('Pose not yet implemented. Write it yourself in initAgentsPose.m');
   G_desired = [ 0 0 0 1 1
        0 0 0 0 1
        0 0 0 0 1
        0 0 0 0 0
        0 0 0 0 0];
else
    error('Not yet implemented. Write it yourself in RunMAS.m');
end
if length(G_desired) ~= n
    error('Size of desired graph and number of agents do not match');
end

Parameters.G_desired = G_desired;

%% Simulation data
% Parameters.eta = 0.05;
% Parameters.kca = 5.0;                                       % Collision avoidance
% Parameters.kag = 1.0;                                       % Aggregation
% Parameters.klm = 5.0;                                       % Link Maintenance
% Parameters.wpMargin = 0.05;                                 % Waypoint convergence margin
% Parameters.rho1 = 20;
% Parameters.rho2 = 25;
Parameters.planner_wait = 0.5;                                % How many seconds the planner waits before taking action
Parameters.planner_threshold = 1;%floor(0.5*n);               % How many agents needs to be stuck before planning
Parameters.finite_energy_threshold = 100;                     % Norm value for lyapunov function check  

%% Set offset of virtual neighbors
% Here we can add how many virtual neighbors we want, the code will adapt to it

% offset vector
% If we want we can use 3 different offset, for now we assume the same
offset = [Parameters.rho 0 0]';
% rotation matrix of virtual neighbors
rotation1 = eul2rotm([deg2rad(90) 0 0]);
rotation2 = eul2rotm([deg2rad(180) 0 0]);
rotation3 = eul2rotm([deg2rad(-90) 0 0]);

% In parameters.virtual_offset{k} needs to be saved the correct translation
Parameters.virtual_offset{1} = rotation1*offset;
Parameters.virtual_offset{2} = rotation2*offset;
Parameters.virtual_offset{3} = rotation3*offset;

Parameters.replicas = length(Parameters.virtual_offset);

%% Init Matlab for ROS
if ros
    % Ros Hostname
    % To get your hostname run 'hostname' on terminal/cmd
    Parameters.ROS_HOSTNAME = 'DESKTOP-VMP0EHV';         % Hostname
    Parameters.ROS_IP = '192.168.11.1';                  % Hostname address
    % Parameters.ROS_HOSTNAME = 'Andreas-MBP';
    
    % ROS Master Node
    % a. Detect the ip of the virtual machine (ifconfig eth0 from terminal)
    % b. Add the following line at the end of the file etc/hosts
    %    192.168.127.XXX	ROS-INDIGO
    %
    %    I.     Linux/Mac:  /etc/hosts
    %    II.    Windows:    C:\Windows\System32\drivers\etc\hosts
    % Both machine should now be able to ping each other
    Parameters.ROS_MASTER_URI = 'http://192.168.11.2:11311';          % Virtual Machine Address
end

%% MAF Data Structure Setting
MAS = initFramework(Parameters);

%% Cleanup Function as Callback [for ctrl+c]
finishup = onCleanup(@() myCleanupFun(MAS));

%% Initialize Graphics
MAS = initGraphics(MAS);

%% Perfor Simulation
MAS = simMAS(MAS);

%% Clean Graphics [Not Implemented Yet]
cleanGraphics(MAS);

%% Show Speed
plotAgentsSpeed(MAS);

%% Show Distance
% plotAgentsDistance(MAS);

% %% Show Lyapunov Evolution
%  plotAgentsLyapunov(MAS);

%% Cleanup The Environment
myCleanupFun(MAS);

end
