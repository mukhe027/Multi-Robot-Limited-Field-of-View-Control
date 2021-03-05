function [MAS] = initFramework(Parameters)

%% Simulation Variables
MAS.n = Parameters.n;                            % Number of Agents
MAS.d = Parameters.d;                            % Euclidean Space Dimension
MAS.s = Parameters.s;                            % Angular
MAS.kin = Parameters.kin;                        % Agent's Kinematic [0: single integrator; 1: unicycle; 2: quadrotor]
MAS.dt = Parameters.dt;                          % Sampling Time
MAS.t = Parameters.t;                            % Total Time
MAS.ct = 0;                                      % Current Time
MAS.l = Parameters.l;                            % Environment Size
MAS.rho = Parameters.rho;                        % Visibility
MAS.rho0 = Parameters.rho0;                      % Agents' repulsion interaction radius
% MAS.rho1 = Parameters.rho1;
% MAS.rho2 = Parameters.rho2;
MAS.iter = 0;                                    % MAS iterations
% MAS.eta = Parameters.eta;                        % Physical Occupancy
MAS.robot_name = Parameters.robot_name;          % Drone type
MAS.ROS = Parameters.ROS;
if MAS.ROS
    MAS.ROS_MASTER_URI = Parameters.ROS_MASTER_URI;
    MAS.ROS_HOSTNAME = Parameters.ROS_HOSTNAME;
    MAS.ROS_IP = Parameters.ROS_IP;
end
MAS.GAZEBO = Parameters.GAZEBO;
MAS.Aerial_HW = Parameters.Aerial_HW;
MAS.AerialIDs = Parameters.AerialIDs;
MAS.Ground_HW = Parameters.Ground_HW;
MAS.GroundIDs = Parameters.GroundIDs;
% MAS.sigma = sqrt(-(MAS.rho2)^2/(2*log(MAS.eta))); % Sigma for fading channel model based on interaction radius
MAS.virtual_offset = Parameters.virtual_offset;
% MAS.kca = Parameters.kca;
% MAS.kag = Parameters.kag;
% MAS.klm = Parameters.klm;
% MAS.wpMargin = Parameters.wpMargin;
MAS.replicas = Parameters.replicas;
MAS.G_desired = Parameters.G_desired;
MAS.planner_wait = Parameters.planner_wait;             % How many seconds the planner waits before taking action
MAS.planner_threshold = Parameters.planner_threshold;   % How many agents needs to be stuck before planning
MAS.global_state = 1;
MAS.magichappened = 0;
MAS.finite_energy_threshold = Parameters.finite_energy_threshold;

MAS.poseHist = [];
MAS.speedHist = [];
MAS.graphHist = [];


%% Graphics Variables
% Visibility
if (MAS.ROS)
    MAS.showGraphics =  false;
else
    MAS.showGraphics =  true;
end

MAS.showAgents =  true;
MAS.showSpeed = false;
MAS.showLinks =  true;
MAS.showRadius =  false;
MAS.centerOfGravity = false;
MAS.showIDs = true;
MAS.showFOV = true;
if MAS.replicas ~= 3
    MAS.showFOV = false;
end
MAS.SHOW_DESIREDCENTERS = true;
MAS.SHOW_GAUSSIAN = false;

%% Grahics Properties
if MAS.d == 2
    MAS.plotRange = [-MAS.l MAS.l -MAS.l MAS.l];
else
    MAS.plotRange = [-MAS.l MAS.l -MAS.l MAS.l -MAS.l MAS.l];
end

MAS.agentMarkerType = 'o';
MAS.agentMarkerFaceColor = 'r';
MAS.agentMarkerSize = 16;
MAS.lineStyleEdges = '-';
MAS.lineWidthEdges = '.5';
MAS.colorEdges = 'b';
MAS.lineStyleRadius = '--';
MAS.widthRadius = '.5';
MAS.colorRadius = 'k';
MAS.offset_text = 0.05;
MAS.labelPad = [0; -0.5];
MAS.linkWidth = 1;
MAS.linkColor = 'b';

%% ROS Data Structure
if (MAS.ROS)
    MAS = initROS(MAS);
else
    MAS = initMATLAB(MAS);
end

end