function [MAS] = initMATLAB(MAS)

n = MAS.n;
d = MAS.d;
speed.xyz = [0 0 0];
speed.rpy = [0 0 0];
linear_speed = zeros(n,3);
rpy_speed = zeros(n,3);

% Agent Location Random Generation
[pose,G] = initAgentsPose(n,d,MAS.s,MAS.l,MAS.rho);

for i=1:n
    agent.id = i;
    
    % Interaction variables
    agent.d = MAS.d;
    agent.rho0 = MAS.rho0;
    agent.rho = MAS.rho;
    agent.isStationary = false;
    
    %FOV new
    agent.rfov=10;
    agent.alphafov=deg2rad(45);
    agent.thfov = pose.rpy(i,3);
    agent.xyfov=zeros(3,2);
    agent.xyhistfov=zeros(3,2);
    agent.histsf=0.99; %0.7
    agent.Ld = 5;
    
    % Hysteresis variables
    agent.nbrs = double(find(G(i,:)==1));
    agent.addCand = [];
    agent.delCand = [];
    agent.repel = [];
    agent.attract = [];
    
    % Virtual neighbours
    agent.replicas = MAS.replicas;              % Number of virtual neighbors
    agent.offset = MAS.virtual_offset;
    agent.vnbrs = cell(agent.replicas,1);
    for k=1:agent.replicas
        agent.vnbrs{k}.xyz = [0 0 0]';
        agent.vnbrs{k}.rpy = [0 0 0]';
        agent.vnbrs{k}.radius = agent.rho;      % Radius of virtual neighbor
    end
    
    % Planning state
    agent.planning_request = false;
    agent.t_planning = 0;
    agent.waiting_time = 0;
    agent.planning_action = false;
    agent.previous_state = 1;
    
    % Structures
    agent.u = zeros(3,1);
    agent.u_ang = zeros(3,1);
    agent.u1 = zeros(2,1);
    agent.u2 = zeros(2,1);
    agent.u3 = zeros(2,1);
    agent.vel = zeros(2,1);
    agent.vel1 = zeros(2,1);
    agent.w = 0;
    agent.velc = 0;
    agent.vel_ref.linear = zeros(1,3);
    agent.vel_ref.angular = zeros(1,3);

    agent.points_history{1} = [0 0 0];
    agent.xy_speed_history{1} = [0 0];
    
    agent.pose.xyz = pose.xyz(i,:)';             % Agent Pose Data Structure
    agent.pose.rpy = pose.rpy(i,:)';             % Agent Pose Data Structure
    agent.speed = speed;
    
    agent = updateVirtualNeighPose(agent);
    
    MAS.agents{i} =  agent;     % Store Agent Data Structure
end

%% Data Structure
MAS.u_opt = zeros(d,n);
MAS.pose = pose;
MAS.speed.xyz = linear_speed;
MAS.speed.rpy = rpy_speed;

%% Initialize Graph and related Data
MAS = computeNeighborhoods(MAS);

end