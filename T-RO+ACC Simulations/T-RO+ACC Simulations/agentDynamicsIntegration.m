function [MAS] = agentDynamicsIntegration(MAS)

%% Local Variables
n = MAS.n;                  % Number of Agents
dt = MAS.dt;                % Sampling Time
d = MAS.d;                  % Sampling Time

% For each agent
for i=1:n
    
    agent = MAS.agents{i};
    
    %% Perform Discrete Dynamic Integration
    
    slow_factor = 1;
    
    MAS.pose.xyz(i,:) = (agent.pose.xyz + slow_factor*agent.u*dt)';
    MAS.pose.rpy(i,:) = (agent.pose.rpy + slow_factor*agent.u_ang*dt)';
    MAS.speed.xyz(i,:) = agent.u';
    MAS.speed.rpy(i,:) = agent.u_ang';
    
    %% Update Data Structure
    MAS.agents{i} = agent;
    
end
end