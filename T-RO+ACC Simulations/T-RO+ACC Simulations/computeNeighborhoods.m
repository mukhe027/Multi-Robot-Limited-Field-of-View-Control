% Function to compute the agents' neighborhoods

function [MAS] = computeNeighborhoods(MAS)

n = MAS.n;
MAS.G_potential = zeros(n);
MAS.G_collision = zeros(n);
MAS.G_fov = zeros(n);
G_desired = MAS.G_desired;
dist = cell(n);
ndist = zeros(n);
states = zeros(n,1);
planner_check = zeros(n,1);

%% Potential Topology

% Process established neighbors first to solidify neighbor counts and compute distances
for i=1:n
    ai = MAS.agents{i};
    
    for j=1:n
        if(j == i)
            continue;
        end
        aj = MAS.agents{j};
        
        % Compute agent distances
        rij = ai.pose.xyz(1:2) - aj.pose.xyz(1:2);
        nrij = norm(rij);
        
        % Store agent distances
        dist{i,j} = rij;
        dist{j,i} = -rij;
        ndist(i,j) = nrij;
        ndist(j,i) = nrij;
    end
    
    %% Function PI
    % - Encode low-level events generated directly from sensor data, e.g. object and neighbors detection
    % - Generate the largest set of possible topologies P corresponding to the feasible interactions among its neighborhood
    
    [G_potential_i,G_fov_i,G_collision_i] = PI_function(i,n,MAS.agents);
    ai.G_potential = G_potential_i;
    ai.G_fov = G_fov_i;
    ai.G_collision = G_collision_i;
    
    %% Function Sigma
    % - Depending on the output of PI selects the corresponding controlling action, i.e. reactive or collision
    % - Checks if planner's help is needed - TBD
    [state_i,ai,planner_check_i] = Sigma_function(ai,G_potential_i,G_desired(i,:),ndist(i,:),MAS.pose.xyz,MAS.finite_energy_threshold,MAS);
   state_i=1;
    ai.state = state_i;
    planner_check(i) = planner_check_i;
    
    %% Function Gamma
    % - Use output of PI and Sigma to select the best possible topology in order to pursue the objective defined by the finite-state machine
    G_i = Gamma_function(state_i,G_desired(i,:),G_fov_i,G_collision_i);
    ai.G = G_i;
    
%     %% Check energy-finiteness
%     [check_i, u_reactive] = computeEnergy(i,MAS.agents,G_i,MAS.finite_energy_threshold);
%     ai.u_reactive = u_reactive;
%     % If previous state was collision and now I'm trying to switch into reactive but the energy is not finite, stay in collision
%     if ai.previous_state == 2 && state_i == 1 && ~check_i
%         ai.state = 2;
%         ai.G = G_collision_i;
%         state_i = 2;
%     end
    
    %% Store data
    states(i) = state_i;
    ai.previous_state = state_i;
    MAS.agents{i} = ai;
end

%% Remove in-neighbors of the robots involved in collision avoidance
for i=1:n
    if states(i) == 2
        for j=1:n
            MAS.agents{j}.G_fov(i) = 0;
        end
    end
end

%% Planner evaluation and global state
if all(states==1)
    global_state = 1;
elseif sum(planner_check) < MAS.planner_threshold
    global_state = 2;
else %here we have sum(planner_check) >= MAS.planner_threshold
    global_state = 3;
end
MAS.machine_state = global_state;


%% Store data
MAS.dist = dist;
MAS.ndist = ndist;
for i=1:n
    MAS.G_potential(i,:) = MAS.agents{i}.G_potential;
    MAS.G_fov(i,:) = MAS.agents{i}.G_fov;
    MAS.G_collision(i,:) = MAS.agents{i}.G_collision;
end
