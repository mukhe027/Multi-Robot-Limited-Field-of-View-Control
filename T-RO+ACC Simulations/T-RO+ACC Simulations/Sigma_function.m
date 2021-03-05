function [state_i,agent,planner_check_i] = Sigma_function(agent,G_potential_i,G_desired_i,ndist_i,pose,finite_energy_threshold,MAS)
%Sigma_function Function Sigma
%   Depending on the output of PI selects the corresponding controlling action, i.e. reactive or collision
%   Checks if planner's help is needed - TBD
%   Returns 1 for Reactive, 2 for Collision, and 3 for Planning

collision_radius = agent.rho0;
collision_state = false;
planning_state = false;
finite_energy_check = true;
planner_check_i = 0;

for j=1:length(G_potential_i)
    if G_potential_i(j) == 1
        if ndist_i(j) < collision_radius
            collision_state = true;
        elseif G_desired_i(j) == 1
            check_energy = (norm(computeEnergy_ij(agent,pose(j,1:2))) < finite_energy_threshold);
            finite_energy_check = finite_energy_check && check_energy;
        end
    end
end

if all(G_potential_i == 0) && any(G_desired_i > 0)
    planning_state = true;
end

if ~collision_state && ~planning_state && finite_energy_check
    state_i = 1;
elseif collision_state && ~planning_state
    state_i = 2;
else
    state_i = 3;
end

% Save time instant planning state is active
if state_i == 3 && ~agent.planning_request
    agent.t_planning = MAS.ct;          %current time
    agent.planning_request = true;
elseif agent.planning_request
    % check how much time has passed
    if MAS.ct - agent.t_planning > MAS.planner_wait
        agent.planning_action = true;       % the planner needs to move him
        planner_check_i = 1;
    end
end

end