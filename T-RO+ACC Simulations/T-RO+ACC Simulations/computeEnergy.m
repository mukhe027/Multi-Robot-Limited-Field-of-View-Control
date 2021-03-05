function [checkF, u_reactive] = computeEnergy(i,agents,G,finite_energy_threshold)

u_reactive = [0 0]';
agent = agents{i};
n = length(agents);

% Agent pose
xi1 = agent.pose.xyz(1);
xi2 = agent.pose.xyz(2);
rho = agent.rho;

for j=1:n
    if G(j) > 0
        % Compute control law
        u_reactive = u_reactive + computeEnergy_ij(agent,agents{j}.pose.xyz(1:2));
    end
end

if norm(u_reactive) > finite_energy_threshold
    checkF = false;
else
    checkF = true;
end

end