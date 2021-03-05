function [G_potential_i,G_fov_i,G_collision_i] = PI_function(i,n,agents)
%PI_function Function PI
%   Encode low-level events generated directly from sensor data, e.g. object and neighbors detection
%   Generate the largest set of possible topologies P corresponding to the feasible interactions among its neighborhood

%G_potential_i = zeros(n);
G_fov_i = zeros(1,n);
G_collision_i = zeros(1,n);

ai = agents{i};
pose_i = ai.pose.xyz(1:2);
fov_radius = ai.rho;
collision_radius = ai.rho0;

% Compute FOV polygon for ith agent (in our case a circle)
%xy = xyFOV(pose_i,fov_radius);
xy = xyFOVnew(pose_i,ai.thfov,ai.alphafov,ai.rfov);

for j=1:n
    
    if i == j
        continue;
    end
    
    pose_j = agents{j}.pose.xyz(1:2);
    
    %% FOV neighborhood
    
    % If agent j is in agent i FOV
    if (inpolygon(pose_j(1),pose_j(2),xy(:,1),xy(:,2)))
%         infov = true;
%        
%         % Check j is out of virtual agent i FOV
%         for k=1:ai.replicas
%             xyk = xyVirtualFOV(ai.vnbrs{k}); % FOV are circles
%             
%             outvfov = ~inpolygon(pose_j(1),pose_j(2),xyk(:,1),xyk(:,2));
%             infov = (infov && outvfov);
%         end
%        % if the agent j is in the agent i FOV but out of every FOV of the virtual neighbors of i
%        if (infov)
            %G_potential_i(i,j) = 1;
            G_fov_i(j) = 1;   % compute only the row relative to the agent i
%        end
    end
    
    %% Collision neighborhood
    if (norm(pose_i - pose_j) < collision_radius)
        G_collision_i(j) = 1;
    end
    
end

%% Total neighborhood
% Logical OR of the two previous graph
G_potential_i = G_fov_i + G_collision_i;
G_potential_i(G_potential_i > 1) = 1;


end
