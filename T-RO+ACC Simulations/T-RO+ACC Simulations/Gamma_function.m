function [G_i] = Gamma_function(state_i,G_desired,G_fov,G_collision)
%Gamma_function Function Gamma
%   Use output of PI and Sigma to select the best possible topology in order to pursue
%   the objective defined by the finite-state machine

if state_i == 1
    % Reactive state
    G_i = G_fov.*G_desired;
elseif state_i == 2
    % Collision state
    G_i = G_collision;
elseif state_i == 3
    % Planning state
    G_i = zeros(1,length(G_desired));
end

end