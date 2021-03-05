function [V_ij] = computeEnergy_ij(agent,xj)

% Agent i pose
xi1 = agent.pose.xyz(1);
xi2 = agent.pose.xyz(2);

% Neighbour j pose
xj1 = xj(1);
xj2 = xj(2);


% OLD 
% % Interaction radius
% rho = agent.rho;
% 
% % Compute gradient for real neigh j
% grad_p1 = [-4*(xi1 - xj1)/((-(rho^2)+(xi1 - xj1)^2+(xi2 - xj2)^2)^3);
%     -4*(xi2 - xj2)/((-(rho^2)+(xi1 - xj1)^2+(xi2 - xj2)^2)^3)];
% 
% grad_pv = [0; 0];
% % Compute gradient for virtual neighbors
% for k=1:agent.replicas
%     xv1 = agent.vnbrs{k}.xyz(1);
%     xv2 = agent.vnbrs{k}.xyz(2);
%     grad_pv = grad_pv + [-4*(xv1 - xj1)/((-(rho^2)+(xv1 - xj1)^2+(xv2 - xj2)^2)^3);
%         -4*(xv2 - xj2)/((-(rho^2)+(xv1 - xj1)^2+(xv2 - xj2)^2)^3)];
% end
% 
% V_ij = - grad_p1 - grad_pv;


% NEW
theta = agent.thfov;            % agent's yaw (equal to pose.rpy(3))
alpha = agent.alphafov;         % circular sector's amplitude
Rfov = agent.rfov;              % triangle's size
eta=0;
Ld = agent.Ld;

% Triangle
th1=[2.*((xi2+(-1).*xj2).*cos(alpha+(-2).*eta+(-1).*theta)+(xi1+(-1).* ...
    xj1).*sin(alpha+(-2).*eta+(-1).*theta)).^(-3).*((xi1+(-1).*xj1).* ...
    cos(alpha+(-2).*eta+(-1).*theta)+((-1).*xi2+xj2).*sin(alpha+(-2).* ...
    eta+(-1).*theta))];

th2=[2.*((xi2+(-1).*xj2).*cos(alpha+2.*eta+theta)+((-1).*xi1+xj1).* ...
    sin(alpha+2.*eta+theta)).^(-3).*((xi1+(-1).*xj1).*cos(alpha+2.* ...
    eta+theta)+(xi2+(-1).*xj2).*sin(alpha+2.*eta+theta))];

th3=[(2.*((-1).*xi2+xj2).*cos(2.*eta+theta)+2.*(xi1+(-1).*xj1).*sin( ...
    2.*eta+theta)).*(Rfov.*cos(alpha)+(xi1+(-1).*xj1).*cos(2.*eta+ ...
    theta)+(xi2+(-1).*xj2).*sin(2.*eta+theta)).^(-3)];

% Gaussian
th4=[exp(1).^((-0.1E0).*(xi1+ ...
    (-0.1E1).*xj1+Ld.*cos(theta)).^2+(-0.1E0).*(xi2+(-0.1E1).*xj2+ ...
    Ld.*sin(theta)).^2).*Ld.*((0.2E0.*xi2+(-0.2E0).*xj2).*cos(theta) ...
    +((-0.2E0).*xi1+0.2E0.*xj1).*sin(theta))];

% uses both point to line and gaussian
w = - th1 - th2 - th3 - th4;
% uses only point to line
% w = -th1-th2-th3;

% Preserving FOV

% Triangle
vel21=[
    (-2).*sin(alpha+(-2).*eta+(-1).*theta).*((xi2+(-1).*xj2).*cos( ...
    alpha+(-2).*eta+(-1).*theta)+(xi1+(-1).*xj1).*sin(alpha+(-2).*eta+ ...
    (-1).*theta)).^(-3)
    (-2).*cos(alpha+(-2).*eta+(-1).*theta).*((xi2+...
    (-1).*xj2).*cos(alpha+(-2).*eta+(-1).*theta)+(xi1+(-1).*xj1).*sin( ...
    alpha+(-2).*eta+(-1).*theta)).^(-3)
    ];

vel22=[
    2.*sin(alpha+2.*eta+theta).*((xi2+(-1).*xj2).*cos(alpha+2.*eta+ ...
    theta)+((-1).*xi1+xj1).*sin(alpha+2.*eta+theta)).^(-3)
    (-2).*cos( ...
    alpha+2.*eta+theta).*((xi2+(-1).*xj2).*cos(alpha+2.*eta+theta)+(( ...
    -1).*xi1+xj1).*sin(alpha+2.*eta+theta)).^(-3)
    ];

vel23=1/2*[
    (-2).*cos(2.*eta+theta).*(Rfov.*cos(alpha)+(xi1+(-1).*xj1).*cos( ...
    2.*eta+theta)+(xi2+(-1).*xj2).*sin(2.*eta+theta)).^(-3)
    (-2).*sin( ...
    2.*eta+theta).*(Rfov.*cos(alpha)+(xi1+(-1).*xj1).*cos(2.*eta+ ...
    theta)+(xi2+(-1).*xj2).*sin(2.*eta+theta)).^(-3)
    ];

% Gaussian
vel24 = [0.2E0.*exp(1).^((-0.1E0).*(xi1+(-0.1E1).*xj1+Ld.*cos(theta)).^2+ ...
    (-0.1E0).*(xi2+(-0.1E1).*xj2+Ld.*sin(theta)).^2).*(xi1+(-0.1E1).* ...
    xj1+Ld.*cos(theta))
    0.2E0.*exp(1).^((-0.1E0).*(xi1+(-0.1E1).*xj1+ ...
    Ld.*cos(theta)).^2+(-0.1E0).*(xi2+(-0.1E1).*xj2+Ld.*sin(theta)) ...
    .^2).*(xi2+(-0.1E1).*xj2+Ld.*sin(theta))];

% uses both point to line and gaussian
vel = - vel21 - vel22 - vel23 - vel24;
% uses only point to line and
% vel = - vel21 - vel22 - vel23;

V_ij = w + vel;

end