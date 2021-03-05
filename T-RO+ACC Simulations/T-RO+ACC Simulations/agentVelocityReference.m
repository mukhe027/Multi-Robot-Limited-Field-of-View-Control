function [MAS] = agentVelocityReference(MAS)

%% Local Variables
N = MAS.n;                      % Number of Agents
xyz = MAS.pose.xyz;             % Pose
rpy = MAS.pose.rpy;             % Pose
rho = MAS.rho;
dist = MAS.dist;
ndist = MAS.ndist;

% Compute Basic Control Term
if MAS.machine_state == 3
    
    %% PLANNING STATE
    
    % Move the agents
    MAS.agents{4}.pose.xyz = MAS.agents{4}.pose.xyz + [0 -5 0]';
    MAS.agents{3}.pose.xyz = MAS.agents{4}.pose.xyz + [0 3 0]';
    MAS.agents{3}.pose.rpy = [0 0 deg2rad(260)]';
    MAS.agents{2}.pose.xyz = MAS.agents{3}.pose.xyz + [4 0 0]';
    MAS.agents{2}.pose.rpy = [0 0 deg2rad(180)]';
    MAS.agents{1}.pose.xyz = MAS.agents{2}.pose.xyz + [0 -5 0]';
    MAS.agents{1}.pose.rpy = [0 0 deg2rad(90)]';
    
    disp('Reallocating the agents...')
    pause
    
    % Clear variables
    for i=1:N
        MAS.agents{i}.vel = [0; 0];
        MAS.agents{i}.w = 0;
        MAS.agents{i}.planning_action = false;
        MAS.agents{i}.planning_request = false;
        MAS.agents{i} = updateVirtualNeighPose(MAS.agents{i});
    end
    
    MAS.magichappened = 1;
    
else
    
    %% Reactive or Collision state
    
    for i=1:N
        agent = MAS.agents{i};
        agent.state = 1;
        if(agent.isStationary)
            continue;
        end
        
        w = 0;
        vel = [0 0]';
        u_c = [0 0]';
        magical = [0 0]';
        
        % Agent pose
        xi1 = agent.pose.xyz(1);
        xi2 = agent.pose.xyz(2);
        
        %         % Agent data NEW
        theta = agent.thfov;            % agent's yaw (equal to pose.rpy(3))
        alpha = agent.alphafov;         % circular sector's amplitude
        Rfov = agent.rfov;              % triangle's size
        eta=0;
        Ld = agent.Ld;
        
        for j=1:N
            
            if(j == i)
                continue;
            end
            
            rij=dist{i,j};
            
            if agent.state == 1
                
                %% REACTIVE STATE
                
                % Is agent j a neighbor?
                if(agent.G(j) > 0)
                    
                    %                     % Moved this part in ComputeNeighborhoods
                    %                     % Neighbour j pose
                    %                     xj1 = MAS.agents{j}.pose.xyz(1);
                    %                     xj2 = MAS.agents{j}.pose.xyz(2);
                    %
                    %                     % Compute gradient for real neigh j
                    %                     grad_p1 = [-4*(xi1 - xj1)/((-(rho^2)+(xi1 - xj1)^2+(xi2 - xj2)^2)^3);
                    %                         -4*(xi2 - xj2)/((-(rho^2)+(xi1 - xj1)^2+(xi2 - xj2)^2)^3)];
                    %
                    %                     grad_pv = [0; 0];
                    %                     % Compute gradient for virtual neighbors
                    %                     for k=1:agent.replicas
                    %                         xv1 = agent.vnbrs{k}.xyz(1);
                    %                         xv2 = agent.vnbrs{k}.xyz(2);
                    %                         grad_pv = grad_pv + [-4*(xv1 - xj1)/((-(rho^2)+(xv1 - xj1)^2+(xv2 - xj2)^2)^3);
                    %                             -4*(xv2 - xj2)/((-(rho^2)+(xv1 - xj1)^2+(xv2 - xj2)^2)^3)];
                    %                     end
                    %
                    %                     % Compute control law
                    %                     vel = vel - grad_p1 - grad_pv;
                    %
                    %                     % Magical force in order to make 1 and 2 collide
                    %                     % (it's just consesus)
                    %                     if ~MAS.magichappened && MAS.ct >= 1.8 && i==1
                    %                         vel = vel + 0.8*dist{2,1};
                    %                     end
                    %                     if ~MAS.magichappened && MAS.ct >= 1.8 && i==2
                    %                         vel = vel + 0.8*dist{1,2};
                    %                     end
                    
                    %                       NEW STUFF
                    
                    
                    % Neighbour j pose
                    xj1 = MAS.agents{j}.pose.xyz(1);
                    xj2 = MAS.agents{j}.pose.xyz(2);
                    
                    % Theta control
                    
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
                    w=w-th1-th2-th3 - th4;
                    % uses only point to line
%                     w = w -th1-th2-th3;

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
                    vel = vel - vel21 - vel22 - vel23 - vel24;
                    % uses only point to line and
%                     vel = vel - vel21 - vel22 - vel23;
                    if i==1 && j==4
                       agent.ind_vel{j}=vel; 
                    end
                     if i==1 && j==5
                       agent.ind_vel{j}=vel; 
                     end
                     if i==2 && j==5
                       agent.ind_vel{j}=vel; 
                     end
                      if i==3 && j==5
                       agent.ind_vel{j}=vel; 
                     end
                    
                end
                
%              elseif agent.state == 2
                
                %% COLLISION STATE
                
%                  vel = 0;
                
%                 if agent.G(j) > 0
                    nrij = ndist(i,j);
                    rij = dist{i,j};
                    r = 1.05*agent.rho0;
                    u_c = u_c - rij*(1/nrij - r/nrij^2);
%                 end
                
            end
        end
        
        beta = 1;
        % Potential control gain
        k1 = beta*3;
        % Yaw control gain
        k2 = beta*1;
        
        % Avoidance
        k_av = 1;
        
        % Control action
        agent.vel  = k1 * vel + 0 * w + k_av*u_c;
        agent.w = w;
        
        
        % term to make the "leader" agent drift away
%         if (N==4 && i==4 && agent.state == 1)
%             agent.vel = agent.vel + [-3.0;0.0]- MAS.agents{3}.vel;%2*cos(2*MAS.ct)];
%         end
          k3=1;
         if (N==5 && i==5 && agent.state == 1)
            agent.vel = agent.vel + [4.0;1.0]- (k3 *MAS.agents{1}.ind_vel{i})-(k3 *MAS.agents{2}.ind_vel{i})-(k3 *MAS.agents{3}.ind_vel{i});%2*cos(2*MAS.ct)];
%             agent.vel = agent.vel + [4.0;3.0];%2*cos(2*MAS.ct)];
         end
         if (N==5 && i==4 && agent.state == 1)
            agent.vel = agent.vel + [4.0;-1.0]- (k3 *MAS.agents{1}.ind_vel{i});%2*cos(2*MAS.ct)];
%              agent.vel = agent.vel + [4.0;3.0];%2*cos(2*MAS.ct)];
        end
        
        MAS.agents{i} = agent;
        
    end
    
end





%% Update velocity
if MAS.ROS
    z_des = MAS.hover_height*ones(1,N);
else
    z_des = zeros(N,1);
end

for i=1:N
    
    agent = MAS.agents{i};
    
    z_ref = z_des(i);
    
    yaw_ref = agent.w;
    
    MAS.agents{i}.yaw_ref_history{MAS.iter} = yaw_ref;
    
    xy_ref = agent.vel;
    MAS.agents{i}.u = [xy_ref; z_ref];
    MAS.agents{i}.u_ang = [0; 0; yaw_ref];
    
    % Store Control Action
    MAS.agents{i}.linear.vel_ref = [xy_ref; z_ref];
    MAS.agents{i}.angular.vel_ref = [0; 0; yaw_ref];
    
    MAS.agents{i}.xy_speed_history{MAS.iter} = xy_ref';
end


end