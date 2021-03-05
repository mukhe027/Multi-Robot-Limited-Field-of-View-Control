%%  Plot a swarm %%

function MAS=initGraphics(MAS)


%% Local Variables
n = MAS.n;                  % Number of Agents
d = MAS.d;                  % Space Dimension
s = MAS.s;                  % Angular Dimension
xyz = MAS.pose.xyz;         % Agents' Poses
xyz_dist=dist(xyz');        % Agents' Interdistances
vel = MAS.u_opt;            % Agents' Speed
l = MAS.l;                  % Environment One Dimension Size
rho = MAS.rho;              % Agent's Visibility
replicas = MAS.replicas;    % Number of virtual robots
if MAS.showFOV
    MAS.fov = cell(n,1);
end

%% Create Figure
if (MAS.showGraphics)
    graphics.h=figure('backings','off');
    set(graphics.h,'renderer','opengl')
    hold on;
    grid on;
    title('Time = 0');
    
    % axis square;
    if (d==3)
        view([45 29]);
    else
        box on;
    end
    
    %% Centering the plot
    minxy = min(xyz(:,1:d));
    maxxy = max(xyz(:,1:d));
    axis_val = [minxy(1) maxxy(1) minxy(2) maxxy(2)]+MAS.plotRange;
    axis(axis_val);
    MAS.axis = axis_val;
    
    %% Drawing
    for i=1:n
        
        % Show Agents
        if(MAS.showAgents)
            if (d==3)
                graphics.pos(i) = plot3(xyz(i,1), xyz(i,2), xyz(i,3), 'Marker', MAS.agentMarkerType, 'MarkerSize', MAS.agentMarkerSize, ...
                    'MarkerFaceColor', MAS.agentMarkerFaceColor,'MarkerEdgeColor','k');
            else % MAS.d==2
                graphics.pos(i) = plot(xyz(i,1), xyz(i,2), 'Marker', MAS.agentMarkerType, 'MarkerSize', MAS.agentMarkerSize, ...
                    'MarkerFaceColor', MAS.agentMarkerFaceColor,'MarkerEdgeColor','k');
            end
            
            % Show agent's IDs
            if (MAS.showIDs)
                graphics.label(i) = text(xyz(i,1)+MAS.labelPad(1), xyz(i,2)+MAS.labelPad(2), num2str(i),'FontSize',20);
            end
            
            % Show agent's Speed
            if (MAS.showSpeed)
                graphics.speed(i) = quiver(xyz(i,1),xyz(i,2),vel(1,i),vel(2,i),'Color',MAS.agentMarkerFaceColor);
            end
            
            % Show agent's Radius
            if (MAS.showRadius)
                if (d==3)
                    
                    % Generate the x, y, and z data for the sphere
                    r = rho * ones(20, 20); % radius is rho
                    [th, phi] = meshgrid(linspace(0, 2*pi, 20), linspace(-pi, pi, 20));
                    [x,y,z] = sph2cart(th, phi, r);
                    x = x + xyz(i,1);  % center at pos(i,1) in x-direction
                    y = y + xyz(i,2);  % center at pos(i,2) in y-direction
                    z = z + xyz(i,3);   % center at pos(i,3) in z-direction
                    % Now we use the surface command to add the sphere. We just need to set the FaceColor as desired.
                    graphics.surface(i)= surface(x,y,z,'FaceColor', 'none','EdgeColor', MAS.colorRadius,'EdgeAlpha',0.05);
                    
                else
                    posR = [xyz(i,1:d)-rho 2*[rho rho]];
                    graphics.rho(i) = rectangle('Position',posR,'Curvature',[1 1],'EdgeColor', MAS.colorRadius,'LineStyle',MAS.lineStyleRadius);
                end
            end
            
            if (MAS.showFOV)
                %                 if d==2 && replicas==3
                %                     MAS.fov{i} = createFOV(MAS.agents{i});
                %                 end
                if d==2
                    MAS.fov{i} = createFOVNEW(MAS.agents{i});
                end
            end
            
            % Show Links
            %             if (MAS.showLinks)
            %                 for j=i+1:n
            %
            %                     if (d==3)
            %                         if (xyz_dist(i,j)<rho)
            %                             data=[xyz(i,:) ; xyz(j,:)];
            %                             graphics.edge(i,j)=line(data(:,1),data(:,2),data(:,3),'Color',MAS.colorEdges,'LineStyle',MAS.lineStyleEdges);
            %                         else
            %                             graphics.edge(i,j)=line([0 0],[0 0],[0 0],'Color',MAS.colorEdges,'LineStyle',MAS.lineStyleEdges);
            %                         end
            %                     else
            %                         if (xyz_dist(i,j)<rho)
            %                             data=[xyz(i,:) ; xyz(j,:)];
            %                             graphics.edge(i,j)=line(data(:,1),data(:,2),'Color',MAS.colorEdges,'LineStyle',MAS.lineStyleEdges);
            %                         else
            %                             graphics.edge(i,j)=line([0 0],[0 0],'Color',MAS.colorEdges,'LineStyle',MAS.lineStyleEdges);
            %                         end
            %
            %                     end
            %                 end
            %             end
        end
        
    end
    
    % Shows agents' barycenter
    
    if (MAS.centerOfGravity)
        barycenter = mean(xyz);
        graphics.bar = plot(barycenter(1),barycenter(2),'Marker','o','MarkerFaceColor','m','Color','m');
    end
    
    if (MAS.showLinks)
        MAS = plotLinks(MAS);
    end
    
    if (MAS.SHOW_DESIREDCENTERS)
        desAnim = cell(n,1);
        for i=1:n
            desAnim{i} = scatter(0,0,100,'xk');
        end
        MAS.desAnim = desAnim;
    end
    
    if (MAS.SHOW_GAUSSIAN)
        gauss = cell(n,1);
        for i=1:n
            [~,gauss{i}] = contour([1 1;0 0],[1 0;1 0],eye(2),8,'LineStyle','-','LineWidth',1.5);
        end
        MAS.gauss = gauss;
    end
    
    %% Store Graphics
    MAS.graphics=graphics;
end

end