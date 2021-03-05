function [MAS]=updateGraphics(MAS,iter)%, lam2)


%% Local Variables
n = MAS.n;
d = MAS.d;
xyz = MAS.pose.xyz;
xyz_dist=dist(xyz');
l = MAS.l;
rho = MAS.rho;
offset_text = MAS.offset_text;
magnitude = 3;

if (MAS.showGraphics)
    %% Data
    graphics = MAS.graphics;
    figure(graphics.h);
    
    %% Centering the plot
    %     offset = mean(xyz(:,1:d),1);
    %     offset = [offset; offset];
    %     offset = offset(:)';
    %     axis_val = round(offset + MAS.plotRange);
    
    if (mod(MAS.iter,25)==0 || MAS.magichappened)
        minxy = min(xyz(:,1:d));
        maxxy = max(xyz(:,1:d));
        axis_val = [minxy(1) maxxy(1) minxy(2) maxxy(2)]+MAS.plotRange;
        axis(axis_val);
        MAS.axis = axis_val;
    end
    
    %% Re-Drawing
    for i=1:n
        
        % Show Agents
        if (MAS.showAgents)
            if (d==3)
                set(graphics.pos(i),'xdata', xyz(i,1), 'ydata', xyz(i,2), 'zdata', xyz(i,3));
            else
                set(graphics.pos(i),'xdata', xyz(i,1), 'ydata', xyz(i,2));
            end
            
            % Show agent's IDs
            if (MAS.showIDs)
                set(graphics.label(i),'Position',[xyz(i,1:2)'+MAS.labelPad; 0]);
            end
            
            % Show agent's Speed
            if (MAS.showSpeed)
                vel = MAS.agents{i}.speed.xyz(1:2);
                set(graphics.speed(i), 'XData',xyz(i,1),'YData',xyz(i,2),'UData',magnitude*vel(1),'VData',magnitude*vel(2));
            end
            
            % Show agent's Radius
            if(MAS.showRadius)
                % Define a circle around the robot location [To be implemented yet!]
                if (d==3)
                    % Generate the x, y, and z data for the sphere
                    r = rho * ones(20, 20);             % radius is rho
                    [th, phi] = meshgrid(linspace(0, 2*pi, 20), linspace(-pi, pi, 20));
                    [x,y,z] = sph2cart(th, phi, r);
                    x = x + xyz(i,1);                   % center at pos(i,1) in x-direction
                    y = y + xyz(i,2);                   % center at pos(i,2) in y-direction
                    z = z + xyz(i,3);                   % center at pos(i,3) in z-direction
                    set(graphics.surface(i),'xdata', x, 'ydata', y, 'zdata', z);
                else
                    % Define a circle around the robot location [To be implemented yet!]
                    posR = [xyz(i,1:d)-rho 2*[rho rho]];
                    set(graphics.rho(i),'Position',posR);
                end
            end
            
            % Show agent's FOV
            if(MAS.showFOV)
                %MAS.fov{i}=updateFOV(MAS.fov{i},MAS.agents{i});
                MAS.fov{i}=updateFOVNEW(MAS.fov{i},MAS.agents{i});
            end
            
            
            %             % Show Links
            %             if (MAS.showLinks)
            %
            %                 for j=i+1:n
            %
            %                     if (d==3)
            %                         if (xyz_dist(i,j)<rho)
            %
            %                             data=[xyz(i,:) ; xyz(j,:)];
            %                             set(graphics.edge(i,j), 'XData', data(:,1), 'YData', data(:,2), 'Zdata', data(:,3));
            %                         else
            %                             set(graphics.edge(i,j), 'XData', [0 0], 'YData', [0 0], 'Zdata', [0 0]);
            %                         end
            %                     else
            %                         if (xyz_dist(i,j)<rho)
            %                             data=[xyz(i,:) ; xyz(j,:)];
            %                             set(graphics.edge(i,j), 'XData', data(:,1), 'YData', data(:,2));
            %                         else
            %                             set(graphics.edge(i,j), 'XData', [0 0], 'YData', [0 0]);
            %                         end
            %                     end
            %
            %                 end
            %             end
            
        end
        
    end
    
    if (MAS.showLinks)
        % Update links
        for i=1:size(MAS.links,1)
            if(MAS.links(i) ~= 0)
                delete(MAS.links(i));
            end
        end
        MAS = plotLinks(MAS);
    end
    
    % Shows agents' barycenter
    if (MAS.centerOfGravity)
        barycenter = mean(xyz);
        set(graphics.bar,'xdata', barycenter(1), 'ydata', barycenter(2));
    end
    
    if (MAS.SHOW_DESIREDCENTERS)
        desAnim = MAS.desAnim;
        for i=1:n
            agent = MAS.agents{i};
            xd = agent.pose.xyz(1:2) + agent.Ld*[cos(agent.thfov);sin(agent.thfov)];
            set(desAnim{i},'XData',xd(1));
            set(desAnim{i},'YData',xd(2));
        end
        MAS.desAnim = desAnim;
    end
    
    if (MAS.SHOW_GAUSSIAN && mod(round(MAS.t),5)==0)%%(sim.t<0.5 || sim.t>0.95*sim.duration))
        gauss = MAS.gauss;
        for i=1:n
            agent = MAS.agents{i};
            xd = agent.pose.xyz(1:2) + agent.Ld*[cos(agent.thfov);sin(agent.thfov)];
            xy = xyFOVnew(agent.pose.xyz(1:2),agent.thfov,agent.alphafov,agent.rfov);
            
            %             % Based on triangle size
            minXY = min(xy);
            maxXY = max(xy);
            step = 100;
            stepX = (maxXY(1)-minXY(1))/step;
            stepY = (maxXY(2)-minXY(2))/step;
            %             [X, Y] = meshgrid(minXY(1):stepX:maxXY(1), minXY(2):stepY:maxXY(2));
            
            % Based on circle passing throught vertices of triangle
            c = 22.1;
            [X, Y] = meshgrid((xd(1)-c):stepX:(xd(1)+c), (xd(2)-c):stepY:(xd(2)+c));
            
            Z = 1- exp( - (0.1*(X-xd(1)).^2 + + 0.1*(Y-xd(2)).^2));
            set(gauss{i},'XData',X,'YData',Y,'ZData',Z);
        end
        MAS.gauss = gauss;
    end
    
    %% Update Title
    str=sprintf('Time: %0.2f / %0.2f',MAS.ct,MAS.t);
    title(str);
    
    %% Store Graphics
    MAS.graphics=graphics;
    
    %% Draw Graphics
    drawnow
end

end
