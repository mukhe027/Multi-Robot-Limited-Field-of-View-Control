%function [ links ] = plotLinks( N, agents, G, linewidth, color )
function [MAS] = plotLinks(MAS)
%PLOTMASLINKS Plots the links of a multi-agent system
N = MAS.n;
agents = MAS.agents;
G_fov = MAS.G_fov.*MAS.G_desired;
G_collision = MAS.G_collision;
linewidth = MAS.linkWidth;
color = MAS.linkColor;

links = zeros((N-1)^2, 1);
idx = 1;
for i=1:N
    pos = agents{i}.pose.xyz(1:2);
    for j=1:N
        if agents{i}.state == 1
            % Topology graph
            if(G_fov(i,j) > 0)
                lc = color;
                lw = linewidth;
                nPos = agents{j}.pose.xyz(1:2);
                %if (G(j,i)>0)
                    %links(idx) = arrow([pos(1) pos(2)], [nPos(1) nPos(2)],'Length',0);
                    [x,y] = ds2nfu([pos(1),nPos(1)], [pos(2),nPos(2)]);
                    if ( all(x<=1) && all(x>=0) && all(y<=1) && all(y>=0) )
                        links(idx) = annotation('arrow', x, y);
                        set(links(idx), 'Color', 'k','LineWidth', 3);
                    end
%                 else
%                     %links(idx) = arrow([pos(1) pos(2)], [nPos(1) nPos(2)],'Length',15);
%                     [x,y] = ds2nfu([pos(1),nPos(1)], [pos(2),nPos(2)]);
%                     if ( all(x<=1) && all(x>=0) && all(y<=1) && all(y>=0) )
%                         links(idx) = annotation('arrow', x, y);
%                         set(links(idx), 'Color', 'k','LineWidth', 3);
%                     end
%                 end
                idx = idx+1;
            end
        elseif agents{i}.state == 2
            % Collision avoidance graph
            if(G_collision(i,j) > 0)
                lc = color;
                lw = linewidth;
                nPos = agents{j}.pose.xyz(1:2);
                %if (G_collision(j,i)>0)
                    %links(idx) = arrow([pos(1) pos(2)], [nPos(1) nPos(2)],'Length',0);
                    [x,y] = ds2nfu([pos(1),nPos(1)], [pos(2),nPos(2)]);
                    if ( all(x<=1) && all(x>=0) && all(y<=1) && all(y>=0) )
                        links(idx) = annotation('line', x, y);
                        set(links(idx), 'Color', 'g','LineWidth', 2.5);
                    end
%                 else % Should never enter inside this, G_collision is undirected
%                     %links(idx) = arrow([pos(1) pos(2)], [nPos(1) nPos(2)],'Length',15);
%                     [x,y] = ds2nfu([pos(1),nPos(1)], [pos(2),nPos(2)]);
%                     if ( all(x<=1) && all(x>=0) && all(y<=1) && all(y>=0) )
%                         links(idx) = annotation('arrow', x, y);
%                         set(links(idx), 'Color', 'g');
%                     end
%                 end
                idx = idx+1;
            end
        end
    end
end

MAS.links = links;

end
