function [h]=createFOVNEW(agent)

xy=xyFOVnew(agent.pose.xyz(1:2),agent.thfov,agent.alphafov,agent.rfov);

hold on;
h=fill(xy(:,1), xy(:,2), 'w','LineStyle','--','EdgeColor', 'b','FaceAlpha', 0);



