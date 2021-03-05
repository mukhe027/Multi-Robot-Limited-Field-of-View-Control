function [h] = updateFOVNEW(h,agent)

xy = xyFOVnew(agent.pose.xyz(1:2),agent.thfov,agent.alphafov,agent.rfov);

set(h,'XData',xy(:,1));
set(h,'YData',xy(:,2));