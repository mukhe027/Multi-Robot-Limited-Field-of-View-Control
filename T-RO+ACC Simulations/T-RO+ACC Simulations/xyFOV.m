function [xy] = xyFOV(center,radius)

% Field of View of the agent i (circle)

t=0:.01:2*pi;

xy = zeros(length(t),2);

xy(:,1) = radius * cos(t) + center(1);
xy(:,2) = radius * sin(t) + center(2);