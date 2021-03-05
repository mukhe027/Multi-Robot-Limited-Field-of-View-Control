function [xy]=xyVirtualFOV(vnbr)

% Field of View of the k-th virtual neighbor of the robot i (circle)

t=0:.01:2*pi;
center = vnbr.xyz(1:2);
radius = vnbr.radius;

xy = zeros(length(t),2);

xy(:,1) = radius * cos(t) + center(1);
xy(:,2) = radius * sin(t) + center(2);