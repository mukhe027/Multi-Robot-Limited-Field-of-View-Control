function [xy1,xy2,xy3] = updatexyFOV(agent)

rotation = rad2deg(agent.thfov);

t1=deg2rad(-30+rotation):.01:deg2rad(30+rotation);
t2=deg2rad(-90+rotation):.01:deg2rad(-30+rotation);
t3=deg2rad(30+rotation):.01:deg2rad(90+rotation);

center = agent.pose.xyz(1:2);
radius = agent.rho;

center_left = agent.vnbrs{1}.xyz(1:2);
radius_left = agent.vnbrs{1}.radius;
center_right = agent.vnbrs{3}.xyz(1:2);
radius_right = agent.vnbrs{3}.radius;

xy1 = zeros(length(t1),2);
xy2 = zeros(length(t2),2);
xy3 = zeros(length(t3),2);

xy1(:,1) = radius * cos(t1) + center(1);
xy1(:,2) = radius * sin(t1) + center(2);
xy2(:,1) = radius_left * cos(t2) + center_left(1);
xy2(:,2) = radius_left * sin(t2) + center_left(2);
xy3(:,1) = radius_right * cos(t3) + center_right(1);
xy3(:,2) = radius_right * sin(t3) + center_right(2);