function [h] = updateFOV(h,agent)

[xy1,xy2,xy3] = updatexyFOV(agent);

h(1) = updateCircle(h(1),xy1);
h(2) = updateCircle(h(2),xy2);
h(3) = updateCircle(h(3),xy3);