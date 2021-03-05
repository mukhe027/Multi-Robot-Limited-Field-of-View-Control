function [xy]=xyFOVnew(pos,theta,alphafov,rfov)

xy = zeros(3,2);

R=[ cos(theta) -sin(theta) 
    sin(theta) cos(theta)
];

xy(1,:)=pos;                   % P1
xy(2,:)=pos+ R*[rfov*cos(alphafov);  -rfov*sin(alphafov) ];
xy(3,:)=pos+ R*[rfov*cos(alphafov);  +rfov*sin(alphafov) ];