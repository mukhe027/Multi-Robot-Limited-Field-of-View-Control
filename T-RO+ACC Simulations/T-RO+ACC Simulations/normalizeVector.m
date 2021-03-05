function [veln]=normalizeVector(vel)


veln=zeros(size(vel));

if norm(vel)>0    
    veln=vel/norm(vel);
end