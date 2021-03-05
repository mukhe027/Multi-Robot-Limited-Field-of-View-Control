function [xyscaled]=scaledCircle(xy,sf)


xyscaled=(xy*sf+repmat(mean(xy),size(xy,1),1)-repmat(mean(xy*sf),size(xy,1),1));