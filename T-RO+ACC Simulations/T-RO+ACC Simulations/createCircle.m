function h=createCircle(xy,c) 

hold on;
%h=fill(xy(:,1), xy(:,2), 'w','LineStyle','--','EdgeColor', c,'FaceAlpha', 0);
h=plot(xy(:,1), xy(:,2),'LineStyle','--','Color', c);


