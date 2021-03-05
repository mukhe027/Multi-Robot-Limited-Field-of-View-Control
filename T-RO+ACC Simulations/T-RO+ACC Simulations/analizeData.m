iterazioni = MAS.iter;

ref_history_1_xyz = zeros(iterazioni,3);
ref_history_2_xyz = zeros(iterazioni,3);
ref_history_3_xyz = zeros(iterazioni,3);

for i=1:iterazioni
    ref_history_1_xyz(i,:) = MAS.agents{1}.points_history{i};
    ref_history_2_xyz(i,:) = MAS.agents{2}.points_history{i};
    ref_history_3_xyz(i,:) = MAS.agents{3}.points_history{i};
end

pose_history_1_xyz = zeros(iterazioni,3);
pose_history_2_xyz = zeros(iterazioni,3);
pose_history_3_xyz = zeros(iterazioni,3);
pose_history_1_rpy = zeros(iterazioni,3);
pose_history_2_rpy = zeros(iterazioni,3);
pose_history_3_rpy = zeros(iterazioni,3);

for i=1:iterazioni
    pose_history_1_xyz(i,:) = MAS.poseHist{i}.xyz(1,:);
    pose_history_2_xyz(i,:) = MAS.poseHist{i}.xyz(2,:);
    pose_history_3_xyz(i,:) = MAS.poseHist{i}.xyz(3,:);
    pose_history_1_rpy(i,:) = MAS.poseHist{i}.rpy(1,:);
    pose_history_2_rpy(i,:) = MAS.poseHist{i}.rpy(2,:);
    pose_history_3_rpy(i,:) = MAS.poseHist{i}.rpy(3,:);
end

figure
plot(pose_history_1_xyz)
hold on
grid on
plot(ref_history_1_xyz)
legend('x','y','z','x ref','y ref','z ref')
title('Agent 1')

figure
plot(pose_history_2_xyz)
hold on
grid on
plot(ref_history_2_xyz)
legend('x','y','z','x ref','y ref','z ref')
title('Agent 2')

figure
plot(pose_history_3_xyz)
hold on
grid on
plot(ref_history_3_xyz)
legend('x','y','z','x ref','y ref','z ref')
title('Agent 3')

% figure
% scatter3(pose_history_1_xyz(:,1),pose_history_1_xyz(:,2),pose_history_1_xyz(:,3))
% hold on
% grid on
% scatter3(ref_history_1_xyz(:,1),ref_history_1_xyz(:,2),ref_history_1_xyz(:,3),'filled','r')
% legend('pose','ref')
% title('Agent 1')
% 
% figure
% scatter3(pose_history_2_xyz(:,1),pose_history_2_xyz(:,2),pose_history_2_xyz(:,3))
% hold on
% grid on
% scatter3(ref_history_2_xyz(:,1),ref_history_2_xyz(:,2),ref_history_2_xyz(:,3),'filled','r')
% legend('pose','ref')
% title('Agent 2')
% 
% figure
% scatter3(pose_history_3_xyz(:,1),pose_history_3_xyz(:,2),pose_history_3_xyz(:,3))
% hold on
% grid on
% scatter3(ref_history_3_xyz(:,1),ref_history_3_xyz(:,2),ref_history_3_xyz(:,3),'filled','r')
% legend('pose','ref')
% title('Agent 3')