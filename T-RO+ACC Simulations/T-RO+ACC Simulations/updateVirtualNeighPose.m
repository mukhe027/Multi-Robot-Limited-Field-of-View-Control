function agent = updateVirtualNeighPose(agent)
agent_pose = agent.pose;
offset = agent.offset;

rpy = agent_pose.rpy;
agent_rotation = eul2rotm([rpy(3) rpy(2) rpy(1)]);

for k=1:agent.replicas
    % Position
    agent.vnbrs{k}.xyz = agent_pose.xyz + agent_rotation*offset{k};
    % Orientation
    agent.vnbrs{k}.rpy  = rpy;
end

end