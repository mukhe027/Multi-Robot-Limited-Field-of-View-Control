%Function to generate the initial robots conditions
function [pose,A]=initAgentsPose(n,d,s,l,rho)


xyz = zeros(n,3);
rpy = zeros(n,3);

%Do while
disp('trying...')
while (1)
    %generate randome conditions
    xyz(:,1:d)=round((l-1)*rand(n,d))+rand(n,d);
    if (s>0)
        rpy(:,1:s) = 2*pi*rand(n,d);
    end
    %Compute the Adjacency matrix
    A=adjancencyMatrix(xyz,rho);
    % Compute the Laplacian Matrix
    L = diag(sum(A,1))-A;
    %If connected -> break, else -> continue
    if rank(L) == n-1; break; end
end

pose.xyz = xyz;
pose.rpy = rpy;

if n==3
%     pose.xyz = [0 0 0;
%                 1 0 0;
%                 2 0 0];
    pose.xyz = [0 0 0;
                5 2.5 0;
                6 -3.0 0];
             pose.rpy = [0 0 deg2rad(0);
                0 0 deg2rad(0);
                0 0 deg2rad(0)];
%               pose.xyz = [-0.5 -sqrt(3)/2 0;
%                 0 0 0;
%                 0.5 sqrt(3)/2 0];

            
%     pose.rpy = [0 0 deg2rad(60);
%                 0 0 deg2rad(60);
%                 0 0 deg2rad(0)];
elseif n==4
 

            
    % NEW SETUP     
     pose.xyz = [0 0 0;
                2 -6.0 0;
                6 -3.0 0;
                5 2.5 0];
             pose.rpy = [0 0 deg2rad(0);
                0 0 deg2rad(20);
                0 0 deg2rad(0)
                0 0 deg2rad(0)];
%     pose.xyz = 1.5*[0 0 0;
%                 3 1.5 0;
%                 0.25 3.5 0;
%                 -2 1.5 0];
%             
%     pose.rpy = [0 0 deg2rad(30);
%                 0 0 deg2rad(150);
%                 0 0 deg2rad(-150);
%                 0 0 deg2rad(250)];
elseif n==5
    pose.xyz = [0 0 0;
                0 -6.0 0;
                5 -7.0 0;
                5 2.5 0
                 6 -3.0 0;];
             pose.rpy = [0 0 deg2rad(0);
                0 0 deg2rad(10);
                0 0 deg2rad(40);
                0 0 deg2rad(0)
                0 0 deg2rad(0)];
end



fprintf('\nconnected!\n')


end