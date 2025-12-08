%% Rapidly exploring random tree (RRT)
% Input: Initial configuration qinit, number of vertices in RRT K, incremental distance Δq
% Output: RRT graph G

%%
clear all;
close all;
clc;

q_init = [0,0]; % Initial configuration
K = 10000; % number of vertices in RRT
Dq = 0.1; % incremental distance
q_max = [10 , 10]; %  Limits of the configuration space C
q_min = [-10,-10]; %  Limits of the configuration space C

G = graph(); % Initialise RRT graph G
G = addnode(G, table(q_init(1), q_init(2), 'VariableNames', {'X','Y'})); % Add the initial configuration to the graph 

%Q_rand = [];

for i = 1 : K
    q_rand = [q_min(1) + (q_max(1) - q_min(1)) * rand , q_min(2) + (q_max(2) - q_min(2)) * rand]; % random configuration qrand in C 
    %Q_rand = [Q_rand;q_rand];

    % "FIND NEAREST_VERTEX": runs through all vertices v in graph G, 
    % calculates the distance between q_rand and v using some measurement function (Euclidian Distance)
    % thereby returning the nearest vertex. 
    coords = [G.Nodes.X, G.Nodes.Y];
    [~, idx_nearest] = min(vecnorm(coords - q_rand, 2, 2)); % vecnorm - returns the 2-norm or Euclidean norm. min - finds the min
    q_nearest = coords(idx_nearest, :);

    % "NEW_CONF" selects a new configuration q_new 
    % by moving an incremental distance Δq from qnear in the direction of qrand.
    direction = q_rand - q_nearest;             % Vector from nearest to random point
    dist = norm(direction);                     % Euclidean distance
    if dist < Dq
        q_new = q_rand;                         % If closer than step size, go directly
    else
        q_new = q_nearest + (Dq/dist) * direction;  % Step Dq toward q_rand
    end


    % Add the new vertex (node) q_new to the graph 
    G = addnode(G, table(q_new(1), q_new(2), 'VariableNames', {'X','Y'})); 
    idx_new = numnodes(G);
    G = addedge(G, idx_nearest, idx_new);
end


%% Plot the RRT Graph
p = plot(G, 'XData', G.Nodes.X, 'YData', G.Nodes.Y, 'MarkerSize', 1, 'NodeLabel', {}); grid on;
%figure; plot(Q_rand(:,1)); hold on; plot(Q_rand(:,2), 'r'); grid on;

