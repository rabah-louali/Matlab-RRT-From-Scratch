%% Rapidly exploring random tree (RRT) with animation
% Input: Initial configuration qinit, number of vertices in RRT K, incremental distance Δq
% Output: RRT graph G

clear all; close all; clc;

% Animate the tree growth in real time
% Check for collisions with obstacles before adding new edges
% Detect goal reach and backtrack to reconstruct the found path

% Parameters
q_init = [0, 0];     % Initial configuration
K = 10000;            % Number of vertices in RRT (set smaller for smooth animation)
Dq = 0.1;            % Step size
q_max = [10, 10];    % Upper bounds
q_min = [-10, -10];  % Lower bounds

% Initialise graph
G = graph();
G = addnode(G, table(q_init(1), q_init(2), 'VariableNames', {'X', 'Y'}));

%% Setup figure for animation
figure;
hold on;
axis([q_min(1) q_max(1) q_min(2) q_max(2)]);
axis equal;
grid on;
title('RRT Tree Growth');
xlabel('X');
ylabel('Y');

% Plot initial point
plot(q_init(1), q_init(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);

%% Main RRT loop
for i = 1:K
    % Generate random configuration
    q_rand = [q_min(1) + (q_max(1) - q_min(1)) * rand, ...
              q_min(2) + (q_max(2) - q_min(2)) * rand];

    % Find nearest vertex
    coords = [G.Nodes.X, G.Nodes.Y];
    [~, idx_nearest] = min(vecnorm(coords - q_rand, 2, 2));
    q_nearest = coords(idx_nearest, :);

    % Move a step towards q_rand
    direction = q_rand - q_nearest;
    dist = norm(direction);
    if dist < Dq
        q_new = q_rand;
    else
        q_new = q_nearest + (Dq/dist) * direction;
    end


    % Add new node and edge
    G = addnode(G, table(q_new(1), q_new(2), 'VariableNames', {'X','Y'}));
    idx_new = numnodes(G); % Find Id of the new added node. The new node is always added at the end of the node table.
    G = addedge(G, idx_nearest, idx_new); % Add to the Graph the new edge that links q_nearest to q_new  

    % === Animate new edge ===
    plot([q_nearest(1), q_new(1)], [q_nearest(2), q_new(2)], 'b-');
    drawnow limitrate; % Efficient refresh (use drawnow if you want smoother updates)

    % Optional: Slow down for visualization (comment this out for speed)
    % pause(0.001);
end

%% Final tree plot
plot(G, 'XData', G.Nodes.X, 'YData', G.Nodes.Y, ...
    'Marker', 'o', 'MarkerSize', 3, 'NodeLabel', {}, 'EdgeColor', 'b');
grid on;
title('Final RRT Tree');
