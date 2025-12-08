%% Rapidly exploring random tree (RRT) with animation + circular obstacles
clear all; close all; clc;

%% Parameters
q_init = [0, 0];      % Initial configuration
K = 10000;            % Number of vertices in RRT
Dq = 0.1;             % Incremental distance
q_max = [10, 10];     % Upper bounds
q_min = [-10, -10];   % Lower bounds

%% Define circular obstacles: [x_center, y_center, radius]
obstacles = [
    3,  3,  1.5;
   -2,  5,  2.0;
    6, -4,  1.2;
   -5, -3,  2.5
];

%% Initialize graph
G = graph();
G = addnode(G, table(q_init(1), q_init(2), 'VariableNames', {'X', 'Y'}));

%% Setup Figure
figure;
hold on;
axis([q_min(1) q_max(1) q_min(2) q_max(2)]);
axis equal;
grid on;
title('RRT Tree Growth (with Circular Obstacles)');
xlabel('X');
ylabel('Y');

% Plot initial point
plot(q_init(1), q_init(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);

% Plot obstacles
theta = linspace(0, 2*pi, 50);
for i = 1:size(obstacles,1)
    cx = obstacles(i,1); cy = obstacles(i,2); r = obstacles(i,3);
    fill(cx + r*cos(theta), cy + r*sin(theta), ...
         [0.6 0.6 0.6], 'EdgeColor','k'); 
end

%% =============================
%%       MAIN RRT LOOP
%% =============================
for i = 1:K

    %% 1. Random configuration
    q_rand = [q_min(1) + (q_max(1) - q_min(1)) * rand, ...
              q_min(2) + (q_max(2) - q_min(2)) * rand];

    %% 2. Find nearest vertex
    coords = [G.Nodes.X, G.Nodes.Y];
    [~, idx_nearest] = min(vecnorm(coords - q_rand, 2, 2));
    q_nearest = coords(idx_nearest, :);

    %% 3. Compute q_new (step toward q_rand)
    direction = q_rand - q_nearest;
    dist = norm(direction);
    if dist < Dq
        q_new = q_rand;
    else
        q_new = q_nearest + (Dq/dist) * direction;
    end

    %% 4. Collision checking with ALL circular obstacles
    collision = false;
    for j = 1:size(obstacles,1)
        center = obstacles(j,1:2);
        r = obstacles(j,3);

        if ~collisionCircle(q_nearest, q_new, center, r)
            collision = true;
            break;
        end
    end

    %% 5. Skip adding this node if collision detected
    if collision
        continue;
    end

    %% 6. Safe: add node + edge
    G = addnode(G, table(q_new(1), q_new(2), 'VariableNames', {'X','Y'}));
    idx_new = numnodes(G);
    G = addedge(G, idx_nearest, idx_new);

    %% 7. Animate safe edge
    plot([q_nearest(1), q_new(1)], [q_nearest(2), q_new(2)], 'b-');
    drawnow limitrate;

end

%% Final tree figure
figure;
plot(G, 'XData', G.Nodes.X, 'YData', G.Nodes.Y, ...
    'Marker', 'o', 'MarkerSize', 3, 'NodeLabel', {}, 'EdgeColor', 'b');
axis equal; grid on;
title('Final RRT Tree (Collision-Free)');
