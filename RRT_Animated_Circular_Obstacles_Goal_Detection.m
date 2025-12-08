%% RRT with Goal Detection, Collision Checking, and Path Extraction
clear all; close all; clc;

%% Parameters
q_init = [0, 0];          % Initial configuration
q_goal = [8, 8];          % Goal configuration
goal_threshold = 0.3;     % Distance threshold for declaring goal reached

K = 10000;                % Maximum RRT iterations
Dq = 0.1;                 % Step size

q_max = [10, 10];
q_min = [-10, -10];

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
idx_init = 1;

%% Setup Figure
figure;
hold on;
axis([q_min(1) q_max(1) q_min(2) q_max(2)]);
axis equal;
grid on;
title('RRT Tree Growth (Goal Detection Enabled)');
xlabel('X');
ylabel('Y');

% Plot start and goal
plot(q_init(1), q_init(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
plot(q_goal(1), q_goal(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);

% Plot obstacles
theta = linspace(0, 2*pi, 50);
for i = 1:size(obstacles,1)
    cx = obstacles(i,1); cy = obstacles(i,2); r = obstacles(i,3);
    fill(cx + r*cos(theta), cy + r*sin(theta), [0.6 0.6 0.6], 'EdgeColor','k'); 
end

%% =============================
%%       MAIN RRT LOOP
%% =============================
goal_reached = false;
idx_goal_node = -1;

for i = 1:K

    %% 1. Random sample
    q_rand = [
        q_min(1) + (q_max(1) - q_min(1)) * rand, ...
        q_min(2) + (q_max(2) - q_min(2)) * rand
    ];

    %% 2. Find nearest existing vertex
    coords = [G.Nodes.X, G.Nodes.Y];
    [~, idx_nearest] = min(vecnorm(coords - q_rand, 2, 2));
    q_nearest = coords(idx_nearest,:);

    %% 3. Compute q_new
    direction = q_rand - q_nearest;
    dist = norm(direction);

    if dist < Dq
        q_new = q_rand;
    else
        q_new = q_nearest + (Dq/dist)*direction;
    end

    %% 4. Collision checking
    collision = false;
    for j = 1:size(obstacles,1)
        center = obstacles(j,1:2);
        r = obstacles(j,3);
        if ~collisionCircle(q_nearest, q_new, center, r)
            collision = true;
            break;
        end
    end
    if collision
        continue;
    end

    %% 5. Safe → Add node + edge
    G = addnode(G, table(q_new(1), q_new(2), 'VariableNames', {'X','Y'}));
    idx_new = numnodes(G);
    G = addedge(G, idx_nearest, idx_new);

    %% 6. Animate edge
    plot([q_nearest(1), q_new(1)], [q_nearest(2), q_new(2)], 'b-');
    drawnow limitrate;

    %% 7. Goal detection
    if norm(q_new - q_goal) < goal_threshold
        disp('🎯 Goal reached!');
        goal_reached = true;
        idx_goal_node = idx_new;
        break;
    end
end

%% =============================
%%       PATH EXTRACTION
%% =============================
if goal_reached
    disp('Backtracking path using shortestpath...');

    path = shortestpath(G, idx_init, idx_goal_node);

    % Extract coordinates
    px = G.Nodes.X(path);
    py = G.Nodes.Y(path);

    % Highlight final path
    plot(px, py, 'r-', 'LineWidth', 2);
    plot(px(1), py(1), 'ko', 'MarkerFaceColor','g', 'MarkerSize', 8);
    plot(px(end), py(end), 'ro', 'MarkerFaceColor','y', 'MarkerSize', 8);

    title('RRT — Final Path Found');

else
    disp('⚠ Goal NOT reached within iteration limit.');
end
