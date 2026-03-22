clc; clear; close all;

%% MAP INITIALIZATION (UNKNOWN MAP FOR SLAM)
gridSize = 50;
trueMap = zeros(gridSize);

% Static obstacles
trueMap(20:30, 20:30) = 1;
trueMap(10:15, 35:45) = 1;

% Dynamic obstacle initial position
dynObs = [25, 10];

% SLAM map (initially unknown)
slamMap = -1 * ones(gridSize); % -1 = unknown

start = [5, 5];
goal  = [45, 45];

%% PID PARAMETERS
Kp = 2; Ki = 0.01; Kd = 0.5;
integral = 0; prev_error = 0;

x = start(1); y = start(2); theta = 0;
dt = 0.1; v = 1;

trajX = []; trajY = [];

%% FUNCTION: A* PATH
function path = Astar(map, start, goal)
    gridSize = size(map,1);
    openSet = start;
    cameFrom = containers.Map;
    gScore = inf(gridSize);
    fScore = inf(gridSize);

    gScore(start(1), start(2)) = 0;
    fScore(start(1), start(2)) = norm(start - goal);

    dirs = [1 0; -1 0; 0 1; 0 -1];

    while ~isempty(openSet)
        [~, idx] = min(arrayfun(@(i) fScore(openSet(i,1), openSet(i,2)), 1:size(openSet,1)));
        current = openSet(idx,:);

        if isequal(current, goal)
            break;
        end

        openSet(idx,:) = [];

        for d = 1:4
            neighbor = current + dirs(d,:);

            if any(neighbor < 1) || any(neighbor > gridSize)
                continue;
            end

            if map(neighbor(1), neighbor(2)) == 1
                continue;
            end

            tentative_g = gScore(current(1), current(2)) + 1;

            if tentative_g < gScore(neighbor(1), neighbor(2))
                key = sprintf('%d,%d', neighbor(1), neighbor(2));
                cameFrom(key) = current;

                gScore(neighbor(1), neighbor(2)) = tentative_g;
                fScore(neighbor(1), neighbor(2)) = tentative_g + norm(neighbor - goal);

                if ~ismember(neighbor, openSet, 'rows')
                    openSet = [openSet; neighbor];
                end
            end
        end
    end

    % Reconstruct path
    path = goal;
    current = goal;
    while ~isequal(current, start)
        key = sprintf('%d,%d', current(1), current(2));
        if ~isKey(cameFrom, key)
            path = start;
            return;
        end
        current = cameFrom(key);
        path = [current; path];
    end
end

%% MAIN LOOP
for step = 1:500

    % ---- SLAM UPDATE (simulate sensor) ----
    radius = 3;
    for i = -radius:radius
        for j = -radius:radius
            xi = round(x) + i;
            yj = round(y) + j;

            if xi>0 && xi<=gridSize && yj>0 && yj<=gridSize
                slamMap(xi, yj) = trueMap(xi, yj);
            end
        end
    end

    % ---- ADD DYNAMIC OBSTACLE ----
    trueMap(dynObs(1), dynObs(2)) = 1;
    slamMap(dynObs(1), dynObs(2)) = 1;

    % Move dynamic obstacle
    dynObs(2) = dynObs(2) + 1;
    if dynObs(2) > gridSize
        dynObs(2) = 1;
    end

    % ---- PATH PLANNING ----
    mapForPlanning = slamMap;
    mapForPlanning(mapForPlanning == -1) = 0;

    path = Astar(mapForPlanning, round([x y]), goal);

    % ---- FOLLOW FIRST WAYPOINT ----
    if size(path,1) < 2
        continue;
    end

    target = path(2,:);

    desired_theta = atan2(target(2)-y, target(1)-x);

    error = desired_theta - theta;
    integral = integral + error*dt;
    derivative = (error - prev_error)/dt;

    omega = Kp*error + Ki*integral + Kd*derivative;

    theta = theta + omega*dt;
    x = x + v*cos(theta)*dt;
    y = y + v*sin(theta)*dt;

    prev_error = error;

    trajX(end+1) = x;
    trajY(end+1) = y;

    % Stop if reached goal
    if norm([x - goal(1), y - goal(2)]) < 1
        break;
    end
end

%% VISUALIZATION
figure; hold on; grid on; axis equal;

imagesc(slamMap');
colormap(gray);

plot(trajX, trajY, 'b', 'LineWidth', 2);
plot(start(1), start(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal(1), goal(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);

title('UAV Navigation with SLAM + Dynamic Obstacle Avoidance');
xlabel('X'); ylabel('Y');
