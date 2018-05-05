
global th_range,  global th_dot_range, global x_start, global x_goal;
th_range = [-pi/2, (3/2) * pi];
th_dot_range = [-10, 10];

figure(1); clf;

x_start = [0; 0];
x_goal = [pi; 0];

V_all = [x_start zeros(2, 9999)];
N = 1;
parents = zeros(1, 10000);
costs = zeros(1, 10000);
goal_reached = false;
goal_reached_dist = 0.25;
sample_dot = false;

global net;
load('dircol_net_akshay_ktrimmed.mat');
global distcounter;
distcounter = 0;

figure(1);
%     clf;
axis([th_range, th_dot_range]);
hold on;
% sample_dot = line();
% nearest_dot = line();
% near_dots = line();
% new_dot = line();
% dots = line();
% lines = line();

while ~goal_reached
    V = V_all(:, 1:N);
    
    axis([th_range, th_dot_range]);
    % Sample a random point
    x_rand = sample(0.1, x_goal, th_range, th_dot_range);
%     delete(sample_dot);
%     plot(x_rand(1), x_rand(2), 'r.');
    
    % Linearize around it
%     [K, S] = LQR(x_rand);
    
    % Find nearest node in tree to sample
    [x_nearest, i_nearest] = nearest(x_rand, [], V);
%     delete(nearest_dot);
%     plot(x_nearest(1), x_nearest(2), 'b.', 'MarkerSize', 5);
   
    % Extend tree toward sample
    x_new = extendEuclidean(x_nearest, x_rand);
%     delete(new_dot);
%     plot(x_new(1), x_new(2), 'b.', 'MarkerSize', 5);
    
    % Linearize around new node
%     [K_new, S_new] = LQR(x_new);
    
    % Collect near nodes, with shortest LQR dists from new node
%     I_near = near(x_new, S_new, i_nearest, V);
%     delete(near_dots);
% %     near_dots = plot(V(near_dots), V(near_dots), 'm.', 'MarkerSize', 5);
%     
%     % Select parent node, s.t. path from start to new node is shortest
%     [costs, parents] = choose_parent(x_new, I_near, S_new, costs, parents, V);
%     delete(near_dots);
% %     near_dots = plot(V(near_dots), V(near_dots), 'm.', 'MarkerSize', 5);
%   
%     % Redraw paths from start to near nodes thru new node, if total cost is less
%     [rewired, parents, costs] = rewire(x_new, I_near, costs, parents, V);
    
    V_all(:, N + 1) = x_new;
    N = N + 1
    
    plot(V(1, :), V(2, :), 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 5);
%         update_plot(sample_dot, x_rand, x_new, I_near, costs, parents, V_all(:, 1:N), N);
    if norm(x_goal - x_new) < goal_reached_dist
        nodes = N
        break;
    end
end

function x_rand = sample(p, x_goal, x_range, y_range)
    if rand(1) < p
        x_rand = x_goal;
    else
        x_rand = [
            rand(1) * (x_range(2) - x_range(1)) + x_range(1);
            rand(1) * (y_range(2) - y_range(1)) + y_range(1);
        ];
    end
end

function [K, S] = LQR(x)
    A = [ 0 1 ; -9.81*cos(x(1))-0.1 -0.1 ];
    B = [ 0 ; 1 ];
    Q = eye(2);
    R = 1;
    [K,S] = lqr(A, B, Q, R);
end

function [x_nearest, i_nearest] = nearest(x_rand, S, V)
    n = size(V, 2);
    dists = zeros(1, n);
    
    for k = 1:n
        x_k = V(:, k);
        dists(k) = dist(x_rand, x_k, S);
%         dists(k) = rand();
    end
    [min_dists, i_nearest] = min(dists);
    x_nearest = V(:, i_nearest);
end

% Extend the tree from x_0 toward x
% Linearize around x, run that LQR policy for 0.1 seconds
% Return point x_new: where you ended up
function x_new = extend(x_0, x)
    [K, ~] = LQR(x);
    x_bar = x_0 - x;
    u = -K * x_bar;
    u = min(max(u, -5), 5); % enforce torque limits
    [~, x] = ode45(create_odefun(u),[0 0.1], x_0);
    x_new = x(end,:)';
end

% returns the point that is closest (by Euclidean distance) to xy and can
% be reached starting from closest_point applying constant input torque
% between -5 and 5 for 0.1 seconds
function new_vert = extendEuclidean(closest_vert, xy)
    [~,x] = ode45(create_odefun(-5),[0 0.1], closest_vert);
    new_vert = x(end,:)';
    min_dist = euclidean_distance(new_vert, xy);
    for u=-4.5:0.5:5
        [~,x] = ode45(create_odefun(u),[0 0.1], closest_vert);
        vert = x(end,:)';
        d = euclidean_distance(vert, xy);
        if d < min_dist
            new_vert = vert;
            min_dist = d;
        end
    end
end

% x, y are 2x1 vectors. returns the wraparound euclidean distance btwn them
function d = euclidean_distance(a, b)
    x_diff = abs(a(1) - b(1));
    y_diff = abs(a(2) - b(2));
    x_diff = min(x_diff, 2*pi - x_diff);    % handle wraparound
    d = sqrt(x_diff^2 + y_diff^2);
end

function I_near = near(x_new, S_new, i_nearest, V)
    gamma = 2; d = 2; N = size(V, 2);
    dists = zeros(1, size(V, 2));
    for k = 1:size(V, 2)
        x_near = V(:, k);
        dists(k) = dist(x_near, x_new, S_new);
    end
    [~, I_near] = mink(dists, 5);
%     find(dists <= gamma * (log(N) / N)^(1 / d));
%     size(I_near, 2)
%     if size(I_near, 2) == 0
%         I_near = i_nearest;
%     end
end

function [costs, parents] = choose_parent(x_new, I_near, S_new, costs, parents, V)
    m = size(I_near, 2);
    i_new = size(V, 2) + 1;
    
    parent = 0;
    min_path_cost = 99999;
    for k = 1:m
        i_near = I_near(k);
        x_near = V(:, i_near);
        path_cost = costs(i_near) + dist(x_near, x_new, S_new);
        if path_cost < min_path_cost
            min_path_cost = path_cost;
            parent = i_near;
        end
    end
    
    costs(i_new) = min_path_cost;
    parents(i_new) = parent;
end

function [rewired, parents, costs] = rewire(x_new, I_near, costs, parents, V)
    m = size(I_near, 2);
    i_new = size(V, 2) + 1;
    
    rewired = false;
    for k = 1:m 
        i_near = I_near(k);
        x_near = V(:, i_near);
        [~, S_near] = LQR(x_near);
        path_cost = costs(i_new) + dist(x_new, x_near, S_near);
        if path_cost < costs(i_near)
            rewired = true;
            costs(i_near) = path_cost;
            parents(i_near) = i_new;
        end
    end
end

function sample_dot = update_plot(sample_dot, x_rand, x_new, I_near, costs, parents, V, N)
    global th_range,  global th_dot_range, global x_start, global x_goal;
    figure(1);
    clf;
    axis([th_range, th_dot_range]);
    hold on;

    plot(x_start(1), x_start(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 10);
    plot(x_goal(1), x_goal(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
    
%     delete(sample_dot);
%     sample_dot = plot(x_rand(1), x_rand(2), 'r.');
    
%     max_cost = 50;
%     cost = min(1, costs(parents(N)) / max_cost);
%     rgb = hsv2rgb([0 1 1]);
%     vert = V(:, N);
%     plot(vert(1), vert(2), 'o', 'MarkerFaceColor', rgb, 'Color', rgb, 'MarkerSize', 5);
%     parent_i = parents(N);
%     if parent_i
%         parent = V(:, parent_i);
%         if abs(vert(1) - parent(1)) < 0.75*(2*pi)
%             line([vert(1), parent(1)],[vert(2), parent(2)], 'Color', 'b', 'LineWidth', 0.5);
%         end
%     end
    

%     for k = 1:N 
% %        cost = costs(k) / max_cost;
% %        rgb = hsv2rgb([1 1 1]);
%        vert = V(:, k);
%        parent_i = parents(k);
%        if parent_i
%            parent = V(:, parent_i);
%            if abs(vert(1) - parent(1)) < 0.75*(2*pi)
% %                max_cost = 50;
% %                cost = min(1, costs(parents(k)) / max_cost);
% %                if cost < 0
% %                end
%                rgb = hsv2rgb([0 1 1]);
%                line([vert(1), parent(1)],[vert(2), parent(2)], 'Color', rgb, 'LineWidth', 1);
%            end
%        end
% %        plot(vert(1), vert(2), 'o', 'MarkerFaceColor', 'w', 'MarkerSize', 1);
%     end
    
%     for k = 1:size(I_near, 2)
%        near_vert = V(:, I_near(k));
%        plot(near_vert(1), near_vert(2), 'o', 'MarkerFaceColor', 'r', 'MarkerSize', 5); 
%     end
%     for k = 1:N
%         if parents(k) == 0
%             continue
%         end
%         child = V(:, k);
%         parent = V(:, parents(k));
% %         
%         if abs(child(1) - parent(1)) < 0.75*(2*pi)
%             line([child(1), parent(1)],[child(2), parent(2)], 'Color', 'b', 'LineWidth', 0.5);
%         end
%     end
    for k = 1:N
        plot(V(1, k), V(2, k), 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 5);
    end
end

% x, y are 2x1 vectors. S is the matrix which lqr distance is calculated
% with respect to. returns the wraparound lqr distance between a and b
function d = dist(a, b, S)
    nn = true;
    if nn
%         d = euclidean_distance(a, b);
        global net;
        d = sim(net, [a; b]);
        if d < 0
            d = 100;
        end
    else  
        x1 = a - b;
        d1 = x1' * S * x1;
        global distcounter;
        
        % handle wraparound
        
        x_diff = 2*pi - abs(a(1) - b(1));
        y_diff = abs(a(2) - b(2));
        x2 = [x_diff ; y_diff];
        d2 = x2' * S * x2;
        distcounter = distcounter + 1;
        
        d = min(d1, d2);
    end
end

% given a value for control input u, returns a function handle to a
% function of the form f(t,x) = f(x,u) = dxdt
function handle = create_odefun(u)
    handle = @odefun;
    function dxdt = odefun(~,x)
        theta     = x(1);
        theta_dot = x(2);
        dxdt    = zeros(2,1);
        dxdt(1) = theta_dot;
        dxdt(2) = u - (9.81 * sin(theta)) - (0.1 * theta_dot);
    end
end







