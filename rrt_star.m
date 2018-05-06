
global th_range;
global th_dot_range;
global x_start;
global x_goal;
global nn;
global u_limit;

th_range = [-pi/2, (3/2) * pi];
th_dot_range = [-10, 10];

x_start = [0; 0];
x_goal = [pi; 0];

nn = true;

u_limit = -5:1:5;

V_all = [x_start zeros(2, 9999)];
V = [x_start];
N = 1;
parents = zeros(1, 10000);
costs = zeros(1, 10000);
goal_reached = false;
goal_reached_dist = 0.25;
sample_dot = false;

global net;
load('net_613_20.mat');

figure(1);
rewired = zeros(1, 10000);
s_dot = line([0 0], [0 0]);
tic;
while true
    
    % Sample a random point
    x_rand = sample(0.05, x_goal, th_range, th_dot_range);
    
    % Collect nodes in tree nearest to sample
    I_near = near(x_rand, V);
   
    % Extend tree toward sample
    x_nearest = V(:, I_near(1));
    [x_new, cost] = extend(x_nearest, x_rand);
    costs(N+1) = costs(I_near(1)) + cost;
    parents(N+1) = I_near(1);
   
    % Redraw paths from start to near nodes thru new node, if total cost is less
    [rewired, parents, costs] = rewire(x_new, I_near, costs, parents, rewired, V);
    
    V_all(:, N + 1) = x_new;
    N = N + 1;
    V = V_all(:, 1:N);
    
    if mod(N, 5) == 0
        s_dot = draw(costs, parents, N, V, rewired, s_dot, x_rand);
        nodes = N
        toc
    end
    
    if norm(x_goal - x_new) < goal_reached_dist
        nodes = N
        path_cost = costs(N)
        draw(costs, parents, N, V, rewired, s_dot, x_rand);
        toc
        break;
    end
end

function s_dot = draw(costs, parents, N, V, rewired, s_dot, x_rand)
    global th_range; global th_dot_range; global x_start; global x_goal;
    figure(1);
    clf;
    axis([th_range, th_dot_range]);
    hold on;
    max_cost = max(costs);
    for k = 1:N
        parent = parents(k);
        if parent == 0
            continue
        end
        x_new = V(:, k);
        x_parent = V(:, parent);
        old_parent = rewired(k);
        line_color = 'b';
        if old_parent
            x_old = V(:, old_parent);
            line([x_old(1), x_new(1)], [x_old(2), x_new(2)], 'Color', 'r');
            line_color = 'g';
        end
        line([x_new(1), x_parent(1)], [x_new(2), x_parent(2)], 'Color', line_color);
        hue = 0.4*(1 - costs(k)/max_cost);
        rgb = hsv2rgb([hue 1 1]);
        plot(V(1, k), V(2, k), 'o', 'MarkerFaceColor', rgb, 'MarkerSize', 5, 'Color', rgb);
    end
    plot(x_start(1), x_start(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
    plot(x_goal(1), x_goal(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
    delete(s_dot);
    s_dot = plot(x_rand(1), x_rand(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 2);
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

% returns the point that is closest (by Euclidean distance) to xy and can
% be reached starting from closest_point applying constant input torque
% between -5 and 5 for 0.1 seconds
function [x_new, cost] = extend(x_nearest, x_rand)
    global u_limit;
    x_new = [];
    min_dist = 9999;
    cost = 0;
    for u=u_limit
        [~,x] = ode45(create_odefun(u),[0 0.1], x_nearest);
        vert = x(end,:)';
        d = dist(vert, x_rand);
        if d < min_dist
            x_new = vert;
            min_dist = d;
            cost = u ^ 2;
        end
    end
end

function I_near = near(x_new, V)
    dists = zeros(1, size(V, 2));
    for k = 1:size(V, 2)
        x_near = V(:, k);
        dists(k) = dist(x_near, x_new);
    end
    [~, I_near] = mink(dists, 5);
end

function [rewired, parents, costs] = rewire(x_new, I_near, costs, parents, rewired, V)
    m = size(I_near, 2);
    i_new = size(V, 2) + 1;
    
    for k = 1:m 
        i_near = I_near(k);
        x_near = V(:, i_near);
        [x_reached, cost] = extend(x_new, x_near);
        d = dist(x_reached, x_near);
        if d > 0.5
            continue
        end
        path_cost = costs(i_new) + cost;
        if path_cost < costs(i_near)
            rewired(i_near) = parents(i_near);
            costs(i_near) = path_cost;
            parents(i_near) = i_new;
        end
    end
end

function d = euclidean_distance(a, b)
    x_diff = abs(a(1) - b(1));
    y_diff = abs(a(2) - b(2));
    x_diff = min(x_diff, 2*pi - x_diff);    % handle wraparound
    d = sqrt(x_diff^2 + y_diff^2);
end

function [K, S] = LQR(x)
    A = [ 0 1 ; -9.81*cos(x(1))-0.1 -0.1 ];
    B = [ 0 ; 1 ];
    Q = eye(2);
    R = 1;
    [K,S] = lqr(A, B, Q, R);
end

% x, y are 2x1 vectors. S is the matrix which lqr distance is calculated
% with respect to. returns the wraparound lqr distance between a and b
function d = dist(a, b)
    d = euclidean_distance(a, b);
    return;
    global nn; global net;
    if nn
        d = sim(net, [a; b]);
        if d < 0
            d = 100;
        end
    else
        x1 = a - b;
        d1 = x1' * S * x1;
        
        % handle wraparound
        x_diff = 2*pi - abs(a(1) - b(1));
        y_diff = abs(a(2) - b(2));
        x2 = [x_diff ; y_diff];
        d2 = x2' * S * x2;
        
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







