function dist_comp_better
    load('dircol_net_full_akshay_10TL.mat');
    
    goal = [pi; 0];
    [~, S] = LQR(goal);
    
    % draw world and goal
    world_bounds_th = [-pi/2,3/2*pi];
    world_bounds_thdot = [-5,5];
    figure(1); clf; hold on;
    axis([world_bounds_th, world_bounds_thdot]);
   
    % discretize state space and get costs
    n_theta = 10*4;
    theta_range = linspace(world_bounds_th(1), world_bounds_th(2), n_theta);
    n_thetadot = 10*4;
    thetadot_range = linspace(world_bounds_thdot(1), world_bounds_thdot(2), n_thetadot);
    
    xs = zeros(2,n_theta*n_thetadot);
    costs = zeros(1,n_theta*n_thetadot);
    k = 0;
    for i = 1:n_theta
        for j = 1:n_thetadot
            k = k+1
            theta = theta_range(i);
            thetadot = thetadot_range(j);
            x = [theta; thetadot];
            %cost = lqrdist(x, goal, S);
            cost = nndist(x, goal, net);
            xs(:,k) = x;
            costs(k) = cost;
        end
    end
    
    % plot points
    max_cost = max(costs);
    for i=1:(n_theta*n_thetadot)
        x = xs(:,i);
        cost = costs(i);
        
        hue = min(1, cost / max_cost);
        rgb = hsv2rgb([hue 1 1]);
        plot(x(1), x(2), 'o', 'MarkerFaceColor', rgb, 'Color', rgb, 'MarkerSize', 10);
    end
    
    xs
    costs
end

function [K, S] = LQR(x)
    A = [ 0 1 ; -9.81*cos(x(1))-0.1 -0.1 ];
    B = [ 0 ; 1 ];
    Q = eye(2);
    R = 1;
    [K,S] = lqr(A, B, Q, R);
end

function d = nndist(a, b, net)
    d = sim(net, [b; a]);
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

function d = lqrdist(a, b, S)
    a = reshape(a, [2, 1]);
    b = reshape(b, [2, 1]);
    x1 = a - b;
    d1 = x1' * S * x1;
    
    % handle wraparound
    x_diff = 2*pi - abs(a(1) - b(1));
    y_diff = abs(a(2) - b(2));
    x2 = [x_diff ; y_diff];
    d2 = x2' * S * x2;
    
    d = min(d1, d2);
end