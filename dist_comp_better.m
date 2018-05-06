function dist_comp_better
    load('dircol_net_full_akshay_10TL.mat');

    x0 = [pi*rand(); 10*rand()]
    
    xs = zeros(2,11);
    for i=1:11
        u = -6 + i;
        [~,x] = ode45(create_odefun(u),[0 0.1], x0);
        x = x(end,:)';
        xs(:,i) = x;
    end
    
    xf = [pi*rand(); 10*rand()]
    [~, S] = LQR(xf);
    
    N = 31;
    T = 2;
    [p, traj_opt] = dircol_setup(N, T);
    
    dists = zeros(3,11);
    for i=1:11
        i
        x = xs(:,i);
        [~, ~, ~, ~, pairwise_dists] = rand_dircol(p, traj_opt, N, T, x, xf, [], []);
        dists(1,i) = pairwise_dists(29);
        dists(2,i) = lqrdist(x, xf, S);
        dists(3,i) = nndist(x, xf, net);
    end
    
    dists
    
    % TODO: add code that shows selected u for distance metric
end

function [K, S] = LQR(x)
    A = [ 0 1 ; -9.81*cos(x(1))-0.1 -0.1 ];
    B = [ 0 ; 1 ];
    Q = eye(2);
    R = 1;
    [K,S] = lqr(A, B, Q, R);
end

function d = nndist(a, b, net)
%    X = [a; b];
    d = sim(net, [b; a]);
%    d2 = sim(net, [b; a]);
%    if d1 < 0
%        d = d2;
%        if d2 < 0
            
%        end
%    else
%        d = d2;
%    end
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