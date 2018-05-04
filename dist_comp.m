function dist_comp

    load('dircol_net.mat');

    th_bins = -pi/2: 0.25 : 3/2 * pi;
    th_dot_bins = -10 : 0.5 : 10;
    
    K = 2;
    
    x0s = zeros(2, K);
    xfs = zeros(2, K);
    
    lqr_dists = zeros(1, K);
    nn_dists = zeros(1, K);
    dircol_dists = zeros(1, K);
    all_dists = zeros(3, K);
    se = zeros(2, K);
    
    N = 31;  % number of knot points
    T = 2;   % max duration allowed
    [p, traj_opt] = dircol_setup(N, T);
    
    x_traj = [];
    u_traj = [];
    for k = 1:K
        x0 = [pi*rand(); 10*rand()];
        xf = [pi*rand(); 10*rand()];
        x0s(:, k) = x0;
        xfs(:, k) = xf;
       
        [x_traj, u_traj, ~, ~, dists] = rand_dircol(p, traj_opt, N, T, x0, xf, x_traj, u_traj);
        all_dists(1, k) = dists(N - 1);
        [~, S] = LQR(xf);
        all_dists(2, k) = lqrdist(x0, xf, S);
        all_dists(3, k) = nndist(x0, xf, net);
        se(1, k) = (all_dists(1, k) - all_dists(2, k)) ^ 2;
        se(2, k) = (all_dists(1, k) - all_dists(3, k)) ^ 2;
    end
    
    all_dists
    lqr_mse = mean(se(1, :))
    nn_mse = mean(se(2, :))
    max_se = max(log([se(1, :) se(2, :)]));
    
    figure(1);
    th_range = [-pi/2, (3/2) * pi];
    th_dot_range = [-10, 10];
    hold on;
    for k = 1:K
        
        midpoint = (x0s(:, k) - xfs(:, k)) / 2 + x0s(:, k);
        
        subplot(2, 1, 1);
        axis([th_range, th_dot_range]);
        colorbar
        error = min(1, log(se(1, k) / max_se));
        rgb = hsv2rgb([error 1 1]);
       
        plot(midpoint(1), midpoint(2), 'o', 'MarkerFaceColor', rgb, 'Color', rgb, 'MarkerSize', 5);

        subplot(2, 1, 2);
        axis([th_range, th_dot_range]);
        error = min(1, log(se(2, k) / max_se));
        rgb = hsv2rgb([error 1 1]);
        plot(midpoint(1), midpoint(2), 'o', 'MarkerFaceColor', rgb, 'Color', rgb, 'MarkerSize', 5);
    end
end

function [K, S] = LQR(x)
    A = [ 0 1 ; -9.81*cos(x(1))-0.1 -0.1 ];
    B = [ 0 ; 1 ];
    Q = eye(2);
    R = 1;
    [K,S] = lqr(A, B, Q, R);
end

function d = nndist(a, b, net)
    X = [a; b];
    d1 = sim(net, [a; b]);
    d2 = sim(net, [b; a]);
    if d1 < 0
        d = d2;
        if d2 < 0
            
        end
    else
        d = d2;
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