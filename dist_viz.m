function dist_viz

    load('dircol_net.mat');

    th_bins = -pi/2: 0.25 : 3/2 * pi;
    th_dot_bins = -10 : 0.5 : 10;
    
    as = zeros(size(th_bins, 2), size(th_dot_bins, 2), 2);
    bs = zeros(size(th_bins, 2), size(th_dot_bins, 2), 2);
    
    lqr_dists = zeros(size(th_bins, 2), size(th_dot_bins, 2));
    nn_dists = zeros(size(th_bins, 2), size(th_dot_bins, 2));
    
    for x = 1:size(as, 1)
        for y = 1:size(as, 2)
            as(x, y, :) = [th_bins(x); th_dot_bins(y)];
            bs(x, y, :) = [th_bins(x)  + 0.5; th_dot_bins(y) + 1];
        end
    end
    tic
    for x = 1:size(as, 1)
        for y = 1:size(as, 2)
            a = reshape(as(x, y, :), [2, 1]);
            b = reshape(bs(x, y, :), [2, 1]);
            
            [~, S] = LQR(a);
            lqr_dists(x, y) = lqrdist(a, b, S);
        end
    end
    lqrtime = toc
    tic
    for x = 1:size(as, 1)
        for y = 1:size(as, 2)
            a = reshape(as(x, y, :), [2, 1]);
            b = reshape(bs(x, y, :), [2, 1]);
            
            nn_dists(x, y) = nndist(a, b, net);
        end
    end
    nntime = toc
                

    
    figure(1);
    
    subplot(2, 1, 1);
    imagesc(lqr_dists)
    colorbar
    
    subplot(2, 1, 2);
    imagesc(nn_dists)
    colorbar
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
    d = sim(net, X);
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