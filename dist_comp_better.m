function dist_comp_better
    load('net808_30_20.mat');
    
    theta_bounds = [0,pi];    % sampling bounds
    thetadot_bounds = [0,8];   % sampling bounds

    K = 11
    
    N = 31;
    T = 2;
    [p, traj_opt] = dircol_setup(N, T, 8);
    
    dists = zeros(4,11);
    se = zeros(3, 11);
    i = 0;
    x0 = [random_sample(theta_bounds); random_sample(thetadot_bounds)];
    while i <= K
        i
        xf = [random_sample(theta_bounds); random_sample(thetadot_bounds)];

        [~, ~, ~, dircol_dists, success] = dircol(p, traj_opt, N, T, x0, xf);
        if ~success
            continue
        else
            i = i+1;
        end
        dists(1,i) = dircol_dists(30);
        
        [~,S] = LQR(xf);
        dists(2,i) = lqrdist(x0, xf, S);
        dists(3,i) = nndist(x0, xf, net);
        dists(4,i) = euclidean_distance(x0, xf);
    end
    
    [~, I_dircol] = mink(dists(1, :), K)
    [~, I_lqr] = mink(dists(2, :), K)
    [~, I_nn] = mink(dists(3, :), K)
    [~, I_euclid] = mink(dists(4, :), K)
    I_rand = [1     5     6     3    12     8     2     9     4    10    11]
    
    tau_lqr = kendalltau(I_dircol, I_lqr)
    tau_nn = kendalltau(I_dircol, I_nn)
    tau_euclid = kendalltau(I_dircol, I_euclid)
    tau_rand = kendalltau(I_dircol, I_rand)
    
    [~, I_dircol] = mink(dists(1, :), 5)
    [~, I_lqr] = mink(dists(2, :), 5)
    [~, I_nn] = mink(dists(3, :), 5)
    [~, I_euclid] = mink(dists(4, :), 5)
    I_rand = [1     5     6     3    12]
    
    tau_lqr = kendalltau(I_dircol, I_lqr)
    tau_nn = kendalltau(I_dircol, I_nn)
    tau_euclid = kendalltau(I_dircol, I_euclid)
    tau_rand = kendalltau(I_dircol, I_rand);
end

function [K, S] = LQR(x)
    A = [ 0 1 ; -9.81*cos(x(1))-0.1 -0.1 ];
    B = [ 0 ; 1 ];
    Q = eye(2);
    R = 1;
    [K,S] = lqr(A, B, Q, R);
end

function tau = kendalltau( order1 , order2 )
    [ ~ , ranking1 ] = sort( order1(:)' , 2 , 'ascend' );
    [ ~ , ranking2 ] = sort( order2(:)' , 2 , 'ascend' );
    N = length( ranking1 );
    [ ii , jj ] = meshgrid( 1:N , 1:N );
    ok = find( jj(:) > ii(:) );
    ii = ii( ok );
    jj = jj( ok );
    nok = length( ok );
    sign1 = ranking1( jj ) > ranking1( ii );
    sign2 = ranking2( jj ) > ranking2( ii );
    tau = sum( sign1 ~= sign2 );
end

function d = euclidean_distance(a, b)
    x_diff = abs(a(1) - b(1));
    y_diff = abs(a(2) - b(2));
    x_diff = min(x_diff, 2*pi - x_diff);    % handle wraparound
    d = sqrt(x_diff^2 + y_diff^2);
end

function d = nndist(a, b, net)
    d = sim(net, [a; b]);
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

function sample = random_sample(bounds)
    sample = bounds(1) + (bounds(2) - bounds(1))*rand();
end