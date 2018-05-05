quadrant = 1;
output_filename = sprintf('quadrant_%d.mat',quadrant);
N = 31;         % number of knot points
T = 2;          % max duration allowed
K = 250;        % number of iterations
u_limit = 5;    % input limit
[p, traj_opt] = dircol_setup(N, T, u);

if i == 1
    theta_bounds = [0,pi];
    thetadot_bounds = [0,8];
elseif i == 2
    theta_bounds = [-pi,0];
    thetadot_bounds = [0,8];
elseif i == 3
    theta_bounds = [-pi,0];
    thetadot_bounds = [-8,0];
elseif i == 4
    theta_bounds = [0,pi];
    thetadot_bounds = [-8,0];
end

x_traj = [];
u_traj = [];
elapsed = zeros(1, K);
for k = 1:K
    tic;
    x0 = [random_sample(theta_bounds); random_sample(thetadot_bounds)];
    xf = [random_sample(theta_bounds); random_sample(thetadot_bounds)];
    [x_traj, u_traj, x0, pairs, dists] = rand_dircol(p, traj_opt, N, T, x0, xf, x_traj, u_traj);
    if exist(output_filename, 'file') ~= 2
        X = pairs;
        Y = dists;
        save(output_filename, 'X', 'Y');
    end
    load(output_filename)
    X = [X pairs];
    Y = [Y dists];
    elapsed(k) = toc;
    save(output_filename, 'X', 'Y', 'elapsed');
    disp(['Trial: ', num2str(k), '/', num2str(K), ' Total time: ', num2str(sum(elapsed))]);
end

% bounds = [a,b]
% returns a random decimal within [a,b]
function sample = random_sample(bounds)
    sample = bounds(1) + random(bounds(2) - bounds(1));
end
