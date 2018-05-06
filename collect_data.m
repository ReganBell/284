output_filename = sprintf('test.mat');

N = 31;      % number of knot points
T = 2;       % max duration allowed
K = 1;       % number of iterations
u_limit = 5; % input limit
theta_bounds = [-pi,pi];    % sampling bounds
thetadot_bounds = [-8,8];   % sampling bounds
[p, traj_opt] = dircol_setup(N, T, u_limit);

failures = 0;
elapsed = zeros(1, K);
for k = 1:K
    tic;
    x0 = [random_sample(theta_bounds); random_sample(thetadot_bounds)];
    xf = [random_sample(theta_bounds); random_sample(thetadot_bounds)];
    [x_traj, u_traj, pairs, dists, success] = dircol(p, traj_opt, N, T, x0, xf);
    elapsed(k) = toc;
    if(success)
        if exist(output_filename, 'file') ~= 2
            X = pairs;
            Y = dists;
            save(output_filename, 'X', 'Y');
        else
            load(output_filename)
            X = [X pairs];
            Y = [Y dists];
            save(output_filename, 'X', 'Y');
        end
    else
        failures = failures + 1;
    end
    
    disp(['Trial: ', num2str(k), '/', num2str(K), ' Total time: ', ...
        num2str(sum(elapsed)),' (', num2str(failures), ' failures', ')']);
end

% bounds = [a,b]
% returns a random decimal within [a,b]
function sample = random_sample(bounds)
    sample = bounds(1) + (bounds(2) - bounds(1))*rand();
end
