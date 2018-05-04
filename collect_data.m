N = 31;  % number of knot points
T = 2;   % max duration allowed
K = 100; % number of iterations
[p, traj_opt] = dircol_setup(N, T);

x_traj = [];
u_traj = [];
elapsed = zeros(1, K);
for k = 1:100
    tic;
    x0 = [pi*rand(); 10*rand()]; % random start point
    xf = [pi*rand(); 10*rand()]; % random goal point
    [x_traj, u_traj, x0, pairs, dists] = rand_dircol(p, traj_opt, N, T, x0, xf, x_traj, u_traj);
    if exist('dircol_data.mat', 'file') ~= 2
        X = pairs;
        Y = dists;
        save('dircol_data.mat', 'X', 'Y');
    end
    load('dircol_data.mat')
    X = [X pairs];
    Y = [Y dists];
    elapsed(k) = toc;
    save('dircol_data.mat', 'X', 'Y', 'elapsed');
    disp(['Trial: ', num2str(k), '/', num2str(K), ' Total time: ', num2str(sum(elapsed))]);
end

% [p,xtraj,utraj,v,x0] = project;
%   warm-start with previous result by calling project(xtraj,utraj);