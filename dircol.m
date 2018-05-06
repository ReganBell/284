function [xtraj,utraj,pairs,dists,success] = dircol(p, traj_opt, N, T, x0, xf) 
    % define optimization problem
    traj_opt = traj_opt.addRunningCost(@running_cost_fun);
    traj_opt = traj_opt.addStateConstraint(ConstantConstraint(x0),1);
    traj_opt = traj_opt.addStateConstraint(ConstantConstraint(xf),N);

    % initial trajectory guess
    t_init = linspace(0,T,N);
    x_init_vec = zeros(2,N);
    u_init_vec = zeros(1,N);
    traj_init.x = PPTrajectory(foh(t_init,x_init_vec));
    traj_init.u = PPTrajectory(foh(t_init,u_init_vec));
    traj_init.x = traj_init.x.setOutputFrame(p.getStateFrame);
    traj_init.u = traj_init.u.setOutputFrame(p.getInputFrame);
    
    % run DIRCOL
    [xtraj, utraj, ~, F, info] = traj_opt.solveTraj(t_init,traj_init);
    
    % make sure it worked
    if info ~= 1
        xtraj = [];
        utraj = [];
        pairs = [];
        dists = [];
        success = false;
        return;
    else
        success = true;
    end

    % extracts states along trajectory
    states = zeros(2,N);
    inputs = zeros(1,N);
    for i = 1:N
        states(:,i) = xtraj.eval(xtraj.pp.breaks(i));
        inputs(i) = utraj.eval(utraj.pp.breaks(i));
    end

    costs = (inputs .^ 2) * (T/(N-1));
    pairs = zeros(4, 465);
    dists = zeros(1, 465);
    m = 1;
    for i = 1:N
        for k = i+1:N
            pairs(:, m) = [states(:, i); states(:, k)];
            dists(m) = sum(costs(i:k-1));
            m = m + 1;
        end
    end
end

% f = scalar cost
% df = [df/fT df/dx df/du]
function [f,df] = running_cost_fun(~,~,u)
f = u^2;
df = [0 zeros(1,2) 2*u];
end
