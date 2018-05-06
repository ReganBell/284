function [xtraj,utraj,x0,pairs,dists] = rand_dircol(p, traj_opt, N, T, x0, xf, xtraj_init, utraj_init)    
    traj_opt = traj_opt.addRunningCost(@running_cost_fun);
    traj_opt = traj_opt.addStateConstraint(ConstantConstraint(x0),1);
    traj_opt = traj_opt.addStateConstraint(ConstantConstraint(xf),N);

    % initial trajectory guess
    t_init = linspace(0,T,N);
    if isempty(xtraj_init)
        x_init_vec = zeros(2,N);
        u_init_vec = zeros(1,N);
        traj_init.x = PPTrajectory(foh(t_init,x_init_vec));
        traj_init.u = PPTrajectory(foh(t_init,u_init_vec));
        traj_init.x = traj_init.x.setOutputFrame(p.getStateFrame);
        traj_init.u = traj_init.u.setOutputFrame(p.getInputFrame);
    else
        traj_init.x = xtraj_init;
        traj_init.u = utraj_init;
    end
    
    % run DIRCOL
    [xtraj, utraj, ~,~,~] = traj_opt.solveTraj(t_init,traj_init);

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

% unused
function draw_traj(states, p, xtraj)
    % visualize trajectory
    v = p.constructVisualizer;
    v.axis = [-1 1 -1 1];
    v.playback(xtraj);
    
    % draw trajectory
    world_bounds_th = [-pi,pi];
    world_bounds_thdot = [-10,10];
    figure(1); clf; hold on;
    axis([world_bounds_th, world_bounds_thdot]);
    plot(states(1,1),states(2,1),'bo','MarkerFaceColor','b','MarkerSize',5);
    for i=2:31
        prev_pt = states(:,i-1);
        new_pt  = states(:,i);
        plot(new_pt(1),new_pt(2),'bo','MarkerFaceColor','b','MarkerSize',5);
        if abs(new_pt(1) - prev_pt(1)) < 0.75*(2*pi)
            line([prev_pt(1),new_pt(1)],[prev_pt(2),new_pt(2)]);
        end
    end
    axis([world_bounds_th, world_bounds_thdot]);
end

% f = scalar cost
% df = [df/fT df/dx df/du]
function [f,df] = running_cost_fun(dt,x,u)
f = u^2;
df = [0 zeros(1,2) 2*u];
end
