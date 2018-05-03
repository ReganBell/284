% To use this file:
%   move file into ~/drake/drake/examples/Pendulum
%   open Matlab and run addpath_drake
%   cd into the ~/drake/drake/examples/Pendulum
%   run by calling [p,xtraj,utraj,v,x0] = project;
%   warm-start with previous result by calling project(xtraj,utraj);
function [p,xtraj,utraj,v,x0] = project(xtraj_init, utraj_init)
% create plant
options.replace_cylinders_with_capsules = false;
options.wrap_flag = [true;false]; % necessary?
p = PlanarRigidBodyManipulator('Pendulum.urdf',options);
p = p.setInputLimits(-10,10);

N = 31  % number of knot points
T = 2   % max duration allowed
x0 = [pi*rand(); 10*rand()] % random start point
xf = [pi*rand(); 10*rand()] % random goal point

% initial trajectory guess
t_init = linspace(0,T,N);
if nargin < 2
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

% duration between T/2 and T (will find one of length T in practice)
traj_opt =  DircolTrajectoryOptimization(p,N,[T/2 T]);

traj_opt = traj_opt.addRunningCost(@running_cost_fun);
traj_opt = traj_opt.addStateConstraint(ConstantConstraint(x0),1);
traj_opt = traj_opt.addStateConstraint(ConstantConstraint(xf),N);
traj_opt = traj_opt.setSolver('fmincon');
traj_opt = traj_opt.setSolverOptions('fmincon','Algorithm','sqp');

% NOT NEEDED:
%invertedConstraint = FunctionHandleConstraint(0,0,2,@(x) final_state_con(p,x),1);
%traj_opt = traj_opt.addStateConstraint(invertedConstraint,N);

% run DIRCOL
tic
[xtraj,utraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
toc

% visualize trajectory
v = p.constructVisualizer;
v.axis = [-1 1 -1 1];
v.playback(xtraj)

% extracts states along trajectory
states = zeros(2,N);
for i = 1:N
    states(:,i) = xtraj.eval(xtraj.pp.breaks(i));
end

% draw trajectory
world_bounds_th = [-pi,pi];
world_bounds_thdot = [-10,10];
figure(1); clf; hold on;
axis([world_bounds_th, world_bounds_thdot]);
plot(states(1,1),states(2,1),'bo','MarkerFaceColor','b','MarkerSize',5);
for i=2:N
    prev_pt = states(:,i-1);
    new_pt  = states(:,i);
    plot(new_pt(1),new_pt(2),'bo','MarkerFaceColor','b','MarkerSize',5);
    if abs(new_pt(1) - prev_pt(1)) < 0.75*(2*pi)
        line([prev_pt(1),new_pt(1)],[prev_pt(2),new_pt(2)]);
    end
end
axis([world_bounds_th, world_bounds_thdot]);

% NOT USED: [~,dcon] = final_state_con(p,xf)
end

% f = scalar cost
% df = [df/fT df/dx df/du]
function [f,df] = running_cost_fun(dt,x,u)
f = u^2;
df = [0 zeros(1,2) 2*u];
end

% concerns:
%   is wrapping working correctly? theta values seem to be alowed
%       beyond [-pi, 3pi/2] - handle manually?

% NOT NEEDED:
% x = [theta thetadot] is the state of the system
% f is a scalar whose value is the difference between theta and pi
% so f is zero if the constraint is satisfied
% df is the derivative of f with respect to x
%function [f,df] = final_state_con(obj,x)
%  theta = x(1);
%  f = theta - pi;
%  df = [1 0];
%end
