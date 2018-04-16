% Bounds on world
world_bounds_th = [-pi/2,(3/2)*pi];
world_bounds_thdot = [-10,10];

% Start and goal positions
figure(1); clf;
xy_start = [0;0]; plot(xy_start(1),xy_start(2),'bo','MarkerFaceColor','b','MarkerSize',10);
xy_goal = [pi;0]; plot(xy_goal(1),xy_goal(2),'go','MarkerFaceColor','g','MarkerSize',10); drawnow;

% Initialize RRT. The RRT will be represented as a 2 x N list of points. So
% each column represents a vertex of the tree.
rrt_verts = zeros(2,1000);
prev_verts = zeros(1,1000);
rrt_verts(:,1) = xy_start;
N = 1;
nearGoal = false; % This will be set to true if goal has been reached
minDistGoal = 0.25; % This is the convergence criterion. We will declare
                    % success when the tree reaches within 0.25 in distance
                    % from the goal. DO NOT MODIFY.


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% FILL ME IN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Choose one of these methods
% method = 'euclidean'; % Euclidean distance metric (part b of problem)
method = 'lqr'; % LQR distance metric (part d of problem)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


figure(1); hold on;
axis([world_bounds_th, world_bounds_thdot]);
 hxy = plot(0,0,'ro');
% RRT algorithm
while ~nearGoal
    % Sample point
    rnd = rand(1);
    % With probability 0.05, sample the goal. This promotes movement to the goal.
    if rnd < 0.05
        xy = xy_goal;
    else
        % Sample from space with probability 0.95
        xs = (world_bounds_th(2) - world_bounds_th(1))*rand(1) + world_bounds_th(1);
        ys = (world_bounds_thdot(2) - world_bounds_thdot(1))*rand(1) + world_bounds_thdot(1);
        xy = [xs;ys];
    end
    
    %% FILL ME IN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if strcmp(method, 'euclidean')
        closest_vert = closestVertexEuclidean(rrt_verts(:,1:N),xy); % Write this function
    elseif strcmp(method, 'lqr')
        [closest_vert,K] = closestVertexLQR(rrt_verts(:,1:N),xy); % Write this function
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% FILL ME IN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if strcmp(method, 'euclidean')closest_vert
        new_vert = extendEuclidean(closest_vert,xy); % Write this function
    else
        new_vert = extendLQR(closest_vert,xy,K); % Write this function
    end
        
    delete(hxy);
    figure(1);
    hxy = plot(xy(1),xy(2),'r.');axis([world_bounds_th, world_bounds_thdot]); 

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
     
    % Plot extension (Comment the next few lines out if you want your code to
    % run a bit quicker. The plotting is useful for debugging though.)
    figure(1)
    hold on
    plot(new_vert(1),new_vert(2),'bo','MarkerFaceColor','b','MarkerSize',5);
    % Plot line (but only if we are not wrapping to the other side of the
    % plot)
    if abs(closest_vert(1) - new_vert(1)) < 0.75*(2*pi)
        line([closest_vert(1),new_vert(1)],[closest_vert(2),new_vert(2)]);
    end
    axis([world_bounds_th, world_bounds_thdot]);

    
    %% DO NOT MODIFY THIS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % If it is collision free, add it to tree    
    N = N+1;
    if N > size(rrt_verts,2)
        rrt_verts = [rrt_verts zeros(size(rrt_verts))];
        prev_verts = [prev_verts zeros(size(prev_verts))];
    end
    rrt_verts(:,N) = new_vert;
    prev_verts(N) = get_index(closest_vert, rrt_verts);
    
    % Check if we have reached goal
    if norm(xy_goal-new_vert) < minDistGoal
        break;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

% Plot vertices in RRT
hold on;
plot(rrt_verts(1,:),rrt_verts(2,:),'bo','MarkerFaceColor','b','MarkerSize',5);

% extract the path
xpath = zeros(2,N);
vert = rrt_verts(:,N);
prev_vert_idx = prev_verts(N);
xpath(:,1) = vert;
i = 1;
while ~isequal(vert, xy_start)
    i = i+1;
    vert_idx = prev_vert_idx;
    vert = rrt_verts(:,vert_idx);
    xpath(:,i) = vert;
    prev_vert_idx = prev_verts(prev_vert_idx);
end
xpath = fliplr(xpath(:,1:i))'

% returns the index of the given 2x1 column c in the given 2xn matrix m
function idx = get_index(c, m)
    for i=1:size(m,2)
        if isequal(m(:,i), c)
            idx = i;
            return;
        end  
    end
end

% x, y are 2x1 vectors. S is the matrix which lqr distance is calculated
% with respect to. returns the wraparound lqr distance between a and b
function dist = lqr_dist(a, b, S)
    x1 = a-b;
    d1 = x1' * S * x1;
    
    % handle wraparound
    x_diff = 2*pi - abs(a(1) - b(1));
    y_diff = abs(a(2) - b(2));
    x2 = [x_diff ; y_diff];
    d2 = x2' * S * x2;
    
    dist = min(d1, d2);
end

% rrt_verts is a 2xN vector consisting of the current vertices of the RRT
% returns the closest RRT vertex to xy by the LQR metric
function [closest,K] = closestVertexLQR(rrt_verts, xy)
    A = [ 0 1 ; -9.81*cos(xy(1))-0.1 -0.1 ];
    B = [ 0 ; 1 ];
    Q = eye(2);
    R = 0.1;
    [K,S] = lqr(A,B,Q,R);

    closest = rrt_verts(:,1);
    dist = lqr_dist(closest, xy, S);
    for i=2:size(rrt_verts, 2)
        vert = rrt_verts(:,i);
        new_dist = lqr_dist(vert, xy, S);
        if new_dist < dist
            closest = vert;
            dist = new_dist;
        end
    end
end

% returns the point that results from applying the LQR policy (with respect
% to input K) for 0.1 seconds. Inputs are still restricted to [-5,5]
function new_vert = extendLQR(closest_vert, xy, K)
    x_bar = closest_vert - xy;
    u = -K * x_bar;
    u = min(max(u, -5), 5); % enfore torque limits
    [~,x] = ode45(create_odefun(u),[0 0.1], closest_vert);
    new_vert = x(end,:)';
end

% x, y are 2x1 vectors. returns the wraparound euclidean distance btwn them
function dist = euclidean_distance(a, b)
    x_diff = abs(a(1) - b(1));
    y_diff = abs(a(2) - b(2));
    x_diff = min(x_diff, 2*pi - x_diff);    % handle wraparound
    dist = sqrt(x_diff^2 + y_diff^2);
end

% rrt_verts is a 2xN vector consisting of the current vertices of the RRT
% returns the closest RRT vertex to xy by the Euclidean metric
function closest = closestVertexEuclidean(rrt_verts, xy)
    closest = rrt_verts(:,1);
    dist = euclidean_distance(closest, xy);
    for i=2:size(rrt_verts, 2)
        vert = rrt_verts(:,i);
        new_dist = euclidean_distance(vert, xy);
        if new_dist < dist
            closest = vert;
            dist = new_dist;
        end
    end
end

% returns the point that is closest (by Euclidean distance) to xy and can
% be reached starting from closest_point applying constant input torque
% between -5 and 5 for 0.1 seconds
function new_vert = extendEuclidean(closest_vert, xy)
    [~,x] = ode45(create_odefun(-5),[0 0.1], closest_vert);
    new_vert = x(end,:)';
    dist = euclidean_distance(new_vert, xy);
    for u=-4.5:0.5:5
        [~,x] = ode45(create_odefun(u),[0 0.1], closest_vert);
        vert = x(end,:)';
        dist1 = euclidean_distance(vert, xy);
        if dist1 < dist
            new_vert = vert;
            dist = dist1;
        end
    end
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
