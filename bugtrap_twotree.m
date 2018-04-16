% Obstacles
w = 0.5;
Obs{1} = [0 0;5 0;5 w;0 w]';
Obs{2} = [0 0;2*w 0;w 10;0 10]';
Obs{3} = [0 10-w;5 10;5 10+w;0 10+w]';
Obs{4} = [5-w 0;5+w 0;5+w 5;5 5]';
Obs{5} = [5-w 10+w;5+w 10+w;5+w 6.25;5 6.25]';
Obs{6} = [4 5;5+w 5;5+w 5+w;4 5+w]';
Obs{7} = [4 6.25;5+w 6.25;5+w 6.25+w;4 6.25+w]';

% Bounds on world
world_bounds_x = [-8,10];
world_bounds_y = [-4,14];

% Draw obstacles
figure(1); clf; hold on;
axis([world_bounds_x world_bounds_y]);

for k = 1:length(Obs)
    patch(Obs{k}(1,:),Obs{k}(2,:),'r');
end

% Start and goal positions
xy_start = [4;1]; plot(xy_start(1),xy_start(2),'bo','MarkerFaceColor','b','MarkerSize',10);
xy_goal = [-4;6]; plot(xy_goal(1),xy_goal(2),'go','MarkerFaceColor','g','MarkerSize',10);

% Initialize RRT trees
tree1_verts = zeros(2,1000);
tree1_verts(:,1) = xy_start;
goal1 = xy_goal;
goal_dist1 = norm(xy_goal-xy_start);
N1 = 1;
color1 = ["go" "g"];

tree2_verts = zeros(2,1000);
tree2_verts(:,1) = xy_goal;
goal2 = xy_start;
goal_dist2 = norm(xy_goal-xy_start);
N2 = 1;
color2 = ["bo" "b"];

connected = false; % This will be set to true if two trees connect
minConnectionDist = 0.25;

% Extension parameter
d = 0.5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% RRT algorithm
while ~connected
   % swap trees
   temp_verts = tree2_verts;
   temp_N = N2;
   temp_goal = goal2;
   temp_goaldist = goal_dist2;
   temp_color = color2;
   
   tree2_verts = tree1_verts;
   N2 = N1;
   goal2 = goal1;
   goal_dist2 = goal_dist1;
   color2 = color1;
   
   tree1_verts = temp_verts;
   N1 = temp_N;
   goal1 = temp_goal;
   goal_dist1 = temp_goaldist;
   color1 = temp_color;
    
    % Sample point
    rnd = rand(1);
    % With probability 0.05, sample the goal
    if rnd < 0.05
        xy = goal1;
    else
        xy = [world_bounds_x(1) + (world_bounds_x(2) - world_bounds_x(1)) * rand(1) ;
              world_bounds_y(1) + (world_bounds_y(2) - world_bounds_y(1)) * rand(1)];
    end
    
    collFree = isCollisionFree(Obs,xy);
    
    % If it's not collision free, continue with loop
    if ~collFree
        continue;
    end
    
    % If it is collision free, find closest point in existing tree. 
    [closest_vert, dist] = closestVertex(tree1_verts(:,1:N1),xy); % Write this function
        
    % Extend tree towards xy from closest_vert
    deltax = (xy(1) - closest_vert(1)) * (0.5 / dist);
    deltay = (xy(2) - closest_vert(2)) * (0.5 / dist);
    delta = [deltax ; deltay];
    new_vert = closest_vert + delta;
    
    % Check if new_vert is collision free
    collFree = isCollisionFree(Obs,new_vert);
    
    % If it is not collision free, continue with loop
     if ~collFree
        continue;
     end
     
    % If it is collision free, add it to tree    
    N1 = N1+1;
    if N1 > size(tree1_verts,2)
        tree1_verts = [tree1_verts zeros(size(tree1_verts))];
    end
    tree1_verts(:,N1) = new_vert;
    
    % Plot extension
    figure(1)
    plot(new_vert(1),new_vert(2), color1(1),'MarkerFaceColor',color1(2),'MarkerSize',5);
    line([closest_vert(1),new_vert(1)],[closest_vert(2),new_vert(2)]);
    
    % extend other tree
    [closest_vert, dist] = closestVertex(tree2_verts(:,1:N2),new_vert); % Write this function
    
    % if closest vertex in tree2 to the new vertex in tree1 is closer than
    % tree1's original goal, then update tree1's goal
    if dist < goal_dist1
        % erase old goal
        figure(1)
        plot(goal1(1), goal1(2),'ro','MarkerFaceColor','r','MarkerSize',5);
        
        % update goal
        goal1 = closest_vert;
        goal_dist1 = dist;
        
        % Plot new goal
        figure(1)
        plot(closest_vert(1), closest_vert(2),'yo','MarkerFaceColor','y','MarkerSize',5);
    end
    
    % similarly, if the new vertex in tree1 is closer to the closest vertex
    % in tree2 than tree2's original goal, then update tree2's goal
    if dist < goal_dist2
        % erase old goal
        figure(1)
        plot(goal2(1), goal2(2),'ro','MarkerFaceColor','r','MarkerSize',5);
        
        goal2 = new_vert;
        goal_dist2 = dist;
        
        % Plot extension (Comment the next 3 lines out if you want your code to
        % run a bit quicker. The plotting is useful for debugging though.)
        figure(1)
        plot(new_vert(1), new_vert(2),'yo','MarkerFaceColor','y','MarkerSize',5);
    end
        
    % Extend tree towards new_vert from closest_vert
    deltax = (new_vert(1) - closest_vert(1)) * (0.5 / dist);
    deltay = (new_vert(2) - closest_vert(2)) * (0.5 / dist);
    delta = [deltax ; deltay];
    new_vert2 = closest_vert + delta;
    
    % Check if new_vert2 is collision free
    collFree = isCollisionFree(Obs,new_vert2); % Same function you wrote before.
    
    % If it is not collision free, continue with loop
    if ~collFree
        continue;
    end
   
    % If it is collision free, add it to tree    
    N2 = N2+1;
    if N2 > size(tree2_verts,2)
        tree2_verts = [tree2_verts zeros(size(tree2_verts))];
    end
    tree2_verts(:,N2) = new_vert2;
    
    % Plot extension
    figure(1)
    plot(new_vert2(1),new_vert2(2),color2(1),'MarkerFaceColor',color2(2),'MarkerSize',5);
    line([closest_vert(1),new_vert2(1)],[closest_vert(2),new_vert2(2)]);
    
    % find closest vertex in tree1 to new_vert2
    [closest_vert, dist] = closestVertex(tree1_verts(:,1:N1),new_vert2);
    
    % if closest vertex in tree1 to the new vertex in tree2 is closer than
    % tree2's original goal, then update tree2's goal
    if dist < goal_dist2
        % erase old goal
        figure(1)
        plot(goal2(1), goal2(2),'ro','MarkerFaceColor','r','MarkerSize',5);
        
        goal2 = closest_vert;
        goal_dist2 = dist;
        
        % Plot new goal
        figure(1)
        plot(closest_vert(1), closest_vert(2),'yo','MarkerFaceColor','y','MarkerSize',5);
    end
    
    % similarly, if the new vertex in tree2 is closer to the closest vertex
    % in tree1 than tree1's original goal, then update tree1's goal
    if dist < goal_dist1
        % erase old goal
        figure(1)
        plot(goal1(1), goal1(2),'ro','MarkerFaceColor','r','MarkerSize',5);
        
        goal1 = new_vert2;
        goal_dist1 = dist;
        
        % Plot extension
        figure(1)
        plot(new_vert2(1), new_vert2(2),'yo','MarkerFaceColor','y','MarkerSize',5);
    end
    
    % Check if we have reached goal
    if norm(new_vert-new_vert2) < minConnectionDist
        break;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

% Plot vertices in RRT
plot(tree1_verts(1,:),tree1_verts(2,:),'bo','MarkerFaceColor','b','MarkerSize',5);

% rrt_verts is 2 x N list of points (each column is a vertex in the tree)
% closest is set to the closest vertex in rrt_verts to xy (using Euclidean
% distance)
function [closest_pt, closest_dist] = closestVertex(rrt_verts,xy)
    closest_pt = rrt_verts(:,1);
    closest_dist = norm(closest_pt - xy);
    for i=2:size(rrt_verts,2)
        vert = rrt_verts(:,i);
        dist = norm(vert - xy);
        if dist < closest_dist
            closest_pt = vert;
            closest_dist = dist;
        end
    end
end

% Obs is a cell array of obstacles, xy is a 2x1 vector to check
% collFree is set to true if xy does not collide with any cells in Obs
function collFree = isCollisionFree(obs,xy)
    for i=1:length(obs)
        ob = obs{i};
        collisions = check_collisions(xy, ob);
        if collisions
            collFree = false;
            return;
        end
    end
    collFree = true;
end

% takes an 2xn matrix X representing n points to check for collision and 
% a 2xm matrix Y of m points that are the hulls of the convex polygon
% obstacle. Returns an 1xn row vector whose ith element is true if the ith
% vector, X(:,i), IS inside the collision polygon (need to ~ to get NOT)
function collisions = check_collisions(X,Y)
    xh = Y(1,:);            % x-coordinates of hull points
    yh = Y(2,:);            % y-coordinates of hull points
    k = convhull(xh,yh);    % indices of points in Y that define polygon
    
    xp = xh(k);             % x-coordinates of polygon vertices
    yp = yh(k);             % y-coordinates of polygon vertices
    
    xq = X(1,:);            % x-coordinates of query points
    yq = X(2,:);            % y-coordinates of query points
    collisions = inpolygon(xq, yq, xp, yp);
end

