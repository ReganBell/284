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
    
    figure

    plot(xp,yp) % polygon
    axis equal

    hold on
    plot(xq(collisions),yq(collisions),'r+') % points inside
    plot(xq(~collisions),yq(~collisions),'bo') % points outside
    hold off
end