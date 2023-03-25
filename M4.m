% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found

function [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    stepsize = 0.1;
    % beta is the pecentage or the chance that q_target = q_goal
    b = 0.1;
    
    v = q_start;
    root = [];
    
    path_found = false;
    

    while true
        
        if rand(0,1) < b 
            q_target = q_goal;
        else 
            %q_target becomes the random node sample 
            q_target = M1(q_min, q_max, 1);
        end
        
        %nearest neigbour of q_target in v
        closest_idx = closestnode(v, q_target);
        
        q_near = v(closest_idx,:);
        q_new = q_near + ((stepsize/norm(q_target - q_near)) * (q_target - q_near));
        
        if ~check_collision(robot, q_new, link_radius, sphere_centers, sphere_radii)
            v = [v;q_new];
            root = [root; closest_idx];
            % E = [E;(q_near, q_new)] % come back here
        end
        
        if norm(q_new - q_goal) < stepsize
            path_found = true;
            break
        end
        % check if q_goal is in v
    end
    if path_found
        path = q_goal;
        parent = size(v,1);
        while parent ~= 1
            path = [v(parent,:); path];
            parent = root(parent-1);
        end
    end
end

function idx = closestnode(samples, q)
    distances = zeros(size(samples, 1),1);
    for i = 1:size(samples,1)
        distances(i) = norm(samples(i,:)-q);
    end
    [~, idx] = min(distances);
end