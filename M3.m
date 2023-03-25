% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        samples -> num_samples x 4 matrix, vertices in the roadmap
%        adjacency -> num_samples x num_samples matrix, the weighted
%                     adjacency matrix denoting edges in the roadmap
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

function [path, path_found] = M3(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    
    % Find the closest samples to the start and goal configurations
    start_idx = findClosestSample(samples, q_start, robot, link_radius, sphere_centers, sphere_radii);
    goal_idx = findClosestSample(samples, q_goal, robot, link_radius, sphere_centers, sphere_radii);
    G = graph(adjacency);
    
    % Use shorthest path algorithm to find the shortest path
    path_indices = shortestpath(G, start_idx, goal_idx);
    
    % Convert the path indices to configurations
    path = samples(path_indices,:);
    path = [path; q_goal];

    % Check if a path was found
    if isempty(path_indices)
        path = [];
        path_found = false;
    else
        path_found = true;
    end 
end

% Function to find the closest sample to a given configuration
function idx = findClosestSample(samples, q, robot, link_radius, sphere_centers, sphere_radii)
    distances = zeros(size(samples,1),1);
    for i = 1:size(samples,1)
        if ~check_collision(robot, q, link_radius, sphere_centers, sphere_radii)
            distances(i) = norm(samples(i,:) - q);
        end
    end
    [~,idx] = min(distances);
end
