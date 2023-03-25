% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function smoothed_path = M5(robot, path, link_radius, sphere_centers, sphere_radii)
    %traverse the tree from the goal node to the start node
    smoothed_path = [];
    node = size(path,1);
    while node ~=1
        %check the parent node of the current node
        for i = 1:node
            if ~check_edge(robot, path(node,:), path(i,:), link_radius, sphere_centers, sphere_radii)
                % if the edge between the current node and its parent node
                % is valid, set the parent node as the current node
                smoothed_path = ([path(node,:); smoothed_path]);
                node = i;
                break;
            end
        end
    end
end