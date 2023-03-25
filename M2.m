% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples in PRM
%        num_neighbors -> Integer denoting number of closest neighbors to
%                         consider in PRM
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: samples -> num_samples x 4 matrix, sampled configurations in the
%                    roadmap (vertices)
%         adjacency -> num_samples x num_samples matrix, the weighted
%                      adjacency matrix denoting edges between roadmap
%                      vertices. adjacency(i,j) == 0 if there is no edge
%                      between vertex i and j; otherwise, its value is the
%                      weight (distance) between the two vertices. For an
%                      undirected graph, the adjacency matrix should be
%                      symmetric: adjacency(i,j) == adjacency(j,i)

function [samples, adjacency] = M2(robot, q_min, q_max, num_samples, num_neighbors,    link_radius, sphere_centers, sphere_radii)
    samples = zeros([1,4]);
    adjacency = zeros([100, 100]);
    n = 1;

    while size(samples, 1) < num_samples
         
        q1 = q_min(1) + (q_max(1) - q_min(1)) *rand(1, 1);
        q2 = q_min(2) + (q_max(2) - q_min(2)) *rand(1, 1);
        q3 = q_min(3) + (q_max(3) - q_min(3)) *rand(1, 1);
        q4 = q_min(4) + (q_max(4) - q_min(4)) *rand(1, 1);

        joints = [q1, q2, q3, q4];
        if ~check_collision(robot, joints, link_radius, sphere_centers, sphere_radii)
        
            samples(n,:) = [q1, q2, q3, q4];
            n = n + 1;
        end
    end

     for i = 1:num_samples
        distances = zeros(num_samples, 1);
        % for each each sample, compute 100 distances 
        for j = 1:num_samples
            equid_distance = norm(samples(i,:) - samples(j,:));
            distances(j) = equid_distance;
        end
        
        % Sort the distances and keep track of the index
        [sorted_distances, I] = sort(distances, 'ascend');
        
        % Connect the node to its `num_neighbors` closest neighbors
        for k = 2:num_neighbors
            distant = sorted_distances(k);
            if ~check_edge(robot, samples(i,:), samples(I(k),:), link_radius, sphere_centers, sphere_radii)
                adjacency(i, I(k)) = distant;
                adjacency(I(k), i) = distant;
            end
        end
    end

end

