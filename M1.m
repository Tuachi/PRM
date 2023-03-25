% Input: q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples to sample
% Output: qs -> num_samples x 4 matrix of joint angles,
%               all within joint limits

function qs = M1(q_min, q_max, num_samples)
    % matlab documentation of how to use the function rand to generate
    % random samples

    %Generate a 10-by-1 column vector of uniformly distributed numbers in the interval (-5,5).
    %r = -5 + (5+5)*rand(10,1)

    %In general, you can generate N random numbers in the interval (a,b) with the formula r = a + (b-a).*rand(N,1)
    q1 = q_min(1) + (q_max(1) - q_min(1)) *rand(num_samples, 1);
    q2 = q_min(2) + (q_max(2) - q_min(2)) *rand(num_samples, 1);
    q3 = q_min(3) + (q_max(3) - q_min(3)) *rand(num_samples, 1);
    q4 = q_min(4) + (q_max(4) - q_min(4)) *rand(num_samples, 1);

    qs = [q1, q2, q3, q4];

end