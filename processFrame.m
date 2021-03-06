function [S1, T]= processFrame(img0, img1, S0, K)
addpath( 'all_solns/05_ransac')
addpath('all_solns/00_camera_projection');
addpath( 'all_solns/04_8point/triangulation');
addpath( 'all_solns/01_pnp');
%% matlab point tracker (KLT)
% the tracker developped in the exercises was first implemented. But it is
% terribly slow (several seconds to track around 80 keypoints), so the
% preimplemented function of matlab is used

% The 3D points are stored from start to end -> this array does never change
S1.p3D = S0.p3D; 
% track candidate keypoints from last frame to current
pointTracker = vision.PointTracker('BlockSize', [51 51]); 
% fliplr is needed because of the different convention on how to store points                                
initialize(pointTracker, fliplr(S0.kp(1:2, :)'),  img0);
[points, ~, scores] = step(pointTracker,img1);

% translate to point storing convention of this course
new_kp = [fliplr( points )'; ones(1, size(points, 1))];
% now only keep the keypoints and their correspondences that could be
% tracked
S1.kp = new_kp(:, scores > 0.8 );
S1.corr = S0.corr( scores > 0.8); % array that stores indices of corresponding S1.p3D
points_3D = S0.p3D(:, S1.corr );



%% P3P RANSAC
%%% RANSAC %%% -> This for loop is taken one to one from the exercise.
%                 The only changes are variable names and eliminating some
%                 unused code.
% It effectively tries to estimate the new pose with 2D -> 3D point
% correspondeces. The outliers found by RANSAC are conviently removed as
% keypoints
num_iterations = 1000;
inlier_mask = [];
% <-- swap of the point coordinates (x <-> y) 
matched_query_keypoints = [S1.kp(2, :); S1.kp(1, :)] ;

max_num_inliers = 0;
k = 3;
pixel_tolerance = 10;

R_C_W = eye(3);
t_C_W = zeros(3, 1);

for i = 1:num_iterations
    [landmark_sample, idx] = datasample(...
        points_3D , k, 2, 'Replace', false);
    keypoint_sample = matched_query_keypoints(:, idx);
    

    normalized_bearings = K\[keypoint_sample; ones(1, 3)];
    for ii = 1:3
        normalized_bearings(:, ii) = normalized_bearings(:, ii) / ...
            norm(normalized_bearings(:, ii), 2);
    end
    poses = p3p(landmark_sample, normalized_bearings);
    R_C_W_guess = zeros(3, 3, 2);
    t_C_W_guess = zeros(3, 1, 2);
    for ii = 0:1
        R_W_C_ii = real(poses(:, (2+ii*4):(4+ii*4)));
        t_W_C_ii = real(poses(:, (1+ii*4)));
        R_C_W_guess(:,:,ii+1) = R_W_C_ii';
        t_C_W_guess(:,:,ii+1) = -R_W_C_ii'*t_W_C_ii;
    end
   
    
    % Count inliers:
    projected_points = projectPoints(...
        (R_C_W_guess(:,:,1) * points_3D) + ...
        repmat(t_C_W_guess(:,:,1), ...
        [1 size(points_3D, 2)]), K);
    difference = matched_query_keypoints - projected_points;
    errors = sum(difference.^2, 1);
    is_inlier = errors < pixel_tolerance^2;
    
    projected_points = projectPoints(...
        (R_C_W_guess(:,:,2) * points_3D) + ...
        repmat(t_C_W_guess(:,:,2), ...
    [1 size(points_3D(:, :), 2)]), K);
    difference = matched_query_keypoints - projected_points;
    errors = sum(difference.^2, 1);
    alternative_is_inlier = errors < pixel_tolerance^2;
    
    if nnz(alternative_is_inlier) > nnz(is_inlier)
        is_inlier = alternative_is_inlier;
        alternative_sol = true;
    else
        alternative_sol = false;
    end

    if nnz(is_inlier) > max_num_inliers && nnz(is_inlier) >= 6
        max_num_inliers = nnz(is_inlier);        
        inlier_mask = is_inlier;
        if alternative_sol
            R_C_W = R_C_W_guess(:, :, 2);
            t_C_W = t_C_W_guess(:, :, 2);
        else
            R_C_W = R_C_W_guess(:, :, 1);
            t_C_W = t_C_W_guess(:, :, 1);
        end        
    end
end


% remove outliners
S1.kp  = S1.kp(:, inlier_mask);
S1.corr = S1.corr( inlier_mask);

% prepare data for pose estimation via DLT
% <-- swap of the point coordinates (x <-> y) 
p_2D = [S1.kp(2,:);S1.kp(1,:)];
p_3D = S1.p3D(1:3, S1.corr);

% call DLT algorithm
M_C_W = estimatePoseDLT(...
        p_2D', ...
        p_3D', K);
R_C_W = M_C_W(:, 1:3);
t_C_W = M_C_W(:, end);

    
% store the pose
T = [ R_C_W t_C_W; 0 0 0 1];

%% find new 3D landmarks 

[S1] = new3DLandmarks(img0, img1, T, K, S1, S0);

%% updating landmarks

% get candidate keypoints
harris_patch_size = 8;
harris_kappa = 0.08;
num_keypoints = 50;
nonmaximum_supression_radius = 8;

harris_scores = harris(img1, harris_patch_size, harris_kappa);
keypoints = selectKeypoints(harris_scores, num_keypoints, ...
                            nonmaximum_supression_radius);

keypoints = keypoints(:, sum( ismember( keypoints, ...
                             round( S1.kp(1:2, :)) ), 1) ~= 2 );

S1.kp_cand0 = [S1.kp_cand0 keypoints];
S1.kp_cand = [S1.kp_cand keypoints];
S1.T_cand = [S1.T_cand repmat(T(:), 1, size( keypoints, 2))];


end