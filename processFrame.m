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
pointTracker = vision.PointTracker('BlockSize', [31 31]); 
% fliplr is needed because of the different convention on how to store points                                
initialize(pointTracker, fliplr(S0.kp(1:2, :)'),  img0);
[points, ~, scores] = step(pointTracker,img1);

% translate to point storing convention of this course
new_kp = [fliplr( points )'; ones(1, size(points, 1))];
% now only keep the keypoints and their correspondences that could be
% tracked
S1.kp = new_kp(:, scores > 0.995 );
S1.corr = S0.corr( scores > 0.995 ); % array that stores indices of corresponding S1.p3D
points_3D = S0.p3D(:, S1.corr );

%% P3P RANSAC
%%% RANSAC %%% -> This for loop is taken one to one from the exercise.
%                 The only changes are variable names and eliminating some
%                 unused code.
% It effectively tries to estimate the new pose with 2D -> 3D point
% correspondeces. The outliers found by RANSAC are conviently removed as
% keypoints
num_iterations = 500;
inlier_mask = [];
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

% prepare data for pose estimation via DLT  <--- ADDED
p_2D = [S1.kp(2,:); S1.kp(1,:)];
p_3D = S1.p3D(1:3, S1.corr);

% call DLT algorithm                        <--- ADDED
M_C_W = estimatePoseDLT(...
        p_2D', ...
        p_3D', K);
R_C_W = M_C_W(:, 1:3);
t_C_W = M_C_W(:, end);

    
% store the pose
T = [ R_C_W t_C_W; 0 0 0 1];

%% find new 3D landmarks -> this part does not work well yet
% there are terrible outliers generated by this section. 
% maybe ransac would be a good idea
% new3D_points = []; % just a preallocation needed
% 
% % At the beginning, there are no tracked candidates, so this step can be
% % left away
% 
% if ~isempty( S0.kp_cand0 )
%     % the second frame doesnt have tracked keypoints yet, only initial kp
%     if isempty( S0.kp_cand )
%         S0.kp_cand = S0.kp_cand0;
%     end
%     % track the keypoints from the last picture to the current one. Same
%     % tracker is used as in the code above.
%     pointTracker.release();
%     initialize(pointTracker, fliplr( round(S0.kp_cand)' ),  img0);
%     [points, found, scores] = step(pointTracker, img1);
%     
%     % set the tracked points to candidates to get a new landmark
%     S1.kp_cand  = fliplr( points( found, : ) )';
%     % only keep the transformation matrix and initial keypoint if the kp
%     % could be tracked
%     S1.T_cand   = S0.T_cand(:, found );
%     S1.kp_cand0 = S0.kp_cand0(:,found);
%     
%     % this is the criteria after which good triangulation of new landmarks
%     % is decided. Not good yet, but may be changed. The import thing is to
%     % keep the shape of readyToTr
%     
%     % baseline criteria -> not enough if baseline is more or less
%     % perpendicular to camera plane
%     min_baseline = 0.9;
%     baseline = sum( (T(:, 4) - S1.T_cand(13:16, :)).^2, 1);
%     
%     %%% the rotation matrices are transformed to rotation vectors. this
%     %%% allowes a direct comparison (since differences of rot matrices do
%     %%% not make sense). the project guideline proposes bearing vectors to
%     %%% compare -> implement this if you now what is meant by it
%     angle_btf = acos(0.5*(S1.T_cand(1, :) + ...
%                           S1.T_cand(6, :) + ...
%                           S1.T_cand(11,:) - 1) );
%     rot_vec = 1./(2*sin(angle_btf)).*[S1.T_cand(7, :)-S1.T_cand(10,:); ...
%                                       S1.T_cand(9, :)-S1.T_cand(3, :); ...
%                                       S1.T_cand(2, :)-S1.T_cand(5, :)];
%     rot_vec = rot_vec.*angle_btf;
%     
%     angle_T = acos( 0.5*( T(1, 1) + T(2, 2) + T(3, 3) - 1));
%     if (abs( angle_T )<1e-9)
%         rot_vec_T = zeros(3,1);
%     else
%         rot_vec_T = 1/(2*sin( angle_T ))*[T(3, 2) - T(2, 3);
%                                           T(1, 3) - T(3, 1);
%                                           T(2, 1) - T(1, 2)];
%     end
%     rot_vec_T = angle_T*rot_vec_T;
%     rot_vec_diff = abs(rot_vec - rot_vec_T); 
%     
%     % this is then the criteria for triangulating new landmarks
%     readyToTr = baseline > min_baseline^2 & ...
%                 sum( rot_vec_diff.^2, 1) < 5;
%     %%%             double check                 %%%
% 
%     
%     
%     % initialize the triangulation with the points that are decided to be
%     % well
%     kp0_triang = [S1.kp_cand0(:, readyToTr ); ones(1, sum( readyToTr ) )];
%     kp1_triang = [S1.kp_cand(:, readyToTr ); ones(1, sum( readyToTr ) )];
%     
%     kp0_triang_sw = [kp0_triang(2,:); kp0_triang(1,:); kp0_triang(3,:)];
%     kp1_triang_sw = [kp1_triang(2,:); kp1_triang(1,:); kp1_triang(3,:)];
%     
%     T_triang = S1.T_cand(:, readyToTr);
%     i = 1;
%     
%     % maybe a outliner removal with ransac is needed at this point
%     while i <= size(kp1_triang_sw, 2)
%         % check for all point that have the same transformation matrix
%         % (because they were detected the first time in the same frame)
%         
%         %T_triang(:, i) == T_triang
%         same_frame = find( sum( T_triang(:, i) == T_triang, 1) / 16 == 1 );
%         first = same_frame(1);
%         last = same_frame(end);
%         
%         % and extract all of these points at once
%         T_it = reshape( T_triang(:, i), 4, 4 );
%         M0 = K*T_it(1:3, :);
%         M1 = K*T(1:3, :);
%         new3D_points = [new3D_points...
%             linearTriangulation( kp0_triang_sw(:, first:last), ...
%             kp1_triang_sw(:, first:last), M0, M1)];
%         % jump to the next section of points
%         i = last + 1;
%     end
%     %%% here a check if point is in front of camera could be added %%%
%     
%     % update the state space
%     S1.corr = [S1.corr  (length(S1.corr) + 1: ...
%                          length(S1.corr) + 1 + size(new3D_points, 2) )];
%     if ~isempty( new3D_points )
%         S1.p3D  = [S1.p3D   new3D_points(1:3, :)];
%     end
%     S1.kp   = [S1.kp    kp1_triang];
%     
%     % remove candidate keypoints
%     S1.kp_cand0(:, readyToTr) = [];
%     S1.kp_cand(:, readyToTr) = [];
%     S1.T_cand(:, readyToTr) = [];
% else
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     S1.kp_cand = [];
%     S1.kp_cand0 = [];
%     S1.T_cand = [];
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% end
[S1] = new3DLandmarks(img0, img1, T, K, S1, S0);
    
%% updating landmarks

% get candidate keypoints
harris_patch_size = 8;
harris_kappa = 0.08;
num_keypoints = 50;
nonmaximum_supression_radius = 8;

harris_scores = harris(img1, harris_patch_size, harris_kappa);
keypoints = selectKeypoints(harris_scores, num_keypoints,...
    nonmaximum_supression_radius);

keypoints = keypoints(:, sum( ismember( keypoints, round( S1.kp(1:2, :)) ), 1) ~= 2 );


plot_1 = S1.kp_cand;
plot_3 = keypoints;

S1.kp_cand0 = [S1.kp_cand0 keypoints];
S1.kp_cand = [S1.kp_cand keypoints];
S1.T_cand = [S1.T_cand repmat(T(:), 1, size( keypoints, 2))];
plot_2 = S1.kp_cand;
% if ~isempty( plot_1)
%     if ~isempty( plot_2)
%         S1.kp_cand(1,:)
%         plot_CheckKeyPoints(plot_1, plot_2, plot_3, img1);
%     end
% end

end