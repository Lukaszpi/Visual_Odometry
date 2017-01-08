function [S0, T_final] = initialization(img1, img2, K)
% add to path
addpath( 'all_solns/02_detect_describe_match');
addpath( 'all_solns/04_8point/8point');
addpath( 'all_solns/04_8point');
addpath( 'all_solns/04_8point/triangulation')
addpath( 'all_solns/04_8point/plot')
%% set parameters
% detecting and matchingfunction [S1] = new3DLandmarks(img0, img1, T, K, S1, S0)


%% find new 3D landmarks -> this part does not work well yet
% there are terrible outliers generated by this section. 
% maybe ransac would be a good idea
new3D_points = []; % just a preallocation needed

% At the beginning, there are no tracked candidates, so this step can be
% left away

if ~isempty( S0.kp_cand0 )
    % the second frame doesnt have tracked keypoints yet, only initial kp
    if isempty( S0.kp_cand )
        S0.kp_cand = S0.kp_cand0;
    end
    % track the keypoints from the last picture to the current one. Same
    % tracker is used as in the code above.
    pointTracker = vision.PointTracker('BlockSize', [51 51]); 
    pointTracker.release();
    initialize(pointTracker, fliplr( round(S0.kp_cand)' ),  img0);
    [points, found, scores] = step(pointTracker, img1);
    
    % set the tracked points to candidates to get a new landmark
    S1.kp_cand  = fliplr( points( found, : ) )';
    % only keep the transformation matrix and initial keypoint if the kp
    % could be tracked
    S1.T_cand   = S0.T_cand(:, found );
    S1.kp_cand0 = S0.kp_cand0(:,found);
    
    % this is the criteria after which good triangulation of new landmarks
    % is decided. Not good yet, but may be changed. The import thing is to
    % keep the shape of readyToTr
    
    % baseline criteria -> not enough if baseline is more or less
    % perpendicular to camera plane
    min_baseline = 0.9;
    baseline = sum( (T(:, 4) - S1.T_cand(13:16, :)).^2, 1);
    
    %%% the rotation matrices are transformed to rotation vectors. this
    %%% allowes a direct comparison (since differences of rot matrices do
    %%% not make sense). the project guideline proposes bearing vectors to
    %%% compare -> implement this if you now what is meant by it
    angle_btf = acos(0.5*(S1.T_cand(1, :) + ...
                          S1.T_cand(6, :) + ...
                          S1.T_cand(11,:) - 1) );
    rot_vec = 1./(2*sin(angle_btf)).*[S1.T_cand(7, :)-S1.T_cand(10,:); ...
                                      S1.T_cand(9, :)-S1.T_cand(3, :); ...
                                      S1.T_cand(2, :)-S1.T_cand(5, :)];
    rot_vec = rot_vec.*angle_btf;
    
    angle_T = acos( 0.5*( T(1, 1) + T(2, 2) + T(3, 3) - 1));
    if (abs( angle_T )<1e-9)
        rot_vec_T = zeros(3,1);
    else
        rot_vec_T = 1/(2*sin( angle_T ))*[T(3, 2) - T(2, 3);
                                          T(1, 3) - T(3, 1);
                                          T(2, 1) - T(1, 2)];
    end
    rot_vec_T = angle_T*rot_vec_T;
    rot_vec_diff = abs(rot_vec - rot_vec_T); 
    
    % this is then the criteria for triangulating new landmarks
    readyToTr = baseline > min_baseline^2 & ...
                sum( rot_vec_diff.^2, 1) > 0.01;
    
    % initialize the triangulation with the points that are decided to be
    % well
    kp0_triang = [S1.kp_cand0(:, readyToTr ); ones(1, sum( readyToTr ) )];
    kp1_triang = [S1.kp_cand(:, readyToTr ); ones(1, sum( readyToTr ) )];
    
    kp0_triang_sw = [kp0_triang(2,:); kp0_triang(1,:); kp0_triang(3,:)];
    kp1_triang_sw = [kp1_triang(2,:); kp1_triang(1,:); kp1_triang(3,:)];
    kp1_triang_sw - kp0_triang_sw
    
    T_triang = S1.T_cand(:, readyToTr);
    i = 1;
    
    % maybe a outliner removal with ransac is needed at this point
    while i <= size(kp1_triang_sw, 2)
        % check for all point that have the same transformation matrix
        % (because they were detected the first time in the same frame)
        
        %T_triang(:, i) == T_triang
        same_frame = find( sum( T_triang(:, i) == T_triang, 1) / 16 == 1 );
        first = same_frame(1);
        last = same_frame(end);
        
        
% Lukasz note - I've noticed that the algorithm in run_sfm.m works better
% with the 3D points estimation when the camera 1 is in the origin without
% the rotation, hence I tried to predict the points from the eye(3,4) pose
% and then translate them by the camera coordinates. The part until linear
% triangulation should be fine if there is enought points to determine the
% relative pose. I haven't found the formula to translate these points with
% the rotation and believe that it is the key. Or maybe we just need to
% have slightly different T to estimate M in original code. Current one
% places all the points next to the origin hence after some time when new
% points are used for the new posee the pose lookes into the origin.
%
% %
%         % taken from run_sfm.m start
%         T_it = reshape( T_triang(:, i), 4, 4 );
%         
%         E = estimateEssentialMatrix(kp1_triang_sw(:, first:last),...
%             kp0_triang_sw(:, first:last), K, K);
% 
%         % Extract the relative camera positions (R,T) from the essential matrix
% 
%         % Obtain extrinsic parameters (R,t) from E
%         [Rots,u3] = decomposeEssentialMatrix(E);
% 
%         % Disambiguate among the four possible configurations
%         [R_C2_W,T_C2_W] = disambiguateRelativePose(Rots,u3,...
%             kp1_triang_sw(:, first:last),...
%             kp0_triang_sw(:, first:last),K,K);
% 
%         % Triangulate a point cloud using the final transformation (R,T)
%         T2 = eye(3,4);
%         T1 = [R_C2_W, T_C2_W];
%         M2 = K * T1;
%         M1 = K * T2;
%         new3D_triang_points = linearTriangulation(kp1_triang_sw(:, first:last),...
%             kp0_triang_sw(:, first:last),M2,M1);
%         
%         new3D_triang_points = T_it*new3D_triang_points;
%         
%         new3D_points = [new3D_points new3D_triang_points];
%         % taken from run_sfm.m end


        
        %and extract all of these points at once
        T_it = reshape( T_triang(:, i), 4, 4 );
        M0 = K*T_it(1:3, :);
        M1 = K*T(1:3, :);
        new3D_points = [new3D_points...
            linearTriangulation( kp0_triang_sw(:, first:last), ...
            kp1_triang_sw(:, first:last), M0, M1)];

        % jump to the next section of points
        i = last + 1;
    end
    %check if the point is in front of the camera
    if ~isempty( new3D_points )
        T_check = [T(1:3,1:3), -T(1:3,1:3)'*T(1:3,4)];
        point_prim = T_check*new3D_points;
        new3D_points = new3D_points(:,point_prim(3,:) > 0);
        S1.corr = [S1.corr  (size(S1.p3D, 2) + 1: ...
                         size(S1.p3D, 2) + size(new3D_points, 2) )];
        S1.p3D  = [S1.p3D   new3D_points(1:3, :)];
        S1.kp   = [S1.kp    kp1_triang(:,point_prim(3,:) > 0)];
    end    
    % remove candidate keypoints
    S1.kp_cand0(:, readyToTr) = [];
    S1.kp_cand(:, readyToTr) = [];
    S1.T_cand(:, readyToTr) = [];
else
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    S1.kp_cand = [];
    S1.kp_cand0 = [];
    S1.T_cand = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
end
harris_patch_size = 9;
harris_kappa = 0.08;
%%% best num keypoints -> intresting enough the matched features heaviliy
%%% depend on the amount of keypoints (not linearly!). Setting
%%% num_keypoints to 1000 for the KITTI set will lead to a decrease by 70%
%%% KITTI: 800
num_keypoints = 800;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 4;

% RANSAC
ransac_iteration = 1000; % obtained by trial and error, can be changed
number_of_points = 8; % fixed value
pixel_threshold = 0.15; % obtained by trial and error, can be changed
max_inliers = 0; % just a pre allocation, is necessary

%% detecting and matching of keypoints
% img0: get and describe keypoints
harris_scores = harris(img1, harris_patch_size, harris_kappa);
keypoints_1 = selectKeypoints(harris_scores, num_keypoints, ...
                            nonmaximum_supression_radius);
descriptors = describeKeypoints(img1, keypoints_1, descriptor_radius);

% img1: get and describe keypoints
harris_scores_2 = harris(img2, harris_patch_size, harris_kappa);
keypoints_2 = selectKeypoints(harris_scores_2, num_keypoints, ...
                              nonmaximum_supression_radius);
descriptors_2 = describeKeypoints(img2, keypoints_2, descriptor_radius);

% match keypoints
% index of matches -> img2 keypoint index
% value of matches -> img1 keypoint index
matches = matchDescriptors(descriptors_2, descriptors, match_lambda);
NumMatches = sum( matches ~= 0 );

% make homogenous points (so just add a 1 for the z-coord)
kp_hom_coord_1 = [ keypoints_1; ones( 1, num_keypoints) ];
kp_hom_coord_2 = [ keypoints_2; ones( 1, num_keypoints) ];
% only keep the ones that were matched. After this the p_1(i) corresponds
% to p_2(i)
p_1 = kp_hom_coord_1( :, matches( matches ~= 0));
p_2 = kp_hom_coord_2( :, matches ~= 0);

%% apply RANSAC with 8-point algorithm

% swap of the point coordinates (x <-> y) 
p_1_ch = [p_1(2,:);p_1(1,:);p_1(3,:)];
p_2_ch = [p_2(2,:);p_2(1,:);p_2(3,:)];

for iterate = 1:ransac_iteration
    % randomly sample number_of_points samples between 1 and NumMatches
    % this can be done in this way because point p_1(i) corresponds with
    % p_2(i)
    idx = datasample( 1:NumMatches, number_of_points, 2, 'Replace', ...
                               false);
    % apply 8-point algorithm
    F = fundamentalEightPoint_normalized( p_1_ch( :, idx), ...
                                          p_2_ch( :, idx) );
    
%%% error measurement with epipolar line distance %%%%
    % this is the one I would suggest because it runs reasonably fast
    % the code is taken from
    % /all_solns/04_8point/8point/distPoint2EpipolarLine.m
    homog_points = [p_1_ch , p_2_ch];
    epi_lines = [F.'*p_2_ch, F*p_1_ch];

    denom = epi_lines( 1, :).^2 + epi_lines( 2, :).^2;
    cost = sqrt( ( sum( epi_lines.*homog_points, 1).^2 )./denom / NumMatches );
    ncost = cost( 1: NumMatches) + cost( NumMatches + 1: 2*NumMatches);
    inliers = ncost < pixel_threshold;

    % keep the solution with the most inliers
    if sum( inliers ) > sum( max_inliers )
        max_inliers = inliers;
    end
end
%% Determine the camera poses
% only keep the inliers for further computations
p1_in = p_1( :,  max_inliers );
p2_in = p_2( :,  max_inliers );

% <-- swap of the point coordinates (x <-> y) 
p1_in_tiang = [p1_in(2,:);p1_in(1,:);p1_in(3,:)];
p2_in_tiang = [p2_in(2,:);p2_in(1,:);p2_in(3,:)];

% again compute the fundamental matrix, now with all points
E = estimateEssentialMatrix(p1_in_tiang, p2_in_tiang, K, K);

[R, T] = decomposeEssentialMatrix(E);
[R, T] = disambiguateRelativePose(R, T, p1_in_tiang, p2_in_tiang, K, K);

% now actually calculate the 3D points
M1 = K*eye( 3, 4);
M2 = K*[R T];
points_3D = linearTriangulation(p1_in_tiang, p2_in_tiang, M1, M2);

% Check if the points are in front of the camera
T_check = [R, -R'*T];
P_prim = T_check*points_3D;
points_3D = points_3D(:,P_prim(3,:)>0);

%% construct state space struct
% stores keypoints of current frame
S0.kp = p2_in(:,P_prim(3,:)>0);
% stores all 3D points detected (so the whole map)
S0.p3D = points_3D(1:3, :);
% the i-th entry of S0.corr links the i-th keypoint in S0.kp to the 3D
% point S0.p3D( S0.corr( i ) )
S0.corr = 1:size(points_3D, 2);
% the following matrices are used for extracting new landmarks
S0.T_cand = []; % keep track of transformation matrices corresponding to tracked keypoints
S0.kp_cand0 = []; % store coordinates of keypoints in their first frame
S0.kp_cand = []; % store coordinates of keypoints in this frame

T_final = [R T];
end