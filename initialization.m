function S0 = initialization(img1, img2, K)
% add to path
addpath( 'all_solns/02_detect_describe_match');
addpath( 'all_solns/04_8point/8point');
addpath( 'all_solns/04_8point');
addpath( 'all_solns/04_8point/triangulation')
%% set parameters
% detecting and matching
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
%%%% add random outliners -> can be used to test the outliner detection by ransac %%%%
% for i = 1:length(matches)
%     if matches(i) == 0 && rand() < 0.01
%         t = 0;
%         while sum(matches == t) ~= 0
%             t = ceil(rand()*length(matches));
%         end
%         matches(i) = t;
%     end  
% end
% NumMatches = sum( matches ~= 0 );
%%%%%
%% apply RANSAC with 8-point algorithm
% make homogenous points (so just add a 1 for the z-coord)
kp_hom_coord_1 = [ keypoints_1; ones( 1, num_keypoints) ];
kp_hom_coord_2 = [ keypoints_2; ones( 1, num_keypoints) ];
% only keep the ones that were matched. After this the p_1(i) corresponds
% to p_2(i)
p_1 = kp_hom_coord_1( :, matches( matches ~= 0));
p_2 = kp_hom_coord_2( :, matches ~= 0);

for iterate = 1:ransac_iteration
    % randomly sample number_of_points samples between 1 and NumMatches
    % this can be done in this way because point p_1(i) corresponds with
    % p_2(i)
    idx = datasample( 1:NumMatches, number_of_points, 2, 'Replace', ...
                               false);
    % apply 8-point algorithm
    F = fundamentalEightPoint_normalized( p_1( :, idx), ...
                                          p_2( :, idx) );
            

%%% error measurement with epipolar line distance %%%%
    % this is the one I would suggest because it runs reasonably fast
    % the code is taken from
    % /all_solns/04_8point/8point/distPoint2EpipolarLine.m
    homog_points = [p_1 , p_2];
    epi_lines = [F.'*p_2, F*p_1];

    denom = epi_lines( 1, :).^2 + epi_lines( 2, :).^2;
    cost = sqrt( ( sum( epi_lines.*homog_points, 1).^2 )./denom / NumMatches );
    ncost = cost( 1: NumMatches) + cost( NumMatches + 1: 2*NumMatches);
    inliers = ncost < pixel_threshold;

%%%% error measurement with reprojection error %%%%%
%     % lecture notes -> most accurate and widely used
%     % experience -> way to slow, not suggested
%     E = K'*F*K;
%     [R, T] = decomposeEssentialMatrix(E);
%     [R, T] = disambiguateRelativePose(R, T, p_1, p_2, K, K);
% 
%     % now actually calculate the 3D points
%     M1 = K*eye( 3, 4);
%     M2 = K*[R T];
%     points_3D = linearTriangulation(p_1, p_2, M1, M2);
%     check1 = K*points_3D(1:3, :);
%     check1 = check1(1:2, :)./check1(3,:);
%     p = [R T]*points_3D;
%     check2 = K*p(1:3, :);
%     check2 = check2(1:2, :)./check2(3,:);
%     error = sqrt( sum( (check1 - p_1(1:2, :)).^2, 1) ) ...
%           + sqrt( sum( (check2 - p_2(1:2, :)).^2, 1) );
%     inliers = error < 1;    
    
    % keep the solution with the most inliers
    if sum( inliers ) > sum( max_inliers )
        max_inliers = inliers;
    end
end
%%
% only keep the inliers for further computations
p1_in = p_1( :,  max_inliers );
p2_in = p_2( :,  max_inliers );

% again compute the fundamental matrix, now with all points
F = fundamentalEightPoint_normalized( p1_in, p2_in );

% compute R and T from F
E = K'*F*K;
[R, T] = decomposeEssentialMatrix(E);
[R, T] = disambiguateRelativePose(R, T, p1_in, p2_in, K, K);

% now actually calculate the 3D points
M1 = K*eye( 3, 4);
M2 = K*[R T];
points_3D = linearTriangulation(p1_in, p2_in, M1, M2);
%% some plotting. should be removed at some point
% figure(4);
% imshow(img1);
% hold on;
% plot(p_1(2, :), p_1(1, :), 'rx', 'Linewidth', 2);
% t = find( matches ~= 0 );
% t = t( ~max_inliers );
% matches( t ) = zeros( length(t), 1);
% plotMatches(matches, keypoints_2, keypoints_1);
% 
% check1 = K*points_3D(1:3, :);
% check1 = check1(1:2, :)./check1(3,:);
% 
% figure(1)
% imshow(img1); hold on;
% plot(check1(2, :), check1(1, :), 'rx', 'LineWidth', 2);
% plot(p_1(2, :), p_1(1, :), 'bx', 'LineWidth', 2);
%% construct state space struct
% stores keypoints of current frame
S0.kp = p2_in;
% stores all 3D points detected (so the whole map)
S0.p3D = points_3D(1:3, :);
% the i-th entry of S0.corr links the i-th keypoint in S0.kp to the 3D
% point S0.p3D( S0.corr( i ) )
S0.corr = 1:size(p2_in, 2);
% the following matrices are used for extracting new landmarks
S0.T_cand = []; % keep track of transformation matrices corresponding to tracked keypoints
S0.kp_cand0 = []; % store coordinates of keypoints in their first frame
S0.kp_cand = []; % store coordinates of keypoints in this frame


end