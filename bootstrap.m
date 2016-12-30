function [ img1,img2 ] = bootstrap( path, K )
img1 = imread([path ...
    sprintf('%06d.png',1)]);
i = 2;
t = [0,0,0,0];
a = 1;
% set a limit for the minimal and maximal length of the transition vector.
lower_limit = 15;
upper_limit = 20;
while a == 1
    img2 = imread([path ...
    sprintf('%06d.png',i)]);
%     % I first tried it with the built-in functions but I get the feeling
%     that their code is more buggy than ours... 
%     cameraParams = cameraParameters('IntrinsicMatrix',K);
%     
%     points = detectFASTFeatures(img1);
%     pointTracker = vision.PointTracker;
%     initialize(pointTracker,points.Location,img1);
%     [points_new,points_validity] = step(pointTracker,img2);
%     points_new = [points_new(points_validity,1),points_new(points_validity,2)];
%     points = [points.Location(points_validity,1),points.Location(points_validity,2)];
%     size(points)
%     size(points_new)
%     E = estimateEssentialMatrix(points,points_new,cameraParams);
%     [U,S,~] = svd(E);
%     W = [[0,-1,0];[1,0,0];[0,0,1]];
%     t = U*W*S*transpose(U);

    addpath( 'all_solns/02_detect_describe_match');
    addpath( 'all_solns/04_8point/8point');
    addpath( 'all_solns/04_8point');
    addpath( 'all_solns/04_8point/triangulation')
    harris_patch_size = 9;
    harris_kappa = 0.08;
    num_keypoints = 800;
    nonmaximum_supression_radius = 8;
    descriptor_radius = 9;
    match_lambda = 4;
    ransac_iteration = 1000; % obtained by trial and error, can be changed
    number_of_points = 8; % fixed value
    pixel_threshold = 0.15; % obtained by trial and error, can be changed
    max_inliers = 0; % just a pre allocation, is necessary
    harris_scores = harris(img1, harris_patch_size, harris_kappa);
    keypoints_1 = selectKeypoints(harris_scores, num_keypoints, ...
                                nonmaximum_supression_radius);
    descriptors = describeKeypoints(img1, keypoints_1, descriptor_radius);
    harris_scores_2 = harris(img2, harris_patch_size, harris_kappa);
    keypoints_2 = selectKeypoints(harris_scores_2, num_keypoints, ...
                                  nonmaximum_supression_radius);
    descriptors_2 = describeKeypoints(img2, keypoints_2, descriptor_radius);
    matches = matchDescriptors(descriptors_2, descriptors, match_lambda);
    NumMatches = sum( matches ~= 0 );
    kp_hom_coord_1 = [ keypoints_1; ones( 1, num_keypoints) ];
    kp_hom_coord_2 = [ keypoints_2; ones( 1, num_keypoints) ];
    p_1 = kp_hom_coord_1( :, matches( matches ~= 0));
    p_2 = kp_hom_coord_2( :, matches ~= 0);

    for iterate = 1:ransac_iteration
        idx = datasample( 1:NumMatches, number_of_points, 2, 'Replace', ...
                                   false);
        F = fundamentalEightPoint_normalized( p_1( :, idx), ...
                                              p_2( :, idx) );
        homog_points = [p_1 , p_2];
        epi_lines = [F.'*p_2, F*p_1];

        denom = epi_lines( 1, :).^2 + epi_lines( 2, :).^2;
        cost = sqrt( ( sum( epi_lines.*homog_points, 1).^2 )./denom / NumMatches );
        ncost = cost( 1: NumMatches) + cost( NumMatches + 1: 2*NumMatches);
        inliers = ncost < pixel_threshold;
        if sum( inliers ) > sum( max_inliers )
            max_inliers = inliers;
        end
    end
    p1_in = p_1( :,  max_inliers );
    p2_in = p_2( :,  max_inliers );
    F = fundamentalEightPoint_normalized( p1_in, p2_in );
    E = K'*F*K;
    [U,S,~] = svd(E);
    W = [[0,-1,0];[1,0,0];[0,0,1]];
    t = U*W*S*transpose(U);
    if norm(t)<upper_limit && norm(t) > lower_limit
        a = 0;
    end
end

