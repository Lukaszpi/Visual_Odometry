function S0 = initialisation2( img0, img1, K )
addpath( 'all_solns/02_detect_describe_match');
addpath( 'all_solns/04_8point/8point');
addpath( 'all_solns/04_8point');
addpath( 'all_solns/04_8point/triangulation')
addpath( 'all_solns/03_stero')
disp = getDisparity(img0,img1,5,5,50);
points = disparityToPointCloud(disp,K,
p2_in = points;
S0.kp = [p2_in(:,1)';p2_in(:,2)';ones(1,length(p2_in(:,2)'))];
S0.p3D = worldPoints';
S0.corr = 1:size(p2_in', 2);
% the following matrices are used for extracting new landmarks
S0.T_cand = []; % keep track of transformation matrices corresponding to tracked keypoints
S0.kp_cand0 = []; % store coordinates of keypoints in their first frame
S0.kp_cand = []; % store coordinates of keypoints in this frame

end
