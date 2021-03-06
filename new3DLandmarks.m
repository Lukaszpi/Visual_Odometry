function [S1] = new3DLandmarks(img0, img1, T, K, S1, S0)


%% find new 3D landmarks 
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
    % is decided. 
    
    % baseline criteria -> not enough if baseline is more or less
    % perpendicular to camera plane
    min_baseline = 0.9;
    baseline = sum( (T(:, 4) - S1.T_cand(13:16, :)).^2, 1);
    
    %%% the rotation matrices are transformed to rotation vectors. this
    %%% allowes a direct comparison (since differences of rot matrices do
    %%% not make sense)
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
    %kp1_triang_sw - kp0_triang_sw
    
    T_triang = S1.T_cand(:, readyToTr);
    i = 1;
    while i <= size(kp1_triang_sw, 2)
        % check for all point that have the same transformation matrix
        % (because they were detected the first time in the same frame)
        
        %T_triang(:, i) == T_triang
        same_frame = find( sum( T_triang(:, i) == T_triang, 1) / 16 == 1 );
        first = same_frame(1);
        last = same_frame(end);
                
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
