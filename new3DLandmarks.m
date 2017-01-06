function [S1] = new3DLandmarks(img0, img1, T, K, S1, S0)


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
    pointTracker = vision.PointTracker('BlockSize', [21 21]); 
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
    min_baseline = 1.2;
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
                sum( rot_vec_diff.^2, 1) < 5;

    
    
    % initialize the triangulation with the points that are decided to be
    % well
    kp0_triang = [S1.kp_cand0(:, readyToTr ); ones(1, sum( readyToTr ) )];
    kp1_triang = [S1.kp_cand(:, readyToTr ); ones(1, sum( readyToTr ) )];
    
    kp0_triang_sw = [kp0_triang(2,:); kp0_triang(1,:); kp0_triang(3,:)];
    kp1_triang_sw = [kp1_triang(2,:); kp1_triang(1,:); kp1_triang(3,:)];
    
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
        
        % taken from run_sfm.m start
        T_it = reshape( T_triang(:, i), 4, 4 );
        
        E = estimateEssentialMatrix(kp0_triang_sw,...
            kp1_triang_sw(:, first:last), K, K);

        % Extract the relative camera positions (R,T) from the essential matrix

        % Obtain extrinsic parameters (R,t) from E
        [Rots,u3] = decomposeEssentialMatrix(E);

        % Disambiguate among the four possible configurations
        [R_C2_W,T_C2_W] = disambiguateRelativePose(Rots,u3,...
            kp0_triang_sw(:, first:last),...
            kp1_triang_sw(:, first:last),K,K);

        % Triangulate a point cloud using the final transformation (R,T)
        T1 = eye(3,4);
        T2 = [R_C2_W, T_C2_W];
        M1 = K * T1;
        M2 = K * T2;
        new3D_points = linearTriangulation(kp0_triang_sw(:, first:last),...
            kp1_triang_sw(:, first:last),M1,M2);
        for i=1:
        %new3D_points = new3D_points + repmat(T_it(1:3, 4), 3, size(new3D_points, 2));
        % taken from run_sfm.m end
        
%         % and extract all of these points at once
%         T_it = reshape( T_triang(:, i), 4, 4 );
%         M0 = K*T_it(1:3, :);
%         M1 = K*T(1:3, :);
%         new3D_points = [new3D_points...
%             linearTriangulation( kp0_triang_sw(:, first:last), ...
%             kp1_triang_sw(:, first:last), M0, M1)];
%         % jump to the next section of points
        i = last + 1;
    end
    %%% here a check if point is in front of camera could be added %%%
    
    % update the state space
    S1.corr = [S1.corr  (length(S1.corr) + 1: ...
                         length(S1.corr) + 1 + size(new3D_points, 2) )];
    if ~isempty( new3D_points )
        S1.p3D  = [S1.p3D   new3D_points(1:3, :)];
    end
    S1.kp   = [S1.kp    kp1_triang];
    
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

    