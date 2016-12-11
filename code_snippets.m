%% using matlab harris %%%%
points1 = detectHarrisFeatures(img1);
points2 = detectHarrisFeatures(img2);

[features1,valid_points1] = extractFeatures(img1,points1);
[features2,valid_points2] = extractFeatures(img2,points2);

indexPairs = matchFeatures(features1,features2);

matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

figure; showMatchedFeatures(img1,img2,matchedPoints1,matchedPoints2);

%% using matlab surf %%%%
points1 = detectSURFFeatures(img1);
points2 = detectSURFFeatures(img2);


[f1,vpts1] = extractFeatures(img1,points1);
[f2,vpts2] = extractFeatures(img2,points2);


indexPairs = matchFeatures(f1,f2) ;
matchedPoints1 = vpts1(indexPairs(:,1));
matchedPoints2 = vpts2(indexPairs(:,2));

figure; showMatchedFeatures(img1,img2,matchedPoints1,matchedPoints2);
legend('matched points 1','matched points 2');

%% adding outliners to matches %%%% 
for i = 1:length(matches)
    if matches(i) == 0 && rand() < 0.05
        t = 0;
        while sum(matches == t) ~= 0
            t = ceil(rand()*length(matches));
        end
        matches(i) = t;
    end  
end
NumMatches = sum( matches ~= 0 );

%% using matlab point tracker
pointTracker = vision.PointTracker('BlockSize', [11 11], ...
                                   'MaxIterations', 50);
initialize(pointTracker, fliplr(kp_init(1:2, :)'),  img1);
[points, point_validity, scores] = step(pointTracker,img2);

kp_new = kp_init(1:2, :)';
points = points(point_validity, :);
imshow(img1);
hold on;
plot(kp_new(:, 2), kp_new(:, 1), 'rx');
plot(points(:, 1), points(:, 2), 'bx');
