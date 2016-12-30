function [] = plot_all( image,S1,T )
%PLOT_ALL Summary of this function goes here
%   Detailed explanation goes here
figure(1)
subplot(2,2,1)
imshow(image);
hold on;
plot(S1.kp(2, :), S1.kp(1, :), 'rx', 'LineWidth', 2);
% Makes sure that plots refresh.    
pause(0.01);
subplot(2,2,2)
hold on;
cam = plotCamera('Location',T(1:3, 4)','Orientation',T(1:3, 1:3),'Opacity',0);

subplot(2,2,3)
hold on;
pos = T(1:3,4);
scatter3(pos(1),pos(2),pos(3),20,'o')
scatter3(S1.p3D(1,:),S1.p3D(2,:),S1.p3D(3,:),2,'x')

end

