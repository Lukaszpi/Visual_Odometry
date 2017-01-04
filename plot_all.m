function [] = plot_all( image,S1,T )
%PLOT_ALL Summary of this function goes here
%   Detailed explanation goes here
figure(2)
%figure('units','normalized','outerposition',[0 0 1 1]);
% get the figure and axes handles
hFig = gcf;
hAx  = gca;
% set the figure to full screen
set(hFig,'units','normalized','outerposition',[0 0 1 1]);
 
subplot(2,2,1)
imshow(image);
hold on;
plot(S1.kp(2, :), S1.kp(1, :), 'rx', 'LineWidth', 2);

% Makes sure that plots refresh.    
pause(0.01);
subplot(2,2,2)
hold on;
%<<<<<<< HEAD
%cam = plotCamera('Location',T(1:3, 4)','Orientation',T(1:3, 1:3),'Opacity',0);
%plot3(S1.p3D(1,:),S1.p3D(2,:),S1.p3D(3,:),'x')
%=======
cam = plotCamera('Location',T(1:3, 4)','Orientation',T(1:3, 1:3),'Opacity',0, 'Size', 0.1);

subplot(2,2,3)
hold on;
pos = T(1:3,4);
%plot3(pos(1),pos(2),pos(3),20,'x')
cam = plotCamera('Location',T(1:3, 4)','Orientation',T(1:3, 1:3),'Opacity',0, 'Size', 1);
%plotCoordinateFrame(T(1:3, 1:3),T(1:3, 4), 100);
%plot3(S1.p3D(1,:),S1.p3D(2,:),S1.p3D(3,:),2,'o')
scatter3(S1.p3D(1,:),S1.p3D(2,:),S1.p3D(3,:),2,'o')
axis equal
rotate3d on;
grid
%>>>>>>> 37cef1f963f5d95b0ce470be6c61ceb665ed53f2

end

