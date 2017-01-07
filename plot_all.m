function [] = plot_all( image,S1,T, route )
%PLOT_ALL Summary of this function goes here
%   Detailed explanation goes here
%close all;
figure(1)
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
if ~isempty( S1.kp_cand )
    plot(S1.kp_cand(2, :), S1.kp_cand(1, :), 'bo', 'LineWidth', 1);
end

% Makes sure that plots refresh.    
pause(0.01);
subplot(2,2,2)
hold on;
%location = -T(1:3, 1:3)'*T(1:3, 4);
location = T(1:3, 4);
cam = plotCamera('Location',location','Orientation',T(1:3, 1:3),'Opacity',0, 'Size', 0.5);
axis equal
view([0 0]);

subplot(2,2,3)
hold on;
pos = T(1:3,4);
%plot3(pos(1),pos(2),pos(3),20,'x')
%cam = plotCamera('Location',T(1:3, 4)','Orientation',T(1:3, 1:3),'Opacity',0, 'Size', 4);
%plotCoordinateFrame(T(1:3, 1:3),T(1:3, 4), 100);
%plot3(S1.p3D(1,:),S1.p3D(2,:),S1.p3D(3,:),2,'o')
scatter3(S1.p3D(3,S1.corr),S1.p3D(2,S1.corr),S1.p3D(1,S1.corr),2,'o')
b = 20;
bz = [-5, 130];
axis([bz, -b, b, -b, b]);
%axis equal
rotate3d on;
grid

subplot(2,2,4);
% plot3(route(3,:), route(2,:), route(1,:));
scatter3(S1.p3D(1,S1.corr),S1.p3D(2,S1.corr),S1.p3D(3,S1.corr))
hold on
axis equal;
b = 10;
bz = [-5, 50];
axis([-b, b, -b, b, bz]);
cam = plotCamera('Location',T(1:3, 4)','Orientation',T(1:3, 1:3),'Opacity',0, 'Size', 2);
end

