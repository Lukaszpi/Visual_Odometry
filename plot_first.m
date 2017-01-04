function [] = plot_first( image,S1,T )
%PLOT_ALL Summary of this function goes here
%   Detailed explanation goes here

% The initial Matrix
T1 = [1 0 0 0;...
      0 1 0 0;...
      0 0 1 0];

figure(1)
% %figure('units','normalized','outerposition',[0 0 1 1]);
% % get the figure and axes handles
% hFig = gcf;
% hAx  = gca;
% % set the figure to full screen
% set(hFig,'units','normalized','outerposition',[0 0 1 1]);
%  
% subplot(2,2,1)
subplot(2,1,1)
imshow(image);
hold on;
plot(S1.kp(2, :), S1.kp(1, :), 'rx', 'LineWidth', 2);
% 
% % Makes sure that plots refresh.    
% pause(0.01);
% subplot(2,2,2)
% hold on;
% cam = plotCamera('Location',T(1:3, 4)','Orientation',T(1:3, 1:3),'Opacity',0);
% plot3(S1.p3D(1,:),S1.p3D(2,:),S1.p3D(3,:),'x')
% cam = plotCamera('Location',T(1:3, 4)','Orientation',T(1:3, 1:3),'Opacity',0, 'Size', 0.5);
% 
% subplot(2,2,3)
subplot(2,1,2)
hold on;
%pos = T(1:3,4);
%plot3(pos(1),pos(2),pos(3),20,'x')
cam = plotCamera('Location',T(1:3, 4)','Orientation',T(1:3, 1:3),'Opacity',0, 'Size', 2);
cam = plotCamera('Location',T1(1:3, 4)','Orientation',T1(1:3, 1:3),'Opacity',0, 'Size', 2, 'Color', 'g');
%plotCoordinateFrame(T(1:3, 1:3),T(1:3, 4), 100);
%plot3(S1.p3D(1,:),S1.p3D(2,:),S1.p3D(3,:),2,'o')
%S1.kp
%scatter3((S1.kp(2,:)-560)/100,(-S1.kp(1,:)+240)/100,S1.kp(3,:),2, 'x')
%S1.p3D
scatter3(S1.p3D(1,:),S1.p3D(2,:),S1.p3D(3,:),2,'o')
axis equal
rotate3d on;
grid

hold off
end

