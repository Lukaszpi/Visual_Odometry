function [] = plot_CheckKeyPoints(plot_1, plot_2, plot_3, img1)
%PLOT_ALL Summary of this function goes here
%   Detailed explanation goes here
figure(2)
imshow(img1);
hold on;
plot(plot_1(2, :), plot_1(1, :), 'rx', 'LineWidth', 10);
plot(plot_2(2, :), plot_2(1, :), 'bo', 'LineWidth', 2);
plot(plot_3(2, :), plot_3(1, :), 'g.', 'LineWidth', 2);
pause(0.1);
end

