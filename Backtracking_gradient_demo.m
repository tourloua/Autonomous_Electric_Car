% Define a grid with x and y in radians (zoomed in)
[x, y] = meshgrid(linspace(-0.4*pi, 0.4*pi, 100));
f = -(cos(x).^2 + cos(y).^2).^2;

% Plot the full surface with gridlines on the surface
figure;
surf(x, y, f, 'EdgeColor', [0.3 0.3 0.3]);  % gray gridlines
colormap jet;
shading faceted;  % keep mesh visible

% Labels and view
xlabel('x (radians)');
ylabel('y (radians)');
zlabel('f(x,y)');
title('f(x,y) = -(cos^2 x + cos^2 y)^2', 'FontSize', 22);
view(45, 30);
axis tight;
grid on;

% Pause for 5 seconds
pause(10);

% Rotate once around z-axis in 5 seconds
frames = 100;
for k = 1:frames
    view(45 + 360 * k / frames, 30);
    drawnow;
    pause(5 / frames);
end

% Plot only the part where x > 0
figure;

% Create mask for x > 0
mask = x > 0;
x_pos = x;
y_pos = y;
f_pos = f;

% Set values outside x > 0 to NaN
x_pos(~mask) = NaN;
y_pos(~mask) = NaN;
f_pos(~mask) = NaN;

% Plot the masked surface with visible gridlines
surf(x_pos, y_pos, f_pos, 'EdgeColor', [0.3 0.3 0.3]);
colormap jet;
shading faceted;

% Labels and view
xlabel('x (radians)');
ylabel('y (radians)');
zlabel('f(x,y)');
title('f(x,y) = -(cos^2 x + cos^2 y)^2 Cross Section', 'FontSize', 22);
view(-67, 30);
axis tight;
grid on;
