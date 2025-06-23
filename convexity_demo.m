% Define rotation settings
fps = 30;
duration = 5;
frames = duration * fps;

%% First Surface: z = (x^2 + y^2)/7
[x1, y1] = meshgrid(-6:0.25:6, -6:0.25:6);
z1 = (x1.^2 + y1.^2)/7;

%% Second Surface: Rotated quartic function
r     = linspace(-5, 5, 300);
theta = linspace(0, 2*pi, 300);
[R, Theta] = meshgrid(r, theta);
X2 = R .* cos(Theta);
Y2 = R .* sin(Theta);
Z2 = (R.^4 - 24*R.^2 + 5)/40;

% Magenta plane data
xlims = [min(X2(:)), max(X2(:))];
ylims = [min(Y2(:)), max(Y2(:))];
[XP, YP] = meshgrid(linspace(xlims(1), xlims(2), 2), linspace(ylims(1), ylims(2), 2));
ZP = -0.5 * ones(size(XP));

%% Create Figure with Subplots
figure

% First plot
subplot(1,2,1)
ax1 = gca;
surf(x1, y1, z1)
colormap(jet)
shading interp
hold on

% Plane at z = 4
z_plane = 4 * ones(size(x1));
surf(x1, y1, z_plane, ...
    'FaceAlpha', 0.4, ...
    'EdgeColor', 'none', ...
    'FaceColor', [0 0 0])

xlabel('x')
ylabel('y')
zlabel('z')
title('z = (x^2 + y^2)/7 (convex)', 'FontSize', 16)
axis equal

% Second plot
subplot(1,2,2)
ax2 = gca;
surf(X2, Y2, Z2)
shading interp
colormap(jet)
hold on

% Magenta rectangle at z = -0.5
surf(XP, YP, ZP, ...
    'FaceColor', [0 0 0], ...
    'FaceAlpha', 0.4, ...
    'EdgeColor', 'none')

xlabel('x')
ylabel('y')
zlabel('z')
title('z= 0.025(x^2+y^2)^2-0.6(x^2+y^2)+0.125 (not convex)', 'FontSize', 16)
axis equal

%% Pause before animation
pause(5)

%% Synchronized rotation
for t = 1:frames
    az = mod((t / frames) * 360, 360);  % azimuth angle
    
    view(ax1, [az 30])
    view(ax2, [az 30])
    
    drawnow
    pause(1/fps)
end


