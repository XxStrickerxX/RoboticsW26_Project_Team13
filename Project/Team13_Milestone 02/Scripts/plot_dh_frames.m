% PLOT_DH_FRAMES  Draws the DH frames (x red, z blue, as in Tutorial 03) of the
% Team 13 arm at a given configuration and saves dh_frames.png. Frame 0 = top of base.

q = [0 0 0 0];                                  % zero (initial) configuration
rows = [q(1)            0.04975 0        pi/2;
        q(2)+pi/2       0       0.140    pi;
        q(3)-0.012434   0       0.132900 pi;
        q(4)-0.082320   0.01226 0.100245 0];

T = eye(4); F = {T};
for i = 1:4
    th = rows(i,1); d = rows(i,2); a = rows(i,3); al = rows(i,4);
    T = T * [cos(th) -sin(th)*cos(al)  sin(th)*sin(al) a*cos(th);
             sin(th)  cos(th)*cos(al) -cos(th)*sin(al) a*sin(th);
             0        sin(al)          cos(al)         d;
             0 0 0 1];
    F{end+1} = T; %#ok<AGROW>
end

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 700 900]);
hold on; axis equal; grid on; L = 0.05;          % axis arrow length (m)
P = cell2mat(cellfun(@(M) M(1:3,4), F, 'UniformOutput', false));
plot3(P(1,:)*1e3, P(2,:)*1e3, P(3,:)*1e3, '-', 'Color', [0.8 0.8 0.8], 'LineWidth', 10);  % links (light, under the arrows)
plot3([0 0], [0 0], [-43.8 0], 'k:', 'LineWidth', 1.5);                      % base below frame 0
names = {'0 (base)', '1', '2', '3', '4 (EE)'};
for i = 1:5
    o = F{i}(1:3,4); x = F{i}(1:3,1); z = F{i}(1:3,3);
    quiver3(o(1)*1e3, o(2)*1e3, o(3)*1e3, L*x(1)*1e3, L*x(2)*1e3, L*x(3)*1e3, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.6);
    quiver3(o(1)*1e3, o(2)*1e3, o(3)*1e3, L*z(1)*1e3, L*z(2)*1e3, L*z(3)*1e3, 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.6);
    text((o(1)+L*x(1))*1e3, (o(2)+L*x(2))*1e3, (o(3)+L*x(3))*1e3, sprintf(' x_{%d}', i-1), 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
    text((o(1)+L*z(1))*1e3, (o(2)+L*z(2))*1e3, (o(3)+L*z(3))*1e3, sprintf(' z_{%d}', i-1), 'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');
    text(o(1)*1e3 - 60, o(2)*1e3, o(3)*1e3, ['Frame ' names{i}], 'FontSize', 10);
end
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title({'DH frame assignment at q = [0 0 0 0]', 'x-axes red, z-axes blue (frame 0 on top of the base)'});
view(35, 15); xlim([-120 120]); ylim([-120 120]); zlim([-50 450]);
exportgraphics(fig, fullfile(fileparts(mfilename('fullpath')), 'dh_frames.png'), 'Resolution', 200);
fprintf('Saved dh_frames.png\n');
