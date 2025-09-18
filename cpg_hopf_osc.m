close all;
clear all;
clc;
% --- Setup ---
L_thigh = 1.0;
L_shank = 1.0;
L_foot = 0.2;
t_sim = linspace(0, 10, 500);
theta_hip1_log = zeros(size(t_sim));  % same size as time vector
theta_knee1_log = zeros(size(t_sim));  % same size
theta_ankle1_log = zeros(size(t_sim));  % same size
dt = t_sim(2) - t_sim(1);

% Joint angle limits
hip_min = -30 * pi / 180; hip_max = 100 * pi / 180;
knee_min = 5 * pi / 180;  knee_max = 150 * pi / 180;
ankle_min = -15 * pi / 180; ankle_max = 15 * pi / 180;

% Hopf oscillator parameters
mu = 0.8; omega = 1.1 * pi * 1; gamma = 0.1; % gamma = coupling strength

% 6 oscillators: hip1, knee1, ankle1, hip2, knee2, ankle2
z = ones(6, 1) + 1i * ones(6, 1);
z(4:6) = z(4:6) * exp(1i * pi);  % Phase-shift second leg by 180 degrees
z(2) = z(2) * exp(1.2i * pi/2);
z(5) = z(5) * exp(1.2i * pi/2);

z(3) = z(3) * exp(-1.2i * pi/2);
z(6) = z(6) * exp(-1.2i * pi/2);

% Phase-coupling matrix (opposing legs get negative influence)
C = [ 0  0  0  1  0  0;
      0  0  0  0  1  0;
      0  0  0  0  0  1;
      1  0  0  0  0  0;
      0  1  0  0  0  0;
      0  0  1  0  0  0];

% Initialize figure
figure;
hold on; grid on;
xlim([-2, 2]); ylim([-2, 2]); zlim([-2.5, 2.5]);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D Walking Simulation with Hopf Oscillator CPG');
view(3);

filename = 'hopf_gait.gif';

% Phase for ankle matching
amp_ankle = pi/36;
phase_offset = 13 * pi / 6;

% --- Simulation Loop ---
for i = 1:length(t_sim)
    % Update each Hopf oscillator
    z_new = z;
    for j = 1:6
        coupling = sum(C(j,:) .* z.');
        dz = (mu - abs(z(j))^2) * z(j) + 1i * omega * z(j) + gamma * coupling;
        z_new(j) = z(j) + dt * dz;
    end
    z = z_new;

    % Use real part as normalized joint angle signal
    theta_hip1 = -1.1*pi /18 * real(z(1));
    theta_knee1 = 7*pi / 20 * real(z(2));
    % theta_ankle1 = 1*pi/36 * real(z(3));
    theta_hip2 = -1.1*pi /18 * real(z(4));
    theta_knee2 = 7*pi / 20 * real(z(5));
    % theta_ankle2 = 1*pi/36 * real(z(6));

    % Ankle1 matching 
    base = amp_ankle * real(z(3) * exp(1i * phase_offset));
    base2 = amp_ankle * real(z(6) * exp(1i * phase_offset));
    harm = (amp_ankle/0.8) * real((z(3)^2) * exp(1i * (phase_offset + pi*0.6)));
    harm2 = (amp_ankle/0.8) * real((z(6)^2) * exp(1i * (phase_offset + pi*0.6)));
    theta_ankle1 = base + harm;
    theta_ankle2 = base2 + harm2;

    % Clamp joint limits
    theta_hip1 = max(min(theta_hip1, hip_max), hip_min);
    theta_knee1 = max(min(theta_knee1, knee_max), knee_min)+max(min(7*pi / 120 * real(z(2)* exp(0.9i * pi)), knee_max), 0);
    theta_ankle1 = max(min(theta_ankle1, ankle_max), ankle_min);

    theta_hip2 = max(min(theta_hip2, hip_max), hip_min);
    theta_knee2 = max(min(theta_knee2, knee_max), knee_min)+max(min(7*pi / 120 * real(z(5)* exp(0.9i * pi)), knee_max), 0);
    theta_ankle2 = max(min(theta_ankle2, ankle_max), ankle_min);

    % --- Forward Kinematics ---
    x_hip1 = 0; y_hip1 = 0; z_hip1 = 0;
    x_knee1 = x_hip1 + L_thigh * sin(theta_hip1);
    z_knee1 = z_hip1 - L_thigh * cos(theta_hip1); 
    y_knee1 = y_hip1;
    x_ankle1 = x_knee1 + L_shank * sin(theta_hip1 + theta_knee1);
    z_ankle1 = z_knee1 - L_shank * cos(theta_hip1 + theta_knee1); 
    y_ankle1 = y_knee1;
    x_foot1 = x_ankle1 - L_foot * cos(theta_hip1 + theta_knee1 + theta_ankle1);
    z_foot1 = z_ankle1 - L_foot * sin(theta_hip1 + theta_knee1 + theta_ankle1); 
    y_foot1 = y_ankle1;
   

    x_hip2 = 0; y_hip2 = 0.5; z_hip2 = 0;
    x_knee2 = x_hip2 + L_thigh * sin(theta_hip2);
    z_knee2 = z_hip2 - L_thigh * cos(theta_hip2); 
    y_knee2 = y_hip2;
    x_ankle2 = x_knee2 + L_shank * sin(theta_hip2 + theta_knee2);
    z_ankle2 = z_knee2 - L_shank * cos(theta_hip2 + theta_knee2); 
    y_ankle2 = y_knee2;
    x_foot2 = x_ankle2 - L_foot * cos(theta_hip2 + theta_knee2 + theta_ankle2);
    z_foot2 = z_ankle2 - L_foot * sin(theta_hip2 + theta_knee2 + theta_ankle2); 
    y_foot2 = y_ankle2;

    theta_hip1_log(i) = rad2deg(theta_hip1);
    theta_knee1_log(i) = rad2deg(theta_knee1 + theta_hip1);
    theta_ankle1_log(i) = rad2deg(theta_ankle1 );

    % --- Draw Model ---
    cla;
    fill3([x_hip1, x_hip2, x_hip2, x_hip1], ...
          [y_hip1, y_hip2, y_hip2, y_hip1], ...
          [-z_hip1, -z_hip2, -(z_hip2 - 1), -(z_hip1 - 1)], ...
          'k', 'FaceAlpha', 0.5);

    plot3([x_hip1, x_knee1], [y_hip1, y_knee1], [z_hip1, z_knee1], 'r-', 'LineWidth', 2);
    plot3([x_knee1, x_ankle1], [y_knee1, y_ankle1], [z_knee1, z_ankle1], 'b-', 'LineWidth', 2);
    plot3([x_ankle1, x_foot1], [y_ankle1, y_foot1], [z_ankle1, z_foot1], 'g-', 'LineWidth', 2);
    scatter3([x_hip1, x_knee1, x_ankle1, x_foot1], [y_hip1, y_knee1, y_ankle1, y_foot1], [z_hip1, z_knee1, z_ankle1, z_foot1], 50, 'k', 'filled');

    plot3([x_hip2, x_knee2], [y_hip2, y_knee2], [z_hip2, z_knee2], 'r-', 'LineWidth', 2);
    plot3([x_knee2, x_ankle2], [y_knee2, y_ankle2], [z_knee2, z_ankle2], 'b-', 'LineWidth', 2);
    plot3([x_ankle2, x_foot2], [y_ankle2, y_foot2], [z_ankle2, z_foot2], 'g-', 'LineWidth', 2);
    scatter3([x_hip2, x_knee2, x_ankle2, x_foot2], [y_hip2, y_knee2, y_ankle2, y_foot2], [z_hip2, z_knee2, z_ankle2, z_foot2], 50, 'k', 'filled');

    pause(0.01);
end

mask_sim = (t_sim >= 5) & (t_sim <= 15);
t_window = t_sim(mask_sim) - 5;

% --- Plot Result ---
figure;
set(gcf, 'Position', [100, 100, 1200, 500]);  % Set figure size

% --- Subplot 1: Left Knee ---
subplot(3,1,1);
plot(t_window, theta_knee1_log(mask_sim), 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Knee Angle (deg)');
title('Left Knee (knee1)');
grid on;
pbaspect([10 1 1]);

% --- Subplot 2: Left Hip ---
subplot(3,1,2);
plot(t_window, theta_hip1_log(mask_sim), 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Hip Angle (deg)');
title('left hip (hip1)');
grid on;
pbaspect([10 1 1]);

% --- Subplot 3: Left ankle ---
subplot(3,1,3);
plot(t_window, theta_ankle1_log(mask_sim), 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Knee Angle (deg)');
title('left knee');
grid on;
pbaspect([10 1 1]);