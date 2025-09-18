clc;
close all
clear all
L_thigh = 1.0; % Length of the thigh link
L_shank = 1.0; % Length of the shank link
L_foot = 0.2;  % Length of the foot link
frequency_hip = 0.6; % Hip frequency in Hz
frequency_knee = 0.6; % Knee frequency in Hz
frequency_ankle = 1.2; % Ankle frequency in Hz
amplitude_hip = 1.1*pi /18; % Amplitude of hip oscillation 
amplitude_knee = 7*pi / 20; % Amplitude of knee oscillation 
amplitude_ankle = pi / 36; % Amplitude of ankle oscillation  
phase_offsets = [ 0, 13*pi / 20, 5*pi/6]; % Phase offsets for hip

t_sim = linspace(0, 10, 500); % Simulation time array
theta_hip1_log = zeros(size(t_sim));  % same size as time vector
theta_knee1_log = zeros(size(t_sim));  % same size as time vector
theta_ankle1_log = zeros(size(t_sim));  % same size as time vector

% Phase offset for the second leg (opposite phase for walking motion)
phase_offset_leg2 = pi;

% Initialize figure
figure;
hold on;
grid on;
xlim([-2, 2]);
zlim([-2.5, 2.5]); % Z-axis is now the "downward vertical" axis
ylim([-2, 2]); % X-axis range for the rectangle and legs
xlabel('X (Horizontal)');
ylabel('Y (Vertical Down)');
zlabel('Z (Depth)');
title('3D Walking Simulation with Z-X as Down Plane');
view(3); % Set 3D view

% Joint angle limits
hip_min = -30 * pi / 180; % -30 degrees
hip_max = 100 * pi / 180; % 100 degrees
knee_min = 5 * pi / 180; % 5 degrees
knee_max = 150 * pi / 180; % 150 degrees
ankle_min = -10 * pi / 180; % -10 degrees
ankle_max = 10 * pi / 180; % 10 degrees

% Simulation loop
for i = 1:length(t_sim)
    t = t_sim(i); % Current time step
 
    % First leg (leg 1) joint angles
    theta_hip1 = amplitude_hip * sin(2 * pi * frequency_hip * t + phase_offsets(1));
    theta_knee1 = -amplitude_knee * sin(2 * pi * frequency_knee * t + phase_offsets(2)) ;
    theta_ankle1 = amplitude_ankle * sin(2 * pi * frequency_ankle * t + phase_offsets(3))+(-2*amplitude_ankle/2) * sin(pi * frequency_ankle * t + phase_offsets(3) + pi*0.6);
    
    % Second leg (leg 2) joint angles with additional phase offset
    theta_hip2 = amplitude_hip * sin(2 * pi * frequency_hip * t + phase_offsets(1) + phase_offset_leg2);
    theta_knee2 = -amplitude_knee * sin(2 * pi * frequency_knee * t + phase_offsets(2) + phase_offset_leg2);
    theta_ankle2 = -amplitude_ankle * sin(2 * pi * frequency_ankle * t + phase_offsets(3) + phase_offset_leg2)+(-2*amplitude_ankle/3) * sin( pi * frequency_ankle * t+ phase_offset_leg2+ phase_offsets(3) + pi*0.6);

    % Clamp angles to valid ranges
    theta_hip1 = max(min(theta_hip1, hip_max), hip_min);
    theta_knee1 = max(min(theta_knee1, knee_max), knee_min)+max(min((-amplitude_knee/6) * sin(2 * pi * frequency_knee * t+ pi*1.45),knee_max), 0);
    theta_ankle1 = max(min(theta_ankle1, ankle_max), ankle_min);
    
    theta_hip2 = max(min(theta_hip2, hip_max), hip_min);
    theta_knee2 = max(min(theta_knee2, knee_max), knee_min)+max(min((-amplitude_knee/6) * sin(2 * pi * frequency_knee * t+ phase_offset_leg2+ pi*1.45),knee_max), 0);
    theta_ankle2 = max(min(theta_ankle2, ankle_max), ankle_min);
    
    % Forward kinematics for leg 1
    x_hip1 = 0; y_hip1 = 0; z_hip1 = 0; % Fixed hip joint
    x_knee1 = x_hip1 + L_thigh * sin(theta_hip1);
    z_knee1 = z_hip1 - L_thigh * cos(theta_hip1); % Z-axis is downward
    y_knee1 = y_hip1;
    x_ankle1 = x_knee1 + L_shank * sin(theta_hip1 + theta_knee1);
    z_ankle1 = z_knee1 - L_shank * cos(theta_hip1 + theta_knee1);
    y_ankle1 = y_knee1;
    x_foot1 = x_ankle1 - L_foot * cos(theta_hip1 + theta_knee1 + theta_ankle1);
    z_foot1 = z_ankle1 - L_foot * sin(theta_hip1 + theta_knee1 + theta_ankle1);
    y_foot1 = y_ankle1;
    
    % Forward kinematics for leg 2
    x_hip2 = 0; y_hip2 = 0.5; z_hip2 = 0; % Fixed hip joint with depth offset
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
    theta_knee1_log(i) = rad2deg(theta_knee1);
    theta_ankle1_log(i) = rad2deg(theta_ankle1);

    
    % Draw back rectangle connecting hips
    cla;
    fill3([x_hip1, x_hip2, x_hip2, x_hip1], ...
          [y_hip1, y_hip2, y_hip2, y_hip1], ...
          [-z_hip1, -z_hip2, -(z_hip2 - 1), -(z_hip1 - 1)], ...
          'k', 'FaceAlpha', 0.5); % Back rectangle

    % Plot Leg 1
    plot3([x_hip1, x_knee1], [y_hip1, y_knee1], [z_hip1, z_knee1], 'r-', 'LineWidth', 2);
    plot3([x_knee1, x_ankle1], [y_knee1, y_ankle1], [z_knee1, z_ankle1], 'b-', 'LineWidth', 2);
    plot3([x_ankle1, x_foot1], [y_ankle1, y_foot1], [z_ankle1, z_foot1], 'g-', 'LineWidth', 2);
    scatter3([x_hip1, x_knee1, x_ankle1, x_foot1], [y_hip1, y_knee1, y_ankle1, y_foot1], [z_hip1, z_knee1, z_ankle1, z_foot1], 50, 'k', 'filled');

    % Plot Leg 2
    plot3([x_hip2, x_knee2], [y_hip2, y_knee2], [z_hip2, z_knee2], 'r-', 'LineWidth', 2);
    plot3([x_knee2, x_ankle2], [y_knee2, y_ankle2], [z_knee2, z_ankle2], 'b-', 'LineWidth', 2);
    plot3([x_ankle2, x_foot2], [y_ankle2, y_foot2], [z_ankle2, z_foot2], 'g-', 'LineWidth', 2);
    scatter3([x_hip2, x_knee2, x_ankle2, x_foot2], [y_hip2, y_knee2, y_ankle2, y_foot2], [z_hip2, z_knee2, z_ankle2, z_foot2], 50, 'k', 'filled');

    pause(0.02);
end

% --- Plot Result ---
figure;
set(gcf, 'Position', [100, 100, 1200, 500]);  % Set figure size

% --- Subplot 1: Left Knee ---
subplot(3,1,1);
plot(t_sim, theta_knee1_log, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Knee Angle (deg)');
title('Left Knee (knee1)');
grid on;
pbaspect([10 1 1]);

% --- Subplot 2: left hip ---
subplot(3,1,2);
plot(t_sim, theta_hip1_log, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Hip Angle (deg)');
title('left hip (hip1)');
grid on;
pbaspect([10 1 1]);

% --- Subplot 3: Left ankle ---
subplot(3,1,3);
plot(t_sim, theta_ankle1_log, 'g', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('ankle Angle (deg)');
title('Left ankle ');
grid on;
pbaspect([10 1 1]);