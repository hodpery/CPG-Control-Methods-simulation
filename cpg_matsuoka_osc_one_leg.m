% Simplified Matsuoka Gait: Single-Leg Oscillator + Delayed Right Leg
close all; clc;close all;

% --- Setup ---
L_thigh = 1.0;
L_shank = 1.0;
L_foot = 0.2;
space = 15; % simulation time(sec)
t_sim = linspace(0, space, space*60);
dt = t_sim(2) - t_sim(1);


% Oscillator parameters
tau_r = 0.30; tau_a = 0.30;
beta = 2.5; w = 2.0; s = 2.2;
k_leg = 0.1;

% Initial states [u1, u2, v1, v2] for hip, knee, ankle (LEFT LEG ONLY)
osc = repmat([0.5002, 0.1498, 0.4048, 0.0660], 3, 1);
osc(2,:) = [0.6993, -0.0150, 0.3698, 0.2968];
osc(3,:) = [0.5260, 0.1384, 0.4172, 0.0648];
outputs = zeros(3,1);

% Intra-leg coupling matrix (3x3)
Couple = zeros(3);
Couple(1,2) = k_leg; Couple(2,1) = k_leg;
Couple(2,3) = k_leg; Couple(3,2) = k_leg;

% Logging
knee1_log = zeros(size(t_sim));
hip_log = zeros(size(t_sim));
ankle_log = zeros(size(t_sim));

% Buffer for output delay (mirror with time offset)
delay_buffer = zeros(3, length(t_sim));
mod_buffer = zeros(1, length(t_sim));
delay_steps = round(1.4*length(t_sim) / (space*2)); % 180 deg offset
mod_delay_steps = round(1.35 * length(t_sim) / space); % π*1.35 phase offset equivalent


% Initialize figure for animation
figure;
hold on; grid on;
xlim([-2, 2]); ylim([-2, 2]); zlim([-2.5, 2.5]);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Simplified Matsuoka Gait with Offset');
view(3);

% --- Simulation Loop ---
for i = 1:length(t_sim)
    % --- Update LEFT leg oscillator states ---
    osc_new = osc;
    for j = 1:3
        coupling_input = sum(Couple(j,:) .* outputs');
        [osc_new(j,:), outputs(j)] = matsuoka_step(osc(j,:), tau_r, tau_a, beta, w, s, dt, coupling_input);
    end
    osc = osc_new;

    % Store current outputs to buffer
    delay_buffer(:, i) = outputs;
    mod_buffer(i) = outputs(3)^2;

    % Compute left leg angles
    theta_hip1   = 1.1*pi/18 * outputs(1);
    theta_knee1  = 7*pi / 20 * outputs(2);

    idx_mod = mod(i - mod_delay_steps - 1, length(t_sim)) + 1;
    mod_signal = mod_buffer(idx_mod);
    amp_ankle = pi/36;
    base = amp_ankle * outputs(3)-deg2rad(3.2);
    mod_term = ( amp_ankle / 0.3) * mod_signal-deg2rad(3.2);
    theta_ankle1 = max(min(base, deg2rad(10)), deg2rad(-10)) + max(min(mod_term, deg2rad(10)), deg2rad(-10));

    

    % Compute right leg angles from delayed buffer
    idx_delay = mod(i - delay_steps - 1, length(t_sim)) + 1;
    theta_hip2   = 1.1*pi/18 * delay_buffer(1, idx_delay);
    theta_knee2  = 7*pi / 20 * delay_buffer(2, idx_delay);

    base2 = amp_ankle * delay_buffer(3, idx_delay)-deg2rad(3.2);
    idx_mod2 = mod(idx_delay - mod_delay_steps - 1, length(t_sim)) + 1;
    mod_signal2 = mod_buffer(idx_mod2);
    mod_term2 = (amp_ankle / 0.3) * mod_signal2-deg2rad(3.2);
    theta_ankle2 = max(min(base2, deg2rad(10)), deg2rad(-10)) + max(min(mod_term2, deg2rad(10)), deg2rad(-10));

    % Clamp left leg angles
    theta_hip1   = max(min(theta_hip1, deg2rad(100)), deg2rad(-30));
    theta_knee1  = max(min(theta_knee1, deg2rad(150)), deg2rad(5))+max(min(theta_knee2/6, deg2rad(150)), deg2rad(0));

    % Clamp right leg angles
    theta_hip2   = max(min(theta_hip2, deg2rad(100)), deg2rad(-30));
    theta_knee2  = max(min(theta_knee2, deg2rad(150)), deg2rad(5))+max(min(theta_knee1/6, deg2rad(150)), deg2rad(0));
   
    % Log knee angles
    knee1_log(i) = theta_knee1;
    hip_log(i) = theta_hip1;
    ankle_log(i) = theta_ankle1;

    % --- Forward Kinematics ---
    %Left leg
    x_hip1 = 0; y_hip1 = 0; z_hip1 = 0;
    x_knee1 = x_hip1 + L_thigh * sin(theta_hip1);
    z_knee1 = z_hip1 - L_thigh * cos(theta_hip1); y_knee1 = y_hip1;
    x_ankle1 = x_knee1 + L_shank * sin(theta_hip1 + theta_knee1);
    z_ankle1 = z_knee1 - L_shank * cos(theta_hip1 + theta_knee1); y_ankle1 = y_knee1;
    x_foot1 = x_ankle1 - L_foot * cos(theta_hip1 + theta_knee1 + theta_ankle1);
    z_foot1 = z_ankle1 - L_foot * sin(theta_hip1 + theta_knee1 + theta_ankle1); y_foot1 = y_ankle1;

    % Right leg
    x_hip2 = 0; y_hip2 = 0.5; z_hip2 = 0;
    x_knee2 = x_hip2 + L_thigh * sin(theta_hip2);
    z_knee2 = z_hip2 - L_thigh * cos(theta_hip2); y_knee2 = y_hip2;
    x_ankle2 = x_knee2 + L_shank * sin(theta_hip2 + theta_knee2);
    z_ankle2 = z_knee2 - L_shank * cos(theta_hip2 + theta_knee2); y_ankle2 = y_knee2;
    x_foot2 = x_ankle2 - L_foot * cos(theta_hip2 + theta_knee2 + theta_ankle2);
    z_foot2 = z_ankle2 - L_foot * sin(theta_hip2 + theta_knee2 + theta_ankle2); y_foot2 = y_ankle2;

    % --- Draw the legs ---
    cla;
    fill3([x_hip1, x_hip2, x_hip2, x_hip1], [y_hip1, y_hip2, y_hip2, y_hip1], [-z_hip1, -z_hip2, -(z_hip2 - 1), -(z_hip1 - 1)], 'k', 'FaceAlpha', 0.5);
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

% --- Plot Result ---
figure;
set(gcf, 'Position', [100, 100, 1200, 500]);  % Set figure size
mask_sim = (t_sim >= 5) & (t_sim <= 15);
t_window = t_sim(mask_sim) - 5;
% --- Subplot 1: Left Knee ---
subplot(3,1,1);
plot(t_window, rad2deg(knee1_log(mask_sim)), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Knee Angle (deg)');
title('Left Knee (knee1)');
grid on;
pbaspect([10 1 1]);

% --- Subplot 2: Left Hip ---
subplot(3,1,2);
plot(t_window, rad2deg(hip_log(mask_sim)), 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Hip Angle (deg)');
title('left hip (hip1)');
grid on;
pbaspect([10 1 1]);

% --- Subplot 3: Left Ankle ---
subplot(3,1,3);
plot(t_window, rad2deg(ankle_log(mask_sim)), 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Ankle Angle (deg)');
title('left ankle');
grid on;
pbaspect([10 1 1]);

% --- Matsuoka Oscillator Step ---
function [state_next, output] = matsuoka_step(state, tau_r, tau_a, beta, w, s, dt, coupling)
    u1 = state(1); u2 = state(2);
    v1 = state(3); v2 = state(4);
    y1 = max(0, u1); y2 = max(0, u2);
    du1 = (-u1 - w*y2 - beta*v1 + s + coupling) / tau_r;
    du2 = (-u2 - w*y1 - beta*v2 + s + coupling) / tau_r;
    dv1 = (y1 - v1) / tau_a;
    dv2 = (y2 - v2) / tau_a;
    u1 = u1 + dt * du1; u2 = u2 + dt * du2;
    v1 = v1 + dt * dv1; v2 = v2 + dt * dv2;
    state_next = [u1, u2, v1, v2];
    output = y1 - y2;
end
