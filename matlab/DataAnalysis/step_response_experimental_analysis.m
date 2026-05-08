%% Create 4-DOF Model w/ Integrated position error

% pendulum characteristics
l = 0.4185; % effective length of pendulum (m)
g = 9.8; % gravitational acceleration m/s^2

% motor characteristics
tau = 0.0561; % motor time constant
b = 0.0026; % PWM signal gain

% state matrix
A = [0 1 0 0;
    0 -1/tau 0 0;
    0 0 0 1;
    0 -1/(l*tau) g/l 0];

% input vector
B = [0; b/tau; 0; b/(l*tau)];

% output matrix
C = [1 0 0 0]; % only position is integrated

% feedforward
D = 0;

% augmented state matrix and input vector
A_aug = [A zeros(size(A,1), size(C, 1)); -C zeros(size(C, 1), size(C, 1))];
B_aug = [B; zeros(size(C,1), 1)];
C_aug = [C zeros(size(C, 1), size(C, 1))];

% create free-response state space model
sys_ol = ss(A_aug, B_aug, C_aug, D);

% verify controllability of the system
controllability_matrix = ctrb(A_aug, B_aug);
rank_controllability = rank(controllability_matrix);

if rank_controllability == length(B_aug)
    fprintf('System is controllable (rank = %d).\n', rank_controllability);
else
    fprintf('System is not controllable (rank = %d).\n', rank_controllability);
end

% verify observability of the system
observability_matrix = obsv(A_aug, C_aug);
rank_observability = rank(observability_matrix);

if rank_observability == length(B_aug)
    fprintf('System is observable (rank = %d).\n', rank_observability);
else
    fprintf('System is not observable (rank = %d).\n', rank_observability);
end

%% Pole placement

close all;

% dominant poles yield:
% T_s = 3 seconds
% OS% = 10%
re = 1.33;
im = 1.82;

desired_poles = [-re + im*1i, -re - im*1i, -3*re + im*1i, -3*re - im*1i, -2];

K_i = round(place(A_aug, B_aug, desired_poles), 2);

A_cl = A_aug-B_aug*K_i;

% analyze natural frequencies and mode shapes
[V, W] = eig(A_cl);

% create system with closed loop state matrix
sys_cl = ss(A_cl, B_aug, C_aug, D);

% pole-zero plot
figure();
hold on;
pzplot(sys_cl);
grid on;
title('Pole-Zero Plot for Open Loop System');
hold off;

% closed-loop impulse response
[step_cl, t] = step(sys_cl, 10);

figure();
hold on;
grid on;
legend;
plot(t, step_cl);
hold off;

% step info
x_step_info = stepinfo(step_cl, t);

%% Run simulation

system_params.l = 0.4185; % pendulum COM length (m)
system_params.tau = 0.0561; % motor time constant (s)
system_params.b = 0.0026; % PWM signal gain
system_params.K = K_i;

% define reference input
system_params.r = 0.5; % scalar

% initial conditions
tspan = [0 8];
x_i = 0;
xdot_i = 0;
theta_i = deg2rad(0);
thetadot_i = deg2rad(0);
z_i = 0;

% initial conditions
x0 = [x_i; xdot_i; theta_i; thetadot_i; z_i];

% solve
options = odeset('MaxStep', 0.01);
[tlist, xlist] = ode45(@(t, x) nonlin_1dof_rate_func_int(t, x, system_params), tspan, x0, options);

% for readability
x = xlist(:,1);
% dx = xlist(:,2);
theta = xlist(:,3);
% dtheta = xlist(:,4);
% z = xlist(:,5);

%% Load experimental data
data = table2array(readtable("step_response_data.csv"));

%% Visualize Data

% organize data by start and end idx
start_idx = 29;
end_idx = int32(8 * 1/Ts + start_idx); % 8 seconds
theta_data = data(start_idx:end_idx, 1);
x_data = data(start_idx:end_idx, 2);

% sampled with a period of 105ms
Ts = 105/1000;
% data time vector
t_data = 0:Ts:(length(data(start_idx:end_idx, 1))-1)*Ts;

% plot result
figure();
subplot(2,1,1);
plot(tlist, x, 'Color', [0.2 0.5 0.9], 'LineWidth', 2);
hold on;
plot(t_data, x_data, '--', 'Color', [0.4 0.5 0.6], 'LineWidth', 2);
title('Time Series of Pendulum States');
ylabel('Position (m)');
legend('Model', 'Experimental', 'Location', 'northwest');
grid on;
hold off;

subplot(2,1,2);
plot(tlist, theta, 'Color', [0.9 0.3 0.2], 'LineWidth', 2);
hold on;
plot(t_data, theta_data, '--', 'Color', [0.7 0.4 0.4], 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Angle (rads)');
legend('Model', 'Experimental', 'Location', 'northwest');
grid on;
hold off;