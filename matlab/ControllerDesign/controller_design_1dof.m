%% Linearization and creation of the 1-DOF 4 state model

% pendulum characteristics
l = 0.4185; % effective length of pendulum (m)
g = 9.8; % gravitational acceleration m/s^2

% motor characteristics
tau = 0.0561; % motor time constant
b = 0.0026; % PWM signal gain

% elements of A and B

% state matrix
A = [0 1 0 0;
    0 -1/tau 0 0;
    0 0 0 1;
    0 -1/(l*tau) g/l 0];

% input vector
B = [0; b/tau; 0; b/(l*tau)];

% output matrix
C = [1 0 0 0; 0 0 1 0];

% feedforward
D = 0;

% create free-response state space model
sys_ol = ss(A, B, C, D);

% verify controllability of the system
controllability_matrix = ctrb(A, B);
rank_controllability = rank(controllability_matrix);

if rank_controllability == length(B)
    fprintf('System is controllable (rank = %d).\n', rank_controllability);
else
    fprintf('System is not controllable (rank = %d).\n', rank_controllability);
end

% verify observability of the system
observability_matrix = obsv(A, C);
rank_observability = rank(observability_matrix);

if rank_observability == length(B)
    fprintf('System is observable (rank = %d).\n', rank_observability);
else
    fprintf('System is not observable (rank = %d).\n', rank_observability);
end

%% Open Loop System Analysis

% open-loop poles
[V, Mp] = eigs(A);

figure();
hold on;
pzplot(sys_ol);
grid on;
title('Pole-Zero Plot for Open Loop System');
hold off;

% impulse response
impulseResponse = impulse(sys_ol);

% plot the impulse response to validate unstable equilibrium
% figure looks correct
figure;
hold on;
plot(impulseResponse);
xlabel('Time (s)');
ylabel('States');
%legend('position', 'velocity', 'angle', 'angular velocity');
title('Impulse Response of the System');
grid on;
hold off;

%% Pole Placement and Closed Loop Analysis

close all;

% dominant poles yield:
% T_s = 3 seconds
% OS% = 10%
re = 1.33;
im = 1.82;

desired_poles = [-re + im*1i, -re - im*1i, -5*re + im*1i, -5*re - im*1i];
K = round(place(A, B, desired_poles), 2);
A_cl = A-B*K;

sys_cl = ss(A_cl, B, C, D);

figure();
hold on;
pzplot(sys_cl);
grid on;
title('Pole-Zero Plot for Open Loop System');
hold off;

% closed-loop impulse response
[step_cl, t] = impulse(sys_cl);

figure();
hold on;
grid on;
legend;
plot(t, step_cl);
hold off;

% step info
step_info = stepinfo(step_cl, t);

%% LQR Controller Design

x_max = 0.025; % m
dx_max = 0.05; % m/s
theta_max = 0.15; % rad
dtheta_max = 0.15; % rad/s
u_max = 300;

Q = diag([1/x_max^2; 1/dx_max^2; 1/theta_max^2; 1/dtheta_max^2]);
R = 1/u_max^2;

[K_lqr, S, P] = lqr(sys_ol, Q, R);
K_lqr = round(K_lqr, 2);

%% Creation of 5 State Model w/ Integrated position error

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
% output is position and velocity
% C = [1 0 0 0;
%     0 0 1 0];

C = [1 0 0 0];

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
% observability_matrix = obsv(A_aug, C_aug);
% rank_observability = rank(observability_matrix);
% 
% if rank_observability == length(B_aug)
%     fprintf('System is observable (rank = %d).\n', rank_observability);
% else
%     fprintf('System is not observable (rank = %d).\n', rank_observability);
% end

%% Pole placement for 6 state system

close all;

% dominant poles yield:
% T_s = 3 seconds
% OS% = 10%
re = 1.33;
im = 1.82;

desired_poles = [-re + im*1i, -re - im*1i, -3*re + im*1i, -3*re - im*1i, -2];

K_i = round(place(A_aug, B_aug, desired_poles), 2);

disp(K_i)

A_cl = A_aug-B_aug*K_i;

% analyze natural frequencies and mode shapes
[V, W] = eig(A_cl);

sys_cl = ss(A_cl, B_aug, C_aug, D);

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
%theta_step_info = stepinfo(step_cl(:, 3), t);