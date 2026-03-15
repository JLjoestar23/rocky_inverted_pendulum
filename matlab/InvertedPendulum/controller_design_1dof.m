%% Linearization of the 1DOF 4 state model

% pendulum characteristics
l = 0.4185; % effective length of pendulum (m)
g = 9.8; % gravitational acceleration m/s^2

% motor characteristics
tau = 0.161; % motor time constant
b = 0.00265; % PWM signal gain

% elements of A and B

% state matrix
A = [0 1 0 0;
    0 -1/tau 0 0;
    0 0 0 1;
    0 -1/(l*tau) g/l 0];

% input vector
B = [0; b/tau; 0; b/(l*tau)];

% output matrix
C = [0 1 0 0];

% feedforward
D = 0;

% 
N = -inv(C*(A-B*K)^-1*B);

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

desired_poles = [-2+1.28i, -2-1.28i, -4+2.57i, -4-2.57i];
K = place(A, B, desired_poles);
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

%% Linearization of 5 State Model w/ Integrated position error

% pendulum characteristics
l = 0.4185; % effective length of pendulum (m)
g = 9.8; % gravitational acceleration m/s^2

% motor characteristics
tau = 0.161; % motor time constant
b = 0.00265; % PWM signal gain

% elements of A and B

% state matrix
A = [0 1 0 0;
    0 -1/tau 0 0;
    0 0 0 1;
    0 -1/(l*tau) g/l 0];

% input vector
B = [0; b/tau; 0; b/(l*tau)];

% output matrix
%C = [1 0 0 0]; % output is position
C = [0 1 0 0]; % output is velocity

% feedforward
D = 0;

% augmented state matrix and input vector
A_aug = [A zeros(length(A), 1); -C 0];
B_aug = [B; 0];

C_aug = eye(5); % for modeling sake, keep all states visible

% create free-response state space model
sys_ol = ss(A_aug, B_aug, C_aug, D);

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

%% Pole placement for 5 state system

close all;

desired_poles = [-2+1i, -2-1i, -3+2i, -3-2i, -8];
K = place(A_aug, B_aug, desired_poles);
K = round(K, 2);

A_cl = A_aug-B_aug*K;

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