%% Linearization of 4 state model

M = 0.5; % cart mass (kg)
m = 0.2; % pendulum mass (kg)
l = 0.3; % distance from pivot to pendulum COM (m)
I = 1/12 * m * l^2; % mass moment of inertia about pendulum COM (kg*m^2)
g = 9.8; % gravitational acceleration m/s^2

% elements of A and B
a = (m^2*l^2*g) / (I*(m+M) + m*M*l^2);
b = m*g*l*(m+M) / (I*(m+M) + m*M*l^2);
c = (I+m*l^2) / (I*(m+M) + m*M*l^2);
d = m*l / (I*(m+M) + m*M*l^2);

% state matrix
A = [0 1 0 0;
     0 0 a 0;
     0 0 0 1
     0 0 b 0];

% input vector
B = [0; c; 0; d];

% output matrix
C = eye(4);

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

desired_poles = [-3+2i, -3-2i, -5+1i, -5-1i];
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
[step_cl, t] = step(sys_cl);

figure();
hold on;
grid on;
legend;
plot(t, step_cl);
hold off;

% step info
step = stepinfo(step_cl, t);