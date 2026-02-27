%% Pendulum State-Space Representation

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


%% Structuring the similarity transform
% Transform original representation into controllable canonical form

Co = ctrb(A,B);     % controllability matrix
rank(Co)            % should be 4

T1 = Co;

T3 = fliplr(eye(4));

p = poly(A);        % [1 b3 b2 b1 b0]
b3 = p(2);
b2 = p(3);
b1 = p(4);
b0 = p(5);

T2 = [1 b3 b2 b1;
      0 1  b3 b2;
      0 0  1  b3;
      0 0  0  1];

T = T1*T2*T3;

Az = T^-1 * A * T
Bz = T^-1 * B

%% Show the characteristic polynomial remains the same between transforms

% convert to symbolic form to avoid precision error
As = sym(A);
Bs = sym(B);
Cos = [Bs, As*Bs, As^2*Bs, As^3*Bs];

% use symbolic characteristic polynomial coefficients
ps = charpoly(As, s); 
coeffs_s = coeffs(ps, s, 'All'); % returns [1, a3, a2, a1, a0] sym

b3 = coeffs_s(2);
b2 = coeffs_s(3);
b1 = coeffs_s(4);
b0 = coeffs_s(5);

T2 = [1 b3 b2 b1;
      0 1  b3 b2;
      0 0  1  b3;
      0 0  0  1];

T3 = fliplr(eye(4));
Ts = Cos * T2 * T3;

K = [k1 k2 k3 k4];
Kz = K * Ts;


Azs = Ts^-1 * As * Ts;
Bzs = Ts^-1 * B;

char_poly = collect(det(s*eye(4) - (As - Bs*K)), s)
char_poly_tf = collect(det(s*eye(4) - (Azs - Bzs*Kz)), s)
