function dX = pendulum_cart_rate(t, X, params, F)

    % Unpack states
    x      = X(1);
    dx     = X(2);
    theta  = X(3);
    dtheta = X(4);

    % Unpack parameters
    mc = params.m_c;   % cart mass
    mp = params.m_p;   % pendulum mass
    l  = params.l;    % COM length
    I  = params.I;    % inertia about pivot
    g  = 9.81;

    % Mass matrix elements
    Mc  = mc + mp;
    M11 = Mc;
    M12 = -mp*l*cos(theta);
    M21 =  mp*l*cos(theta);
    M22 = I + mp*l^2;

    % Right-hand side
    b1 = mp*l*dtheta^2*sin(theta) + F;
    b2 = -mp*g*l*sin(theta);

    % Determinant
    D = M11*M22 - M12*M21;

    % Solve
    ddx     = ( M22*b1 - M12*b2 ) / D;
    ddtheta = ( -M21*b1 + M11*b2 ) / D;

    % Pack derivative
    dX = [dx ; ddx ; dtheta ; ddtheta];
end
