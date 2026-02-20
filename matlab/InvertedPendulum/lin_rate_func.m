function dXdt = lin_rate_func(t, X, system_params)
    % unpack states
    x = X(1);
    dx = X(2);
    theta = X(3);
    dtheta = X(4);
    
    % define system params
    M = system_params.m_c;
    m = system_params.m_p;
    l = system_params.l;
    I = 1/12 * m * l^2;
    g = 9.81;
    K = system_params.K;
    
    % step function
    if mod(floor(t/4), 2) == 0
        X_ref = [0; 0; 0; 0];
    else
        X_ref = system_params.X_ref;
    end
    
    % calculate the error between reference and current state
    error = X_ref - X;

    % calculate control input
    u = K * error;

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

    % calculate and return state derivatives
    dXdt = A*X + B*u;
end