function dXdt = nonlin_rate_func(t, X, system_params)
    % unpack states
    x = X(1);
    dx = X(2);
    theta = X(3);
    dtheta = X(4);
    
    % define system params
    m_c = system_params.m_c;
    m_p = system_params.m_p;
    l = system_params.l;
    I = 1/3 * m_p * 2*l^2;
    g = 9.81;
    K = system_params.K;
    X_ref = system_params.X_ref;
    
    % step function
    if t < 2
        X_ref(1) = 0;
    else
        X_ref(1) = 1;
    end

    % calculate the error between reference and current state
    error = X_ref - X;

    % integrate position error
    

    %if abs(X_ref(1) - x)

    % calculate control input
    u = K * error;

    M = [m_c + m_p, -m_p*l*cos(theta); 
         -m_p*l*cos(theta), I + m_p*l^2];
    B = [-m_p*l*dtheta^2*sin(theta); m_p*g*l*sin(theta)]; % free response
    B_force = [-m_p*l*dtheta^2*sin(theta) + u; m_p*g*l*sin(theta)]; % feedback response

    % calculate accelerations using the inverse of the mass matrix
    accels = M \ B_force;

    % assign calculated values to the output derivatives
    ddx = accels(1); 
    ddtheta = accels(2);

    dXdt = [dx; ddx; dtheta; ddtheta];
end