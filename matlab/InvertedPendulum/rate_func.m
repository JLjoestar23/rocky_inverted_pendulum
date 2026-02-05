function dXdt = rate_func(t, X, system_params)
    % unpack states
    x = X(1);
    dx = X(2);
    theta = X(3);
    dtheta = X(4);
    
    % define system params
    m_c = system_params.m_c;
    m_p = system_params.m_p;
    l = system_params.l;
    I = system_params.I;
    g = 9.81;
    
    M = [m_c + m_p, -m_p*l*cos(theta); 
         -m_p*l*cos(theta), I + m_p*l^2];
    B = [-m_p*l*dtheta^2*sin(theta) ; m_p*g*l*sin(theta)];

    % calculate accelerations using the inverse of the mass matrix
    accels = M \ B;

    % assign calculated values to the output derivatives
    ddx = accels(1); 
    ddtheta = accels(2);

    dXdt = [dx; ddx; dtheta; ddtheta];
end