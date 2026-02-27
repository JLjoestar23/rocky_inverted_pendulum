function dXdt = nonlin_rate_func_v2(t, X, system_params)
    % unpack states
    x = X(1);
    dx = X(2);
    theta = X(3);
    dtheta = X(4);
    z = X(5); % integral state
    
    % define system params
    M = system_params.m_c;
    m = system_params.m_p;
    l = system_params.l;
    I = 1/12 * m * l^2;
    g = 9.81;
    K = system_params.K;
    X_ref = system_params.X_ref;
    
    % step function
    %if mod(floor(t/4), 2) == 0
    %    X_ref = [0; 0; 0; 0];
    %else
    %    X_ref = system_params.X_ref;
    %end

    % calculate the error between reference and current state
    %error = X_ref - X;

    % calculate control input
    u = K(1:4) * (X_ref - X(1:4)) + K(5)*-X(5);

    disturbance = 0; % disturbance onto the cart
    u = u + disturbance;

    M = [M + m, -m*l*cos(theta); 
         -m*l*cos(theta), I + m*l^2];
    B = [-m*l*dtheta^2*sin(theta); m*g*l*sin(theta)]; % free response
    B_force = [-m*l*dtheta^2*sin(theta) + u; m*g*l*sin(theta)]; % feedback response

    % calculate accelerations using the inverse of the mass matrix
    accels = M \ B_force;

    % assign calculated values to the output derivatives
    ddx = accels(1); 
    ddtheta = accels(2);

    % integrate position error
    dz = X_ref(1) - X(1);

    dXdt = [dx; ddx; dtheta; ddtheta; dz];
end