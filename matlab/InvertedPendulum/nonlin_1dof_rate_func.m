function dXdt = nonlin_1dof_rate_func(t, X, system_params)
    % unpack states
    x = X(1);
    dx = X(2);
    theta = X(3);
    dtheta = X(4);
    
    % define system params
    l = system_params.l;
    g = 9.81;
    tau = system_params.tau; 
    b = system_params.b;
    K = system_params.K;
    X_ref = system_params.X_ref;

    % step function
    %{
    if mod(floor(t/4), 2) == 0
        X_ref = [0; 0; 0; 0];
    else
        X_ref = system_params.X_ref;
    end
    %}

    % simulate angle impulse disturbance
    pulse_period = 4;      % every 4 seconds
    pulse_duration = 0.05; % 50ms
    pulse_magnitude = 5.0;
    
    if mod(t, pulse_period) < pulse_duration
        dist_torque = pulse_magnitude;
    else
        dist_torque = 0;
    end

    % calculate the error between reference and current state
    error = X_ref - X;

    % calculate control input
    u = K * error;
    %u = 30; % for now

    % calculating accelerations
    ddx = -dx/tau + b*u/tau; 
    ddtheta = 1/l * (g*sin(theta) + (-dx/tau + b*u/tau)*cos(theta));

    dXdt = [dx; ddx; dtheta; ddtheta];
end