function dXdt = nonlin_1dof_rate_func_int(t, X, system_params)
    % unpack states
    x = X(1);
    dx = X(2);
    theta = X(3);
    dtheta = X(4);
    z = X(5); % integral state
    
    % define system params
    l = system_params.l;
    g = 9.81;
    tau = system_params.tau; 
    b = system_params.b;
    K = system_params.K;
    r = system_params.r;

    % step function
    % if mod(floor(t/4), 2) == 0
    %     r = 0;
    % else
    %     r = system_params.r;
    % end

    % ramp function
    % if t > 0
    %     X_ref = [0.5*t; 0; 0; 0];
    % else
    %     X_ref = zeros(4, 1);
    % end

    % simulate impulse disturbance
    pulse_period = 0;      % every X seconds
    pulse_duration = 0;    % 50ms
    pulse_magnitude = -2;
    
    if mod(t, pulse_period) < pulse_duration
        dist_torque = pulse_magnitude;
    else
        dist_torque = 0;
    end

    % calculate control input
    u = K(1:4)*-X(1:4) + K(5)*-z;

    % calculating accelerations
    ddx = -dx/tau + b*u/tau; 
    ddtheta = 1/l * (g*sin(theta) + (-dx/tau + b*u/tau)*cos(theta) + dist_torque);
    
    dz = r-x; % integrate position error
    % dz = r-dx; % integrate velocity error

    dXdt = [dx; ddx; dtheta; ddtheta; dz];
end