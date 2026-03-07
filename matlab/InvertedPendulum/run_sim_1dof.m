function run_sim_1dof(K, record_status)
    
    arguments
        K = [-7.86, -5.1, 25, 4.18, 5.13];
        % default option is to not record
        record_status = false;
    end

    clc;
    
    system_params.l = 0.4185; % pendulum COM length (m)
    system_params.tau = 0.161; % motor time constant (s)
    system_params.b = 0.00265; % PWM signal gain
    
    % choose between gain matrix
    %system_params.K = [-6, -4.75, 25, 4]; % hand-tuned gain matrix
    %system_params.K = [0, -5, 25, 4]; % experimental gain matrix
    %system_params.K = [-7.6, -6.4, 31.6, 6.1]; % gain matrix via pole placement
    %system_params.K = [-7.86, -5.1, 25, 4.18, 0]; experimental gain matrix
    system_params.K = K; % gain matrix w/ integrated x error
    
    
    % choose bewteen reference matrix
    system_params.X_ref = [0.5; 0; 0; 0]; % 4 state reference vector
    
    % initial conditions
    tspan = [0 10];
    x_i = 0;
    xdot_i = 0;
    theta_i = deg2rad(0);
    thetadot_i = deg2rad(0);
    z_i = 0;
    x0 = [x_i; xdot_i; theta_i; thetadot_i];
    %x0 = [x_i; xdot_i; theta_i; thetadot_i; z_i];
    
    % solve
    options = odeset('MaxStep', 0.01);
    [tlist, xlist] = ode45(@(t, x) nonlin_1dof_rate_func(t, x, system_params), tspan, x0, options);
    
    % for readability
    x = xlist(:,1);
    dx = xlist(:,2);
    theta = xlist(:,3);
    dtheta = xlist(:,4);
    %z = xlist(:,5);

    % calculating actuation effort at each timestep
    u_effort = K(1)*(system_params.X_ref(1) - x) + ...
           K(2)*(system_params.X_ref(2) - dx) + ...
           K(3)*(system_params.X_ref(3) - theta) + ...
           K(4)*(system_params.X_ref(4) - dtheta);

    % plot result
    figure();
    plot(tlist, x, 'DisplayName', 'Position (m)');
    hold on;
    plot(tlist, dx, 'DisplayName', 'Velocity (m/s)');
    plot(tlist, theta, 'DisplayName', 'Theta (rad)');
    plot(tlist, dtheta, 'DisplayName', 'Angular Rate (rad/s)');
    legend show;
    xlabel('Time (s)');
    ylabel('State Variables');
    title('Pendulum State Evolution Over Time');
    grid on;
    hold off;
    
    figure();
    plot(tlist, u_effort);
    hold on;
    xlabel('Time (s)');
    ylabel('Actuation Effort (N)');
    title('Actuation Effort Over Time');
    grid on;
    hold off;
    
    animate_cart_pendulum(tlist, xlist, system_params, record_status);
    
    % disp step response characteristics for position
    disp(stepinfo(xlist(:, 1), tlist));
    % disp step response characteristics for angle
    %disp(stepinfo(xlist(:, 3), tlist));

end