function run_sim_1dof(K, record_status)
    
    arguments
        K = [-330.68, -728.98, 2.2655e+03, 452.26, 0];
        % default option is to not record
        record_status = false;
    end

    clc;
    close all;
    
    system_params.l = 0.4185; % pendulum COM length (m)
    system_params.tau = 0.161; % motor time constant (s)
    system_params.b = 0.00265; % PWM signal gain
    
    % choose between gain matrix
    %system_params.K = [-6, -4.75, 25, 4]; % hand-tuned gain matrix
    %system_params.K = [0, -5, 25, 4]; % experimental gain matrix
    %system_params.K = [-7.6, -6.4, 31.6, 6.1]; % gain matrix via pole placement
    %system_params.K = [-7.86, -5.1, 25, 4.18, 0]; experimental gain matrix
    system_params.K = K;
    
    % define reference output
    system_params.r = 0.5; % scalar
    
    % initial conditions
    tspan = [0 20];
    x_i = 0;
    xdot_i = 0;
    theta_i = deg2rad(0);
    thetadot_i = deg2rad(0);
    z_i = 0;
    % x0 = [x_i; xdot_i; theta_i; thetadot_i];
    x0 = [x_i; xdot_i; theta_i; thetadot_i; z_i];
    
    % solve
    options = odeset('MaxStep', 0.01);
    [tlist, xlist] = ode45(@(t, x) nonlin_1dof_rate_func_v2(t, x, system_params), tspan, x0, options);
    
    % for readability
    x = xlist(:,1);
    dx = xlist(:,2);
    theta = xlist(:,3);
    dtheta = xlist(:,4);
    % z = xlist(:,5);

    % plot result
    figure();
    plot(tlist, x, 'DisplayName', 'Position (m)');
    hold on;
    plot(tlist, dx, 'DisplayName', 'Velocity (m/s)');
    plot(tlist, theta, 'DisplayName', 'Theta (rad)');
    plot(tlist, dtheta, 'DisplayName', 'Angular Rate (rad/s)');
    % plot(tlist, z, 'DisplayName', 'Integrated Position Error');
    legend show;
    xlabel('Time (s)');
    ylabel('State Variables');
    title('Pendulum State Evolution Over Time');
    grid on;
    hold off;
    
    animate_cart_pendulum(tlist, xlist, system_params, record_status);
    
    % disp step response characteristics for position
    disp(stepinfo(xlist(:, 1), tlist));
    % disp step response characteristics for angle
    %disp(stepinfo(xlist(:, 3), tlist));

end