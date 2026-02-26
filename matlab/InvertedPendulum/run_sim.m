function run_sim(record_status)
    
    arguments
        % default option is to not record
        record_status = false;
    end

    system_params.m_c = 0.5; % cart mass (kg)
    system_params.m_p = 0.2; % pendulum mass (kg)
    system_params.l = 0.2; % pendulum COM length (m)
    %system_params.K = [-6, -4.75, 25, 4]; % hand-tuned gain matrix
    %system_params.K = [0, -5, 25, 4]; % experimental gain matrix
    system_params.K = [-7.6, -6.4, 31.6, 6.1]; % gain matrix via pole placement
    system_params.X_ref = [1; 0; 0; 0]; % reference state vector
    
    % initial conditions
    tspan = [0 10];
    x_i = 0;
    xdot_i = 0;
    theta_i = deg2rad(0);
    thetadot_i = deg2rad(0);
    x0 = [x_i; xdot_i; theta_i; thetadot_i];
    
    % solve
    [tlist, xlist] = ode45(@(t, x) nonlin_rate_func(t, x, system_params), tspan, x0);
    
    % initialize animation
    figure();
    plot(tlist, xlist(:,1), 'DisplayName', 'Position');
    hold on;
    plot(tlist, xlist(:,2), 'DisplayName', 'Velocity');
    plot(tlist, xlist(:,3), 'DisplayName', 'Theta');
    plot(tlist, xlist(:,4), 'DisplayName', 'Angular Rate');
    legend show;
    xlabel('Time (s)');
    ylabel('State Variables');
    title('Pendulum State Evolution Over Time');
    grid on;
    hold off;
    
    animate_cart_pendulum(tlist, xlist, system_params, record_status);

end