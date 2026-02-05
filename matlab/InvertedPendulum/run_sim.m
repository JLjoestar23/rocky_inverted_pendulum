function run_sim()
    system_params.m_c = 0.5;
    system_params.m_p = 0.2;
    system_params.l = 0.2;
    system_params.I = 0.0006;

    tspan = [0 15];
    x0 = [0; 0; 0.1; 0];
    
    [tlist, xlist] = ode45(@(t, x) rate_func(t, x, system_params), tspan, x0);
    
    figure();
    plot(tlist, xlist(:,1), 'DisplayName', 'Position');
    hold on;
    plot(tlist, xlist(:,2), 'DisplayName', 'Velocity');
    plot(tlist, xlist(:,3), 'DisplayName', 'Theta');
    plot(tlist, xlist(:,4), 'DisplayName', 'Angular Rate');
    legend show;
    xlabel('Time (s)');
    ylabel('State Variables');
    grid on;
    hold off;

    animate_cart_pendulum(tlist, xlist, system_params);

end