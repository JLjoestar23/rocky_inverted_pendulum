function animate_cart_pendulum(tlist, xlist, params)

    % unpack params
    l = 2*params.l;

    % figure
    figure;
    axis equal
    hold on
    grid on
    xlabel('x (m)')
    ylabel('y (m)')
    title('Cart–Pendulum Animation')

    % visualization settings
    cart_width  = 0.3;
    cart_height = 0.15;
    pend_width  = 0.02;

    % set axis limits based on motion
    xmin = min(xlist(:,1)) - 0.5;
    xmax = max(xlist(:,1)) + 0.5;
    ylim([-0.5, 0.5]);
    xlim([xmin xmax]);

    % initialize graphics objects
    cart = rectangle('Position', [xlist(1,1)-cart_width/2, -cart_height/2, cart_width, cart_height], ...
                     'FaceColor', [0 0.4 1]);
    
    pendulum = line([xlist(1,1), xlist(1,1) - l*sin(xlist(1,3))], ...
                    [0, l*cos(xlist(1,3))], ...
                    'LineWidth', 10, 'Color', 'k');

    % Animation loop
    for k = 1:length(tlist)

        x     = xlist(k,1);
        theta = xlist(k,3);

        % Update cart position
        cart.Position = [x - cart_width/2, -cart_height/2, cart_width, cart_height];

        % Compute pendulum endpoint
        px = x - l*sin(theta);
        py = l*cos(theta);

        % Update pendulum line
        pendulum.XData = [x px];
        pendulum.YData = [0 py];

        drawnow;

        pause(0.01);
    end

end