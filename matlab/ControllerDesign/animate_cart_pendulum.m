function animate_cart_pendulum(tlist, xlist, params, record_status)
    % Enhanced cart-pendulum animation with improved aesthetics and functionality
    
    % Unpack params
    l = params.l;
    
    % Create figure
    fig = figure('Color', 'w');
    ax = axes('Parent', fig);
    hold(ax, 'on');
    grid(ax, 'on');
    xlabel(ax, 'Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel(ax, 'Height (m)', 'FontSize', 12, 'FontWeight', 'bold');
    
    % Enhanced visualization settings
    cart_width  = 0.15;
    cart_height = 0.10;
    pend_width  = 0.025;
    
    % Set axis limits with padding
    xmin = min(xlist(:,1)) - 0.5;
    xmax = max(xlist(:,1)) + 0.5;
    ymin = -cart_height - 0.1;
    ymax = l + cart_height/2 + 0.2;
    xlim(ax, [xmin xmax]);
    ylim(ax, [ymin ymax]);
    axis(ax, 'equal');
    
    % Rail on which cart moves
    rail_y = 0;
    rail = line(ax, [xmin-0.5, xmax+0.5], [rail_y, rail_y], ...
                'Color', [0.5 0.5 0.5], 'LineWidth', 3);
    
    % Initialize cart
    cart_x0 = xlist(1,1);
    cart_shadow = rectangle(ax, 'Position', [cart_x0-cart_width/2+0.01, -cart_height/2-0.01, ...
                                             cart_width, cart_height], ...
                            'Curvature', 0.1, ...
                            'FaceColor', [0.7 0.7 0.7 0.3], ...
                            'EdgeColor', 'none');
    
    cart = rectangle(ax, 'Position', [cart_x0-cart_width/2, -cart_height/2, ...
                                      cart_width, cart_height], ...
                     'Curvature', 0.1, ...
                     'FaceColor', [0.2 0.5 0.9], ...
                     'EdgeColor', [0.1 0.3 0.6], ...
                     'LineWidth', 2);

    % Pendulum rod
    pendulum = patch(ax, 'XData', [], 'YData', [], ...
                     'FaceColor', [0.3 0.3 0.3], ...
                     'EdgeColor', [0.1 0.1 0.1], ...
                     'LineWidth', 1.5);
    
    % Pivot point (pin joint)
    pivot = plot(ax, cart_x0, 0, 'o', ...
                'MarkerFaceColor', [0.8 0.2 0.2], ...
                'MarkerEdgeColor', [0.4 0.1 0.1], ...
                'MarkerSize', 10, ...
                'LineWidth', 2);
    
    % Pendulum mass (bob at end)
    px0 = cart_x0 - l*sin(xlist(1,3));
    py0 = l*cos(xlist(1,3));
    bob_radius = 0.04;
    bob = rectangle(ax, 'Position', [px0-bob_radius, py0-bob_radius, ...
                                     2*bob_radius, 2*bob_radius], ...
                    'Curvature', [1 1], ...
                    'FaceColor', [0.9 0.3 0.2], ...
                    'EdgeColor', [0.5 0.1 0.1], ...
                    'LineWidth', 2);
    
    % Create info text box
    info_text = text(ax, 0.02, 0.98, '', ...
                    'Units', 'normalized', ...
                    'VerticalAlignment', 'top', ...
                    'HorizontalAlignment', 'left', ...
                    'FontSize', 10, ...
                    'FontName', 'FixedWidth', ...
                    'BackgroundColor', [1 1 1 0.8], ...
                    'EdgeColor', [0.3 0.3 0.3], ...
                    'Margin', 5);
    
    % Title
    title(ax, 'Inverted Pendulum on Cart', 'FontSize', 14, 'FontWeight', 'bold');
    
    % Animation timing
    fps = 60;
    dt_real = 1/fps;
    
    % Uniformly spaced animation times with interpolation
    t_anim = (tlist(1) : dt_real : tlist(end))';
    x_anim = interp1(tlist, xlist, t_anim, 'pchip'); % Smoother interpolation
    
    % Initialize video if recording
    if record_status
        myVideo = VideoWriter('cart_pendulum_animation', 'MPEG-4');
        myVideo.FrameRate = fps;
        myVideo.Quality = 95;
        open(myVideo);
    end
    
    % Animation loop
    tic;
    for k = 1:length(t_anim)
        % Current state
        x     = x_anim(k, 1);
        dx    = x_anim(k, 2);
        theta = x_anim(k, 3);
        dtheta = x_anim(k, 4);
        
        % Update cart position
        cart.Position(1) = x - cart_width/2;
        cart_shadow.Position(1) = x - cart_width/2 + 0.01;
        
        % Update pivot point
        pivot.XData = x;
        
        % Compute pendulum endpoint
        px = x - l*sin(theta);
        py = l*cos(theta);
        
        % Update bob position
        bob.Position(1) = px - bob_radius;
        bob.Position(2) = py - bob_radius;
        
        % Pendulum rod with tapered width
        % Direction vector
        dx_vec = px - x;
        dy_vec = py - 0;
        L_vec = sqrt(dx_vec^2 + dy_vec^2);
        ux = dx_vec / L_vec;
        uy = dy_vec / L_vec;
        
        % Perpendicular vector
        wx = -uy;
        wy = ux;
        
        % Width
        w_base = pend_width / 2;
        w_tip = pend_width / 2;
        
        % Four corners of trapezoid
        x_corners = [x + w_base*wx, x - w_base*wx, ...
                    px - w_tip*wx, px + w_tip*wx];
        y_corners = [0 + w_base*wy, 0 - w_base*wy, ...
                    py - w_tip*wy, py + w_tip*wy];
        
        % Update pendulum
        pendulum.XData = x_corners;
        pendulum.YData = y_corners;
        
        % Update info text
        info_str = sprintf(['Time:     %6.2f s\n' ...
                           'Pos:      %+6.3f m\n' ...
                           'Vel:      %+6.3f m/s\n' ...
                           'Angle:    %+6.2f°\n' ...
                           'Rate:     %+6.2f°/s'], ...
                          t_anim(k), x, dx, rad2deg(theta), rad2deg(dtheta));
        info_text.String = info_str;
        
        % Draw and capture frame
        drawnow;
        
        if record_status
            frame = getframe(fig);
            writeVideo(myVideo, frame);
        end
        
        % Real-time pacing (compensate for drawing time)
        elapsed = toc;
        target_time = k * dt_real;
        pause_time = max(0, target_time - elapsed);
        if pause_time > 0 && ~record_status
            pause(pause_time);
        end
    end
    
    % Close video if recording
    if record_status
        close(myVideo);
        fprintf('Animation saved to: cart_pendulum_animation.mp4\n');
    end
    
    hold(ax, 'off');
end