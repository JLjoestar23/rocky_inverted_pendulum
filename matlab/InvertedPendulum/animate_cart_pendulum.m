function animate_cart_pendulum(tlist, xlist, params, record_status)

    % unpack params
    l = 2*params.l;

    % figure
    figure;
    axis equal;
    hold on;
    grid on;
    xlabel('x (m)');
    ylabel('y (m)');

    % visualization settings
    cart_width  = 0.1;
    cart_height = 0.1;
    pend_width  = 0.02;

    % set axis limits based on motion
    xmin = min(xlist(:,1)) - 0.5;
    xmax = max(xlist(:,1)) + 0.5;
    %ylim([-0.5, 0.5]);
    xlim([xmin xmax]);

    % initialize graphics objects
    cart = rectangle('Position', [xlist(1,1)-cart_width/2, -cart_height/2, cart_width, cart_height], ...
                     'FaceColor', [0 0.4 1]);

    %pendulum = line([xlist(1,1), xlist(1,1) - l*sin(xlist(1,3))], ...
    %                [0, l*cos(xlist(1,3))], ...
    %                'LineWidth', 250*pend_width, 'Color', 'k');

    pendulum = patch('XData', [], 'YData', [], ...
                    'FaceColor', 'k', ...
                    'EdgeColor', 'none');


    % adjustments in order to create a real time animation
    fps = 60; % desired frames-per-second
    dt_real = 1/fps; % seconds per frame
    % uniformly spaced animation times
    t_anim = (tlist(1) : dt_real : tlist(end)); 
    x_anim = interp1(tlist, xlist, t_anim, 'linear'); % linear interp

    % initialize video
    if record_status == true
        myVideo = VideoWriter('full-state-feedback'); % open video file
        myVideo.FrameRate = 60;
        open(myVideo)
    end

    % animation loop
    for k = 1:length(t_anim)

        x     = x_anim(k,1);
        theta = x_anim(k,3);

        % update cart position
        cart.Position = [x - cart_width/2, -cart_height/2, cart_width, cart_height];

        % compute pendulum endpoint
        px = x - l*sin(theta);
        py = l*cos(theta);
        
        % direction vector
        dx = px - x;
        dy = py - 0;
        
        % normalize direction
        L = sqrt(dx^2 + dy^2);
        ux = dx / L;
        uy = dy / L;
        
        % perpendicular vector
        wx = -uy;
        wy = ux;
        
        % half width
        w = pend_width / 2;
        
        % four rectangle corners
        x_corners = [x + w*wx, x - w*wx, px - w*wx, px + w*wx];
        
        y_corners = [0 + w*wy, 0 - w*wy, py - w*wy, py + w*wy];
        
        % update
        pendulum.XData = x_corners;
        pendulum.YData = y_corners;

        drawnow;

        title(sprintf('Cart–Pendulum Animation (t = %.2f s)', t_anim(k)));

        if record_status == true
            frame = getframe(gcf); %get frame
            writeVideo(myVideo, frame);
        end

        pause(dt_real/8);
    end

    if record_status == true
        close(myVideo);
    end

end