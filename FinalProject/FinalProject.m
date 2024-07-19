clear;
% Initialize bodies
body1 = struct('M', 1.0, 'x', -0.9700043688, 'y', 0.2420875563, 'vx', 0.466203646850, 'vy', 0.432385657300);
body3 = struct('M', 1.0, 'x', 0.9700043688, 'y', -0.2420875388, 'vx', 0.4623462036850, 'vy', 0.432643657300);
body2 = struct('M', 1.0, 'x', 0, 'y', 0, 'vx', -0.93324073887, 'vy', -0.8647314976);

t = 0;

G = 1;
h = 0.0002;
t_final = 20;

% Initialize points
points = [];

% Perform simulation
step = 0;
while t < t_final
    % Calculate slope k1 at starting point
    [k1_3_x, k1_3_y, k1_3_vx, k1_3_vy] = AccelCalcu(body3, body1, body2);
    [k1_1_x, k1_1_y, k1_1_vx, k1_1_vy] = AccelCalcu(body1, body2, body3);
    [k1_2_x, k1_2_y, k1_2_vx, k1_2_vy] = AccelCalcu(body2, body1, body3);

    % Extend slope k1 to midpoint for velocity
    body3_k1_vy = body3.vy + k1_3_y*h/2;
    body1_k1_vy = body1.vy + k1_1_y*h/2;
    body2_k1_vy = body2.vy + k1_2_y*h/2;
    
    
    body3_k1_vx = body3.vx + k1_3_x*h/2;
    body1_k1_vx = body1.vx + k1_1_x*h/2;
    body2_k1_vx = body2.vx + k1_2_x*h/2;

    % Extend slope k1 to midpoint
    body1_k1_x = body1.x + k1_1_vx*h/2;
    body2_k1_x = body2.x + k1_2_vx*h/2;
    body3_k1_x = body3.x + k1_3_vx*h/2;

    body2_k1_y = body2.y + k1_2_vy*h/2;
    body3_k1_y = body3.y + k1_3_vy*h/2;
    body1_k1_y = body1.y + k1_1_vy*h/2;
    

    
    

    

    % Update to midpoint
    body3_t = struct('M', body3.M, 'x', body3_k1_x, 'y', body3_k1_y, 'vx', body3_k1_vx, 'vy', body3_k1_vy);
    body1_t = struct('M', body1.M, 'x', body1_k1_x, 'y', body1_k1_y, 'vx', body1_k1_vx, 'vy', body1_k1_vy);
    body2_t = struct('M', body2.M, 'x', body2_k1_x, 'y', body2_k1_y, 'vx', body2_k1_vx, 'vy', body2_k1_vy);
    

    % Calculate slope k2 at midpoint
    [k2_1_x, k2_1_y, k2_1_vx, k2_1_vy] = AccelCalcu(body1_t, body2_t, body3_t);
    [k2_3_x, k2_3_y, k2_3_vx, k2_3_vy] = AccelCalcu(body3_t, body1_t, body2_t);
    [k2_2_x, k2_2_y, k2_2_vx, k2_2_vy] = AccelCalcu(body2_t, body1_t, body3_t);
    

    % Extend slope k2 to midpoint
    body1_k2_x = body1.x + k2_1_vx*h/2;
    body3_k2_x = body3.x + k2_3_vx*h/2;
    body2_k2_x = body2.x + k2_2_vx*h/2;
    

    body2_k2_y = body2.y + k2_2_vy*h/2;
    body3_k2_y = body3.y + k2_3_vy*h/2;
    body1_k2_y = body1.y + k2_1_vy*h/2;
    

    % Extend slope k2 to midpoint for velocity
    body2_k2_vx = body2.vx + k2_2_x*h/2;
    body3_k2_vx = body3.vx + k2_3_x*h/2;
    body1_k2_vx = body1.vx + k2_1_x*h/2;
    

    body2_k2_vy = body2.vy + k2_2_y*h/2;
    body3_k2_vy = body3.vy + k2_3_y*h/2;
    body1_k2_vy = body1.vy + k2_1_y*h/2;
    

    % Update to midpoint
    body3_t = struct('M', body3.M, 'x', body3_k2_x, 'y', body3_k2_y, 'vx', body3_k2_vx, 'vy', body3_k2_vy);
    body1_t = struct('M', body1.M, 'x', body1_k2_x, 'y', body1_k2_y, 'vx', body1_k2_vx, 'vy', body1_k2_vy);
    body2_t = struct('M', body2.M, 'x', body2_k2_x, 'y', body2_k2_y, 'vx', body2_k2_vx, 'vy', body2_k2_vy);
    

    % Calculate slope k3 at midpoint
    [k3_1_x, k3_1_y, k3_1_vx, k3_1_vy] = AccelCalcu(body1_t, body2_t, body3_t);
    [k3_3_x, k3_3_y, k3_3_vx, k3_3_vy] = AccelCalcu(body3_t, body1_t, body2_t);
    [k3_2_x, k3_2_y, k3_2_vx, k3_2_vy] = AccelCalcu(body2_t, body1_t, body3_t);
    

    % Extend slope k3 to endpoint
    body3_k3_x = body3.x + k3_3_vx*h;
    body1_k3_x = body1.x + k3_1_vx*h;
    body2_k3_x = body2.x + k3_2_vx*h;
    

    body3_k3_y = body3.y + k3_3_vy*h;
    body1_k3_y = body1.y + k3_1_vy*h;
    body2_k3_y = body2.y + k3_2_vy*h;
    

    % Extend slope k3 to endpoint for velocity
    body2_k3_vx = body2.vx + k3_2_x*h;
    body3_k3_vx = body3.vx + k3_3_x*h;
    body1_k3_vx = body1.vx + k3_1_x*h;
    

    body2_k3_vy = body2.vy + k3_2_y*h;
    body3_k3_vy = body3.vy + k3_3_y*h;
    body1_k3_vy = body1.vy + k3_1_y*h;
    

    % Update to endpoint
    body1_t = struct('M', body1.M, 'x', body1_k3_x, 'y', body1_k3_y, 'vx', body1_k3_vx, 'vy', body1_k3_vy);
    body3_t = struct('M', body3.M, 'x', body3_k3_x, 'y', body3_k3_y, 'vx', body3_k3_vx, 'vy', body3_k3_vy);
    body2_t = struct('M', body2.M, 'x', body2_k3_x, 'y', body2_k3_y, 'vx', body2_k3_vx, 'vy', body2_k3_vy);
    
    % Calculate slope k4 at endpoint
    [k4_3_x, k4_3_y, k4_3_vx, k4_3_vy] = AccelCalcu(body3_t, body1_t, body2_t);
    [k4_1_x, k4_1_y, k4_1_vx, k4_1_vy] = AccelCalcu(body1_t, body2_t, body3_t);
    [k4_2_x, k4_2_y, k4_2_vx, k4_2_vy] = AccelCalcu(body2_t, body1_t, body3_t);
    

    % Update to endpoint using weighted slopes
    body1.x = body1.x + (1/6)*(k1_1_vx + 2*(k2_1_vx + k3_1_vx) + k4_1_vx)*h;
    body1.vx = body1.vx + (1/6)*(k1_1_x + 2*(k2_1_x + k3_1_x) + k4_1_x)*h;
    body1.y = body1.y + (1/6)*(k1_1_vy + 2*(k2_1_vy + k3_1_vy) + k4_1_vy)*h;
    
    body1.vy = body1.vy + (1/6)*(k1_1_y + 2*(k2_1_y + k3_1_y) + k4_1_y)*h;

    body2.x = body2.x + (1/6)*(k1_2_vx + 2*(k2_2_vx + k3_2_vx) + k4_2_vx)*h;
    body2.vx = body2.vx + (1/6)*(k1_2_x + 2*(k2_2_x + k3_2_x) + k4_2_x)*h;
    body2.y = body2.y + (1/6)*(k1_2_vy + 2*(k2_2_vy + k3_2_vy) + k4_2_vy)*h;
    
    body2.vy = body2.vy + (1/6)*(k1_2_y + 2*(k2_2_y + k3_2_y) + k4_2_y)*h;

    body3.x = body3.x + (1/6)*(k1_3_vx + 2*(k2_3_vx + k3_3_vx) + k4_3_vx)*h;
    body3.vx = body3.vx + (1/6)*(k1_3_x + 2*(k2_3_x + k3_3_x) + k4_3_x)*h;
    body3.y = body3.y + (1/6)*(k1_3_vy + 2*(k2_3_vy + k3_3_vy) + k4_3_vy)*h;
    
    body3.vy = body3.vy + (1/6)*(k1_3_y + 2*(k2_3_y + k3_3_y) + k4_3_y)*h;

    t = t + h;
    step = step + 1;

    if mod(step, 100) == 0
        points = [points; [body1.x, body1.y, body2.x, body2.y, body3.x, body3.y]];
    end
end

% Plot animated
figure;
hold on;

axis([-2 2 -1.5 1.5]);

for i = 1:size(points, 1)
    
    plot(points(i, 1:2:end), points(i, 2:2:end), 'r*', 'MarkerSize', 12);
    pause(0.05);
    cla;
end
hold off;

function [dv_x, dv_y, vx, vy] = AccelCalcu(b0, b1, b2)
    rdiff_x_1 = b0.x - b1.x;
    rdiff_y_1 = b0.y - b1.y;

    G = 1;
    dv_x = 0.0;
    dv_y = 0.0;
    GM_1 = -G * b1.M;

    

    rcubed_1 = sqrt(rdiff_x_1^2 + rdiff_y_1^2)^3;

    dv_y = dv_y + GM_1 * rdiff_y_1 / rcubed_1;
    dv_x = dv_x + GM_1 * rdiff_x_1 / rcubed_1;
    
    rdiff_x_2 = b0.x - b2.x;
    rdiff_y_2 = b0.y - b2.y;
    GM_2 = -G * b2.M;

    

    rcubed_2 = sqrt(rdiff_x_2^2 + rdiff_y_2^2)^3;

    dv_x = dv_x + GM_2 * rdiff_x_2 / rcubed_2;
    dv_y = dv_y + GM_2 * rdiff_y_2 / rcubed_2;

    vx = b0.vx;
    vy = b0.vy;
end


