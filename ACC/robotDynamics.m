function [dx] = robotDynamics( x, u)
    phi = u(1);
    theta=x(3);
    % dx = zeros(3, 1);
    % dx(1) = v * cos(theta);
    % dx(2) = v * sin(theta);
    % dx(3) = omega;
    dx=[cos(theta);
      sin(theta);
   2*tan(phi)];

    y = x;  % assuming full-state measurement
