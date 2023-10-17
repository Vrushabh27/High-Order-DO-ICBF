nx = 6;  % number of states
nu = 2;  % number of inputs
ny = 6;  % number of outputs

nlobj = nlmpc(nx, ny, nu);

Ts = 0.01;  % sample time
nlobj.Ts = Ts;

nlobj.PredictionHorizon = 5;
nlobj.ControlHorizon = 5;

% nlobj.Model.StateFcn = "robotDynamics";
nlobj.Model.StateFcn = @vehicleDynamics;

% nlobj.Model.OutputFcn = @(t,x,u) x;  % This can remain an anonymous function as it's directly defined
nlobj.Model.IsContinuousTime = true;

% nlobj.Model.OutputFcn = @(t,x,u) x;
% nlobj.Model.NumberOfParameters = 1;

% Define constraints if needed (e.g. maximum and minimum speeds, etc.)
% For instance:
% nlobj.MV(1).Min = -2;  % minimum v
% nlobj.MV(1).Max = 2;   % maximum v
% nlobj.MV(2).Min = -pi/4;  % minimum omega
% nlobj.MV(2).Max = pi/4;   % maximum omega

% Define cost function weights
nlobj.Weights.OutputVariables = [1, 1,0.2,1,1, 0.2];  % You can change these weights according to your needs
nlobj.Weights.ManipulatedVariablesRate = [0.1, 0.1];  % Weights on input rate changes
% Initial condition
x0 = [-5; 0; 0; 0;0;0 ];
u0 = [0, 0]; % Initial guess for the input
validateFcns(nlobj, x0, u0);

% Define a reference trajectory
ref = [5, 0, 0, 0, 0,0 ]; % For instance, to drive the robot to the origin with zero theta
xCurrent = x0;  % Initial state
u0 = [0.1, 0.1]; % Initial guess for the input
xHistory = x0';  % to save state trajectory
uHistory = [];   % to save control input trajectory

% Compute the control actions using the NMPC
% [x, u, y, t] = sim(nlobj, x0, u0, ref);
N = 500; % total number of time steps, you can adjust this as needed
for k = 1:N
    % Compute control action using nlmpcmove
    [mv, ~] = nlmpcmove(nlobj, xCurrent, u0, ref);
    
    % Save control and state
    uHistory = [uHistory; mv'];
    
    % Integrate system dynamics over one time step using ODE solver (e.g., ode45)
    [~, y] = ode45(@(t, x) vehicleDynamics1(t, x, mv), [0 Ts], xCurrent);
    
    % Update current state
    xCurrent = y(end, :)';
    
    % Save state history
    xHistory = [xHistory; xCurrent'];
end



t = (0:N) * Ts;

% Plot states
figure;
subplot(3,1,1);
plot(t, xHistory(:,1));
xlabel('Time (s)');
ylabel('x');
title('State x');

subplot(3,1,2);
plot(t, xHistory(:,2));
xlabel('Time (s)');
ylabel('y');
title('State y');

subplot(3,1,3);
plot(t, xHistory(:,3));
xlabel('Time (s)');
ylabel('\theta');
title('State \theta');

% Plot control inputs
figure;
subplot(2,1,1);
stairs(t(1:end-1), uHistory(:,1),"LineWidth",2);
xlabel('Time (s)');
ylabel('v');
title('Control input v');

subplot(2,1,2);
stairs(t(1:end-1), uHistory(:,2),"LineWidth",2);
xlabel('Time (s)');
ylabel('\omega');
title('Control input \omega');

% Plot 2D trajectory
figure;
ylim([0,3]);hold on;
plot(xHistory(:,1), xHistory(:,2), '-o',"LineWidth",2);
ylim([0,3]);
xlabel('x');
ylabel('y');
ylim([0,3]);
title('2D Trajectory');
% Nonlinear Model Function
function dxdt = vehicleDynamics( x, u)
m = 1500; % Mass of the vehicle (kg)
Iz = 2667; % Inertia about z-axis (kg.m^2)
Lf = 1.2; % Distance from CG to front axle (m)
Lr = 1.6; % Distance from CG to rear axle (m)
Rw = 0.3; % Wheel radius (m)
Tmax = 500; % Maximum torque (Nm)
Kf = 50000; % Front tire stiffness (N/rad)
Kr = 60000; % Rear tire stiffness (N/rad)
Cr0 = 0.01; % Roll resistance coefficient
Cr1 = 0.005; % Air resistance coefficient
    % Extract states and inputs
    X = x(1); Y = x(2); psi = x(3); vx = x(4); vy = x(5); omega = x(6);
    tr = u(1); delta = u(2);
    
    % Tire forces
    Fxf = (tr * Tmax) / (2 * Rw);
    Fxr = Fxf;
    alpha_f = -atan((omega * Lf + vy) / (vx+0.00001)) + delta;
    alpha_r = atan((omega * Lr - vy) / (0.00001+vx));
    Fyf = Kf * alpha_f;
    Fyr = Kr * alpha_r;
    
    % Drag force
    Fdrag = Cr0 + Cr1 * vx^2;
    
    % Vehicle dynamics equations
    dxdt = zeros(6, 1);
    dxdt(1) = vx * cos(psi) - vy * sin(psi);
    dxdt(2) = vx * sin(psi) + vy * cos(psi);
    dxdt(3) = omega;
    dxdt(4) = (Fxf * cos(delta) - Fyf * sin(delta) + Fxr - Fdrag + m * vy * omega) / m;
    dxdt(5) = (Fxf * sin(delta) + Fyf * cos(delta) + Fyr - m * vx * omega) / m;
    dxdt(6) = (Lf * Fyf * cos(delta) + Fxf * sin(delta) - Lr * Fyr) / Iz;
end

% Nonlinear Model Function
function dxdt = vehicleDynamics1(t, x, u)
m = 1500; % Mass of the vehicle (kg)
Iz = 2667; % Inertia about z-axis (kg.m^2)
Lf = 1.2; % Distance from CG to front axle (m)
Lr = 1.6; % Distance from CG to rear axle (m)
Rw = 0.3; % Wheel radius (m)
Tmax = 500; % Maximum torque (Nm)
Kf = 50000; % Front tire stiffness (N/rad)
Kr = 60000; % Rear tire stiffness (N/rad)
Cr0 = 0.01; % Roll resistance coefficient
Cr1 = 0.005; % Air resistance coefficient
    % Extract states and inputs
    X = x(1); Y = x(2); psi = x(3); vx = x(4); vy = x(5); omega = x(6);
    tr = u(1); delta = u(2);
    
    % Tire forces
    Fxf = (tr * Tmax) / (2 * Rw);
    Fxr = Fxf;
    alpha_f = -atan((omega * Lf + vy) / (vx+0.00001)) + delta;
    alpha_r = atan((omega * Lr - vy) / (0.00001+vx));
    Fyf = Kf * alpha_f;
    Fyr = Kr * alpha_r;
    
    % Drag force
    Fdrag = Cr0 + Cr1 * vx^2;
    
    % Vehicle dynamics equations
    dxdt = zeros(6, 1);
    dxdt(1) = vx * cos(psi) - vy * sin(psi);
    dxdt(2) = vx * sin(psi) + vy * cos(psi);
    dxdt(3) = omega;
    dxdt(4) = (Fxf * cos(delta) - Fyf * sin(delta) + Fxr - Fdrag + m * vy * omega) / m;
    dxdt(5) = (Fxf * sin(delta) + Fyf * cos(delta) + Fyr - m * vx * omega) / m;
    dxdt(6) = (Lf * Fyf * cos(delta) + Fxf * sin(delta) - Lr * Fyr) / Iz;
end
