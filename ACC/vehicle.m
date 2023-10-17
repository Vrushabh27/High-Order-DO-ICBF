

% Initialize NMPC
nx = 6; % Number of states
nu = 2; % Number of inputs
ny=2;
nlobj = nlmpc(nx,ny, nu);

% Define state and input names
nlobj.States = struct('Name', {'X' 'Y' 'psi' 'vx' 'vy' 'omega'});
nlobj.Inputs = struct('Name', {'tr' 'delta'});

% Specify prediction model
nlobj.Model.StateFcn = "vehicleDynamics";

% Specify constraints
nlobj.MV(1).Min = -1;
nlobj.MV(1).Max = 1;
nlobj.MV(2).Min = -deg2rad(150);
nlobj.MV(2).Max = deg2rad(150);

% Specify cost function
nlobj.Optimization.CustomCostFcn = @(X,U,e,data) myCostFunction(X,U,e,data);



% Simulation and control loop
T = 20; % Total simulation time
dt = 0.1; % Time step
N = T/dt; % Number of time steps
x0 = [-20; 0; 0; 5; 0; 0]; % Initial state
u0 = [0; 0]; % Initial input
x = x0;
u = u0;
X_hist = x0;
U_hist = u0;

% Initialize plot
figure;
hold on;
grid on;
xlabel('X');
ylabel('Y');
title('Vehicle Trajectory');

for k = 1:N
    % Solve NMPC optimization problem
    [mv, ~] = nlmpcmove(nlobj, x, u);
    
    % Update state using Euler's method
    dx = vehicleDynamics(0, x, mv);
    x = x + dx * dt;
    
    % Store history
    X_hist = [X_hist, x];
    U_hist = [U_hist, mv];
    
    % Update plot
    plot(X_hist(1, :), X_hist(2, :), 'b');
    pause(0.01);
end

hold off;
% Constants and Parameters


% Nonlinear Model Function
function dxdt = vehicleDynamics(t, x, u)
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
    alpha_f = -atan((omega * Lf + vy) / vx) + delta;
    alpha_r = atan((omega * Lr - vy) / vx);
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

% Custom Cost Function
function J = myCostFunction(X, U, e, data)
    % Objective is to steer the vehicle to the origin
    X_goal = [0; 0; 0; 0; 0; 0];
    J = sum((X - X_goal).^2, 'all');
end