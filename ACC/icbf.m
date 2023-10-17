nx = 3;  % number of states
nu = 1;  % number of inputs
ny = 3;  % number of outputs

nlobj = nlmpc(nx, ny, nu);

Ts = 0.1;  % sample time
nlobj.Ts = Ts;

nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 10;

% nlobj.Model.StateFcn = "robotDynamics";
nlobj.Model.StateFcn = @robotDynamics;

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
nlobj.Weights.OutputVariables = [1, 1, 0.5];  % You can change these weights according to your needs
nlobj.Weights.ManipulatedVariablesRate = [0.1];  % Weights on input rate changes
% Initial condition
x0 = [-5; -5; 0];
u0 = [0]; % Initial guess for the input
validateFcns(nlobj, x0, u0);

% Define a reference trajectory
ref = [2, 1, 0]; % For instance, to drive the robot to the origin with zero theta
xCurrent = x0;  % Initial state
u0 = [0]; % Initial guess for the input
xHistory = x0';  % to save state trajectory
uHistory = [];   % to save control input trajectory

% Compute the control actions using the NMPC
% [x, u, y, t] = sim(nlobj, x0, u0, ref);
N = 100; % total number of time steps, you can adjust this as needed
for k = 1:N
    % Compute control action using nlmpcmove
    [mv, ~] = nlmpcmove(nlobj, xCurrent, u0, ref);
    
    % Save control and state
    uHistory = [uHistory; mv'];
    
    % Integrate system dynamics over one time step using ODE solver (e.g., ode45)
    [~, y] = ode45(@(t, x) robotDynamics1(t, x, mv), [0 Ts], xCurrent);
    
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
subplot(1,1,1);
stairs(t(1:end-1), uHistory(:,1));
xlabel('Time (s)');
ylabel('v');
title('Control input v');

% subplot(2,1,2);
% stairs(t(1:end-1), uHistory(:,2));
% xlabel('Time (s)');
% ylabel('\omega');
% title('Control input \omega');

% Plot 2D trajectory
figure;
plot(xHistory(:,1), xHistory(:,2), '-o');
xlabel('x');
ylabel('y');
title('2D Trajectory');
