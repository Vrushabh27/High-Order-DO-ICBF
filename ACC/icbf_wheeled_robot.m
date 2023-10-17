nx = 3;  % number of states
nu = 1;  % number of inputs
ny = 3;  % number of outputs

nlobj = nlmpc(nx, ny, nu);

Ts = 0.1;  % sample time
nlobj.Ts = Ts;

nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 10;

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
nlobj.Weights.OutputVariables = [0.8, 0.8, 0.5];  % You can change these weights according to your needs
nlobj.Weights.ManipulatedVariablesRate = [0.1/2];  % Weights on input rate changes
% Initial condition
x0 = [-5; -5; 0];
u0 = [0.1]; % Initial guess for the input
validateFcns(nlobj, x0, u0);

% Define a reference trajectory
ref = [2, 2, 0]; % For instance, to drive the robot to the origin with zero theta
xCurrent = x0;  % Initial state
xHistory = x0';  % to save state trajectory
uHistory = [];   % to save control input trajectory
AddedU=[];
n=1;
alpha1=1/n;
alpha2=1/n;
alpha3=1/n;
% Compute the control actions using the NMPC
% [x, u, y, t] = sim(nlobj, x0, u0, ref);
N = 500*2; % total number of time steps, you can adjust this as needed
for k = 1:N
    % Compute control action using nlmpcmove
    [mv, ~] = nlmpcmove(nlobj, xCurrent, u0, ref);

        %%%%%%%% Changing the nominal u=mv
    if k>1
    mv=mv+v_star*Ts;
    end
    
    % Save control and state
    uHistory = [uHistory; mv'];


    % Integrate system dynamics over one time step using ODE solver (e.g., ode45)
    [~, y] = ode45(@(t, x) robotDynamics1(t, x, mv), [0 Ts], xCurrent);
    
    % Update current state
    xCurrent = y(end, :)';

   %%%%%%%%%%%%%%%%%%%%%%%%%%%
    xpos=xCurrent(1);ypos=xCurrent(2);theta=xCurrent(3);
    u=mv;
    if k==1
    udot=0;
    else
        u_prev=uHistory(end-1);
        udot=(u-u_prev)/Ts;
%         udot=((u-u_prev)./(xCurrent-xHistory(end-1,:)'))'*[cos(theta);sin(theta);tan(u)];
    end
    b0=xpos^2+ypos^2-1;
    b1=2*xpos*cos(theta)+2*ypos*sin(theta)+alpha1*b0;
    b2=(2*cos(theta)+2*alpha1*xpos)*cos(theta)+(2*sin(theta)+2*alpha1*ypos)*sin(theta)+(-2*xpos*sin(theta)+2*ypos*cos(theta))*tan(u)+alpha2*b1;
    p=(sec(u))^2*(-2*xpos*sin(theta)+2*ypos*cos(theta));
    d=[2*alpha1*cos(theta)-2*sin(theta)*tan(u)+alpha2*2*cos(theta)+2*alpha1*alpha2*xpos;....
        2*alpha1*sin(theta)+2*cos(theta)*tan(u)+2*alpha2*sin(theta)+2*alpha1*alpha2*ypos;.....
        -2*alpha1*xpos*sin(theta)+2*alpha1*ypos*cos(theta)+tan(u)*(-2*xpos*cos(theta)-2*ypos*sin(theta))]'*[cos(theta);sin(theta);tan(u)]+.....
        ((sec(u))^2)*(-2*xpos*sin(theta)+2*ypos*cos(theta))*udot+alpha3*b2;
    A = -p;
b = d;
H = 2;
F = 0;
options = optimoptions('quadprog',...
    'Algorithm','interior-point-convex','Display','off');
[v_star,fval,exitflag,output,lambda] = ...
   quadprog(H,F,A,b,[],[],[],[],[],options);

AddedU=[AddedU;v_star];

   %%%%%%%%%%%%%%%
    
    % Save state history
    xHistory = [xHistory; xCurrent'];
end



t = (0:N) * Ts;

% Plot states
figure;
subplot(3,1,1);
plot(t, xHistory(:,1),"LineWidth",2);
xlabel('Time (s)');
ylabel('x');
title('State x');

subplot(3,1,2);
plot(t, xHistory(:,2),"LineWidth",2);
xlabel('Time (s)');
ylabel('y');
title('State y');

subplot(3,1,3);
plot(t, xHistory(:,3),"LineWidth",2);grid on;hold on;
xlabel('Time (s)');
ylabel('\theta');
title('State \theta');

% Plot control inputs
figure;
% stairs(t(1:end-1), uHistory(:,1),"LineWidth",2);grid on;hold on;
plot(t(1:end-1),(uHistory(:,1))',"LineWidth",2);
xlabel('Time (s)');
ylabel('phi');
title('Control input phi');



% Plot 2D trajectory
figure;
plot(xHistory(:,1), xHistory(:,2),"LineWidth",2);grid on;hold on;
circle(0,0,1,'g');hold on;
ylim([-5.5, 3]);
xlabel('x');
ylabel('y');
title('2D Trajectory');

function circles = circle(x,y,r,c)
hold on
th = 0:pi/50:2*pi;
x_circle = r * cos(th) + x;
y_circle = r * sin(th) + y;
circles = plot(x_circle, y_circle);
fill(x_circle, y_circle, c)
hold off
axis equal
end

function [dx] = robotDynamics( x, u)
    phi = u(1);
    theta=x(3);
    % dx = zeros(3, 1);
    % dx(1) = v * cos(theta);
    % dx(2) = v * sin(theta);
    % dx(3) = omega;
    dx=[cos(theta);
      sin(theta);
   1*tan(phi)];

    y = x;  % assuming full-state measurement
end
function [dx] = robotDynamics1( t,x, u)
    phi = u(1);
    theta=x(3);
    % dx = zeros(3, 1);
    % dx(1) = v * cos(theta);
    % dx(2) = v * sin(theta);
    % dx(3) = omega;
    dx=[cos(theta);
      sin(theta);
   1*tan(phi)];

    y = x;  % assuming full-state measurement
end
