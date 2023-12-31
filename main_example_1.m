clear all;
close all
set(0,'DefaultTextInterpreter','latex')
global d_rand
d2=rand(3,1);d3=rand(3,1);
[x_history,u_history,d_history,d_real,t]=adaptive_cruise_control("without",zeros(3,1),0);
[x_history_with,u_history_with,d_history_with,d_real_with,t]=adaptive_cruise_control("with",0.2*ones(3,1),0);
[x_history_with_d,u_history_with_d,d_history_with_d,d_real_with_d,t]=adaptive_cruise_control("with",0.2*ones(3,1),1);
g=9.81;
m = 1650;  
font=20;
figure(1)
% plot(t, x_history(:, 2),"Linewidth",2);grid on;hold on;
plot(t, x_history_with(:, 2),"Linewidth",2);grid on;hold on;
plot(t, x_history_with_d(:, 2),"Linewidth",2);grid on;hold on;
legend('With ICBF','With DO-ICBF','Interpreter','latex')
set(gca,"FontSize",font)
xlabel('Time (s)','fontsize',font);
ylabel('Velocity $x_2$ (m/s)','fontsize',font);
% title('Velocity vs. Time','fontsize',font);



safe1= x_history(:,3)-1.8*x_history(:,2);
safe2= x_history_with(:,3)-1.8*x_history_with(:,2);
safe3= x_history_with_d(:,3)-1.8*x_history_with_d(:,2);

figure(2)
% plot(t, safe1,"Linewidth",2);grid on;hold on;
plot(t, safe2,"Linewidth",2);grid on;hold on;
plot(t, safe3,"Linewidth",2);grid on;hold on;
set(gca,"FontSize",font)
plot(t,zeros(length(u_history)),'k',"Linewidth",2);
legend('With ICBF','With DO-ICBF','Interpreter','latex')
xlabel('Time (in $s$)','fontsize',font);
ylabel('$h_x$','fontsize',font);
% title('h_x vs. Time');

safe4= (m*0.3*g)^2-u_history.*u_history;
safe5= (m*0.3*g)^2-u_history_with.*u_history_with;
safe6= (m*0.3*g)^2-u_history_with_d.*u_history_with_d;



figure(3)
plot(t, d_history_with(:,1),"Linewidth",2);grid on;hold on;
plot(t(1:end-1), d_real_with(:,1),"Linewidth",2);grid on;hold on;
%plot(t(1:end-1), d3(1)*ones(length(t)-1),"Linewidth",2);grid on;hold on;
set(gca,"FontSize",font)
 legend('$\hat{d}$','$d$','Interpreter','latex')
xlabel('Time (in $s$)','fontsize',font);
ylabel('Disturbances','fontsize',font);
% title('h_x vs. Time');




function [x_history,u_history,d_history,d_real,t]=adaptive_cruise_control(scenario,d_randd,delta)
% clear all close all
x_history=[];
u_history=[];
d_history=[];
d_real=[];

set(0,'DefaultTextInterpreter','latex')
global m c0 c1 c2 v0 vd alpha T g beta d_rand
d_rand=d_randd;
g=9.81;
m = 1650;           % vehicle mass (example value)
c0 =100;% 0.1;           % rolling resistance parameter (example value)
c1 =0.1;% 5;            % rolling resistance parameter (example value)
c2 = 5;%0.25;             % ignored as per the problem statement
ca = 0.4;
cd = 0.4;
psc=1;
v0 = 13.89;            % velocity of lead vehicle (example value)
vd = 24;            % desired speed (example value)
alpha = 1;          % control gain (example value)
T = 1;              % time horizon for prediction (example value)
itr=0;
beta=2;
% Simulation parameters
% tspan = [0 20];     % time span for simulation
z0 = [0; 20; 100; 0;0;0;0]; % initial state [position, velocity, distance, control]
alpha1=0.15;
alpha2=0.5;
N=200;Ts=0.1;z=z0';
checks=[];v_star=0;
d_real=[];
for i=1:N
x0=z0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Fr = c0 + c1*x0(2) + c2*x0(2)^2;
phi0 = -2*(x0(2) - vd)*Fr/m + eps*(x0(2) - vd)^2;
phi1 = 2*(x0(2) - vd)/m;
h0=x0(3)-1.8*x0(2);
h1=v0 - x0(2) +1.8*Fr/m + alpha1*(x0(3) - 1.8*x0(2))-1.8*x0(4)/m;
x2dot=-Fr/m+x0(4)/m;
phii=alpha*c1*(exp(-c1*T/m)-1)^-1 * (-c1^(-1)*(c0 - x0(4) + m*vd - c1*exp(-c1*T/m)*(x0(2) + (c0 - x0(4) + m*vd)/c1)));
LfB=-x2dot+1.8*(c1*x2dot+2*c2*x0(2)*x2dot)/m-1.8*phii/m+alpha2*h1;
LgB = 1.8/m;



A=[LgB ; 2*x0(4)];
b=[LfB-delta;((m*0.3*g)^2-x0(4)^2)];
H=2*eye(1);F=[0];
options = optimoptions('quadprog',...
    'Algorithm','interior-point-convex','Display','off');
[v_star,fval,exitflag,output,lambda] = ...
   quadprog(H,F,A,b,[],[],[],[],[],options);


checks=[checks;v_star];
if isempty(v_star)
    v_star=0;itr=itr+1;
end
if scenario=="without"
v_star=0;
end
%%%%%%%%%%%%%%%%%%%%%%%


% Solve the ODE
[t, y] = ode45(@(t,x) aug_dynamics(t,x,v_star), [0 Ts], z0);
z=[z;y(end,:)];
z0=y(end,:)';
d_real=[d_real;d_rand'];
end
t=(0:N)*Ts;
 %d_real=0.02*sin(2*pi*t)';
 %d_real=[d_real(:,1) d_real(:,1) d_real(:,1)];
font=20;
% Extract the state and control histories
x_history = z(:, 1:3);
u_history = z(:, 4)/(m*g);
d_history=z(:,5:7)+beta*x_history;





    function dz = aug_dynamics(t, z,v_star)
%     global m c0 c1 c2 v0 vd alpha T beta d_rand

            function Fr_val = Frv(x2)
        Fr_val = c0 + c1*x2 + c2*x2^2;
    end

    function f_val = f(x,u)
        f_val = [x(2); -1/m * Frv(x(2)) + 1/m * u; v0 - x(2)];
    end

    function phi_val = phi(x,u)
        phi_val = alpha*c1*(exp(-c1*T/m)-1)^-1 * y_hat(x,u);
    end

    function y_hat_val = y_hat(x,u)
        y_hat_val = -c1^-1*(c0 - u + m*vd - c1*exp(-c1*T/m)*(x(2) + (c0 - u + m*vd)/c1));
    end

l=2*eye(3);%l(1,1)=0;l(3,3)=0;
% if norm(d_rand)>0
%     d_rand=(0.2*sin(2*pi*t))*ones(3,1);
% end
% d_rand=0.1*ones(3,1);
%rand(3,1);
        x = z(1:3);
        u = z(4);
        d_hat=z(5:7)+beta*x;
        dr=-beta*(f(x,u)+l*d_hat);
         dx = f(x, u)+l*d_rand;%[0.1;0.1;0.1];
                %dx = f(x, u)+l*0.02*sin(2*pi*t)*ones(3,1);%[0.1;0.1;0.1];

        du = phi(x, u)+v_star;
        dz = [dx; du;dr];
    end
 end
