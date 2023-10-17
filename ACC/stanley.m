    % Simulation parameters
    clear all
    tspan = [0, 50]; % You can change the simulation time
    x0 = [-5; -5; pi/4; 0.5;0;1;0;1]; % Initial states [x; y; psi; v]

    % Controller gains (you need to set these values)
    global K_P K_I K_D k Ts u11 u22
    K_P = 80.5; %earlier it was 1
    K_I = 4.6;
    K_D = 0.05;
    k = 1;
    % vd = 0; % desired velocity

    % Simulate
    N=17;Ts=0.5;
    x_total=[];
time = (0:N) * Ts;
all_vstars=[];
%%%%%%%%%%%%%%%%%%%%
alpha1=0.9;
alpha2=1.2;
alpha3=0.5;
radius=0.5;
f = dynamics_analytical();
syms X Y psi_angle v delta_f a v_d
diff_of_f_x=[diff(f(1),X);diff(f(2),Y);diff(f(3),psi_angle);diff(f(4),v)];
b0=(X+2.5)^2+(Y)^2-radius^2;
diff_of_b0_with_time=[diff(b0,X);diff(b0,Y);diff(b0,psi_angle);diff(b0,v)]'*f+alpha1*(b0);
b1=diff_of_b0_with_time;
diff_of_b1_with_time=[diff(b1,X);diff(b1,Y);diff(b1,psi_angle);diff(b1,v)]'*f+alpha2*(b1);
b2=diff_of_b1_with_time;
residue=[diff(b2,X);diff(b2,Y);diff(b2,psi_angle);diff(b2,v)]'*f+alpha3*(b2);
v_d=1;%0.05*sqrt(X^2+Y^2);
v_d_dot=[diff(v_d,X);diff(v_d,Y);diff(v_d,psi_angle);diff(v_d,v)]'*f;
v_d_ddot=[diff(v_d_dot,X);diff(v_d_dot,Y);diff(v_d_dot,psi_angle);diff(v_d_dot,v)]'*f;
stanley_u=psi_angle+atan(k*(Y)/v);
stanley_u_dot=[diff(stanley_u,X);diff(stanley_u,Y);diff(stanley_u,psi_angle);diff(stanley_u,v)]'*f;
checks=[];
% x_total_test=[];
% y0=x0;
v_star=[0;0];itr=0;
%%%%%%%%%%%%%%

    for i=1:N
        
         [t, x] = ode45(@(t,x) dynamics(t,x,v_star), [0,Ts], x0);
         
x0=x(end,:);
% y0=y(end,:);
x_total=[x_total;x0];
% x_total_test=[x_total_test;y0];
    % [t, x] = ode45(@dynamics, tspan, x0);
    
%%%%%%%%%%%%%%%%%%%%
xc=x0';
[u1,u2]=controller(time(i),xc(3),xc(4),xc);
u1=u11;u2=u22;
% if i<=3
% u1=0;u2=0;
% end
u=[u1;u2];
udot_analytical=K_P*(v_d_dot-a)+K_I*(v_d-v);
udot=double([subs(udot_analytical,{X,Y,psi_angle,v,delta_f,a},{xc(1),xc(2),xc(3),xc(4),u(1),u(2)});.....
    subs(stanley_u_dot,{X,Y,psi_angle,v,delta_f,a},{xc(1),xc(2),xc(3),xc(4),u(1),u(2)})]); 

 p=[diff(b1,delta_f);diff(b1,a)];
diff_of_b1_with_time=diff_of_b1_with_time+[diff(b1,delta_f);diff(b1,a)]'*udot;
   
    A=-double(subs(p,{X, Y, psi_angle, v,delta_f,a},{xc(1),xc(2),xc(3),xc(4),u(1),u(2)}));
    A=A';
    % A=ones(1,2);
    b=double(subs(diff_of_b1_with_time,{X, Y, psi_angle,v,delta_f,a},{xc(1),xc(2),xc(3),xc(4),u(1),u(2)}));
        % b=double(subs(b0,{X, Y, psi_angle,v,delta_f,a},{xc(1),xc(2),xc(3),xc(4),u(1),u(2)}));
if A~=zeros(1,2)
    H=1*eye(2);F=zeros(2,1);
    options = optimoptions('quadprog',...
    'Algorithm','interior-point-convex','Display','off');
    [v_star,fval,exitflag,output,lambda] = ...
   quadprog(H,F,A,b,[],[],[],[],[],options);
    checks=[checks;b];
    all_vstars=[all_vstars,v_star];
    itr=itr+1;
 end
  v_star=[0;0];
%%%%%%%%%%%%%%%%%


    end
    x=x_total;
%     y=y_total;
    % figure;
    % subplot(2,2,1); plot( x(:,1),"Linewidth",2); title('x vs time');grid on;hold on;
    % xlabel('Time (s)'); ylabel('x position');
    % 
    % subplot(2,2,2); plot(x(:,2),"Linewidth",2); title('y vs time');
    % xlabel('Time (s)'); ylabel('y position');
    % 
    % subplot(2,2,3); plot(x(:,3),"Linewidth",2); title('\psi vs time');
    % xlabel('Time (s)'); ylabel('Orientation (\psi)');
    % 
    % subplot(2,2,4); plot( x(:,4),"Linewidth",2); title('v vs time');grid on;hold on;
    % xlabel('Time (s)'); ylabel('Velocity (v)');
    % plot( x(:,1),x(:,2),"Linewidth",2); title('x vs y')
    figure(2)
    plot( x(:,5),x(:,6),"Linewidth",2); title('x vs y')

function dx = dynamics(t, x,v_star)
global Ts u11 u22
    % Unpack states
    x_pos = x(1);
    y_pos = x(2);
    psi = x(3);
    v = x(4);
    vd=1;%0.05*sqrt(x_pos^2+y_pos^2);
    % Controller
     [delta_f, a] = controller(t, psi, v,x);
     u11=delta_f;
     u22=a;
     delta_f=delta_f+v_star(1)*Ts;
     a=a+v_star(2)*Ts;
% delta_f=u(1);
% a=u(2);

    % Compute beta
    lr = 1; % rear wheel to center of gravity
    lf = 1; % front wheel to center of gravity
    beta = atan(lr / (lf + lr) * tan(delta_f));
psi_d=x(7);
    % System dynamics
    dx1 = v * cos(psi + beta);
    dx2 = v * sin(psi + beta);
    dx3 = v/lr * sin(beta);
    dx4 = a;
   a_d=0;v_d=1;psi_d=0;beta_d=pi/2;
        dx1_d = v_d * cos(psi_d + beta_d);
    dx2_d = v_d * sin(psi_d + beta_d);
    dx3_d = v_d/lr * sin(beta_d);
    dx4_d = a_d;

    dx = [dx1; dx2; dx3; dx4;dx1_d;dx2_d;dx3_d;dx4_d];
end

function [delta_f, a] = controller(t, psi, v,x)
    global K_P K_I K_D k 
    vd=1;%0.05*sqrt(x(1)^2+x(2)^2);
    % Calculate the error
    e = vd - v;

    % PD control for acceleration
    persistent e_int
    if isempty(e_int)
        e_int = 0;
    end
    de = (e - e_int) / (t - 1); % Simple difference for derivative (not accurate for small timesteps)
    e_int = e_int + e; % Simple integration

    a = K_P * e + K_I * e_int + K_D * de;

    % Lateral control for steering angle
    d = (x(2)-x(1))/sqrt(2); % This is the lateral error, you might need to compute it based on path tracking
    delta_f = psi + atan(k * (d) / v);
end

function f = dynamics_analytical()
syms psi_angle delta_f v a 

    % Compute beta
    lr = 1.5; % rear wheel to center of gravity
    lf = 1.5; % front wheel to center of gravity
    beta = atan(lr / (lf + lr) * tan(delta_f));
f=[    v * cos(psi_angle + atan(lr / (lf + lr) * tan(delta_f)));
    v * sin(psi_angle + atan(lr / (lf + lr) * tan(delta_f)));
    v/lr * sin(atan(lr / (lf + lr) * tan(delta_f)));
    a];

end


