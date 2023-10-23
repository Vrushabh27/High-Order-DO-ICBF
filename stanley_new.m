clear all
set(0,'DefaultTextInterpreter','latex')

% Simulation parameters
dt = 0.1;
num_steps = 700;
% Define the circle and its parameters
radius = 10; % Radius of the circle
center = [0, 10]; % Center of the circle
theta = 0:0.1:2*pi; % Discrete points around the circle
circle_x = center(1) + radius*cos(theta);
circle_y = center(2) + radius*sin(theta);
circle_theta = atan2(diff(circle_y), diff(circle_x)); % Heading of the tangent
global v
% Vehicle parameters
v = 0.5; % constant velocity
wheelbase = 1; % length between front and rear axle

% Initial vehicle state [x, y, yaw]
vehicle_state = [radius+2, 10, pi/2];
steering_angles=[];


% Stanley parameters
k = 0.5; % Gain for cross track error

ylim([10 25]);
vehicle_states=[];
steering_angle=0;
%%%%%%%%%%%%%%%%
checks=[];
radius1=1;
alpha1=0.2;
alpha2=2;
alpha3=2;
f = analytical();
syms X Y psii steering
diff_of_f_x=[diff(f(1),X);diff(f(2),Y);diff(f(3),psii)];
b0=(X)^2+(Y)^2-radius1^2;
diff_of_b0_with_time=[diff(b0,X);diff(b0,Y);diff(b0,psii)]'*f+alpha1*(b0);
b1=diff_of_b0_with_time;
diff_of_b1_with_time=[diff(b1,X);diff(b1,Y);diff(b1,psii)]'*f+alpha2*(b1);
b2=diff_of_b1_with_time;
residue=[diff(b2,X);diff(b2,Y);diff(b2,psii)]'*f+alpha3*(b2);

%%%%%%%%%%%%%%%%


% Simulation loop
for i = 1:num_steps

%%%%%%%%%%%%%%%
 if i<3
udot=0;
 else
udot=(steering_angle-steering_angles(end-1))/dt;
end
 p=[diff(b2,steering)];
diff_of_b2_with_time=residue+[diff(b2,steering)]'*udot;
A=-double(subs(p,{X, Y, psii,steering},{vehicle_state(1),vehicle_state(2),vehicle_state(3),steering_angle}));
b=double(subs(diff_of_b1_with_time,{X, Y, psii,steering},{vehicle_state(1),vehicle_state(2),vehicle_state(3),steering_angle}));
    H=1*eye(1);F=zeros(1,1);
    options = optimoptions('quadprog',...
    'Algorithm','interior-point-convex','Display','off');
    [v_star,fval,exitflag,output,lambda] = ...
   quadprog(H,F,A,b,[],[],[],[],[],options);
checks=[checks;v_star];
% if isempty(v_star)
%     v_star=0;
% end

%v_star=0;

%%%%%%%%%%%%%%%%

beta=1;


    % Find the closest point on the path
    dist = sqrt((circle_x - vehicle_state(1)).^2 + (circle_y - vehicle_state(2)).^2);
    [min_dist, idx] = min(dist);
    
    % Cross-track error
    cte = min_dist;
    if vehicle_state(2) < circle_y(idx)
        cte = -cte;
    end
    circle_theta = atan2(diff(circle_y), diff(circle_x)); % Heading of the tangent
circle_theta = [circle_theta, circle_theta(1)]; % Append the first heading to the end

    % Heading error
    yaw_path = circle_theta(idx);
    yaw_diff = yaw_path - vehicle_state(3);
    yaw_diff = atan2(sin(yaw_diff), cos(yaw_diff)); % Normalize to [-pi, pi]
    
    % Stanley steering control
    steering_angle = (yaw_diff + atan2(k*cte, v));
    % if steering_angle>pi
    %     steering_angle=steering_angle+pi/2;
    % end
steering_angle=steering_angle+v_star;
    % Update vehicle state using bicycle model
    %d_estimate=(vehicle_state(4:6)+beta*vehicle_state(1:3))';
    vehicle_state(1) = vehicle_state(1) + (v*cos(vehicle_state(3)))*dt;
    vehicle_state(2) = vehicle_state(2) + v*sin(vehicle_state(3))*dt;
    vehicle_state(3) = vehicle_state(3) + v*tan(steering_angle)/wheelbase*dt;

    % Normalize yaw to [-pi, pi]
   % vehicle_state(3) = atan2(sin(vehicle_state(3)), cos(vehicle_state(3)));
    %vehicle_state(1,4:6) = vehicle_state(1,4:6) + ([v*cos(vehicle_state(3));v*sin(vehicle_state(3));v*tan(steering_angle)/wheelbase]+d_estimate)'*dt;

    steering_angles=[steering_angles;steering_angle];
    % Plot the vehicle position
    % plot(vehicle_state(1), vehicle_state(2), 'r.');hold on;
    vehicle_states=[vehicle_states;vehicle_state];
    % ylim([10 25]);
end
font=20;
figure(1)
plot(circle_x, circle_y, 'b');hold on; % Plot the circle path
    plot(vehicle_states(:,1), vehicle_states(:,2), 'r.');hold on;
circle(0,0,radius1,'g');hold on;
set(gca,"FontSize",font)
ylim([-5 12]);
xlabel('X (in m)','fontsize',font);
ylabel('Y (in m)','fontsize',font);
title('Vehicle tracking a circle using High order DO-ICBF','fontsize',font);
grid on;ylim([-5 15]);xlim([-12 12]);
legend('Desired Path', 'Vehicle Path');
% axis equal;
t=(1:num_steps)*dt;
figure(2)
plot(t,(steering_angles(:,1)), 'r');hold on; % Plot the circle path
    % plot(vehicle_states(:,1), vehicle_states(:,2), 'r.');
figure(3)
b1_values=double(subs(b1,{X, Y, psii,steering},{vehicle_states(:,1),vehicle_states(:,2),vehicle_states(:,3),steering_angle}));
b2_values=double(subs(b2,{X, Y, psii,steering},{vehicle_states(:,1),vehicle_states(:,2),vehicle_states(:,3),steering_angle}));
plot(t,vehicle_states(:,1).^2+vehicle_states(:,2).^2-radius1^2,"Linewidth",2);hold on;grid on;
plot(t,b1_values,'g',"Linewidth",2);hold on;
plot(t,b2_values,'r',"Linewidth",2);hold on;
plot(t,zeros(length(steering_angles)),'k',"Linewidth",2);hold on;
title('High order DO-ICBF','fontsize',font);
ylim([-10 255]);xlim([0 t(end)]);
set(gca,"FontSize",font)
xlabel('Time (s)','fontsize',font);
ylabel('DO-ICBF','fontsize',font);
legend('$b_0(x)$','$b_1(x)$','$b_2(x,u)$','Interpreter','latex');


figure(4)
plot(t,vehicle_states(:,3), 'b');grid on;hold on; % Plot the circle path
set(gca,"FontSize",font);
xlim([0 t(end)]);
xlabel('Time (s)','fontsize',font);
ylabel('$\psi$ (rad)','fontsize',font,'Interpreter','latex');
%legend('$b_0(x)$','$b_1(x)$','$b_2(x,u)$','Interpreter','latex');
% figure(2); 
%     plot(circle_x, 'b');hold on; % Plot the circle path
%     plot( vehicle_states(:,1), 'r.');
% 
% figure(3); 
%     plot(circle_y, 'b');hold on; % Plot the circle path
%     plot( vehicle_states(:,2), 'r.');
% ylim([10 25]);



function f=analytical()
syms psii steering 
global v
f=[v*cos(psii);v*sin(psii);v*tan(steering)];
end
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
