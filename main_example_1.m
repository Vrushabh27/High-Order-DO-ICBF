clear all;
close all
set(0,'DefaultTextInterpreter','latex')

[x_history,u_history,d_history,d_real,t]=ames_with_d_new("without",zeros(3,1),0);
[x_history_with,u_history_with,d_history_with,d_real_with,t]=ames_with_d_new("with",0.2*ones(3,1),0);
[x_history_with_d,u_history_with_d,d_history_with_d,d_real_with_d,t]=ames_with_d_new("with",0.2*ones(3,1),1);
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

figure(2)
plot(t, u_history,"Linewidth",2);grid on;hold on;
plot(t, u_history_with,"Linewidth",2);grid on;hold on;
plot(t, u_history_with_d,"Linewidth",2);grid on;hold on;
set(gca,"FontSize",font)
xlabel('Time $(s)$','fontsize',font);
ylabel('Control Input $u$','fontsize',font);
% title('$\text{Control Input vs. Time}$','fontsize',font);
safe1= x_history(:,3)-1.8*x_history(:,2);
safe2= x_history_with(:,3)-1.8*x_history_with(:,2);
safe3= x_history_with_d(:,3)-1.8*x_history_with_d(:,2);

figure(3)
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

figure(4)
% plot(t, safe4./(m*g),"Linewidth",2);grid on;hold on;
% plot(t, safe5./(m*g),"Linewidth",2);grid on;hold on;
% plot(t, safe6./(m*g),"Linewidth",2);grid on;hold on;
plot(t, (m*0.3*g)-u_history,"Linewidth",2);grid on;hold on;
plot(t, (m*0.3*g)-u_history_with,"Linewidth",2);grid on;hold on;
plot(t, (m*0.3*g)-u_history_with_d,"Linewidth",2);grid on;hold on;
set(gca,"FontSize",font)
legend('$h_u/(mg)^2$ (Without ICBF)','$h_u/(mg)^2$ (With ICBF)','$h_u/(mg)^2$ (With DO-ICBF)','Interpreter','latex')

% plot(t,zeros(length(u_history)),'k',"Linewidth",2);
xlabel('Time (s)','fontsize',font);
ylabel('$h_u$','fontsize',font);
% title('h_x vs. Time');

figure(5)
plot(t, d_history_with(:,1),"Linewidth",2);grid on;hold on;
plot(t(1:end-1), d_real_with(:,1),"Linewidth",2);grid on;hold on;
set(gca,"FontSize",font)
 legend('$\hat{d}$','$d$','Interpreter','latex')
xlabel('Time (in $s$)','fontsize',font);
ylabel('Disturbances','fontsize',font);
% title('h_x vs. Time');
