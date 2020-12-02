function Draw_MPC_PS_Obstacles (t,xx,xs1,u_cl,xs,rob_diam,obs_x,obs_y,obs_diam)

set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)

line_width = 1.5;
fontsize_labels = 14;

%--------------------------------------------------------------------------
%-----------------------Simulate robots -----------------------------------
%--------------------------------------------------------------------------
x_r_1 = [];
y_r_1 = [];



r = rob_diam/2;  % robot radius
ang=0:0.005:2*pi;
xp=r*cos(ang);
yp=r*sin(ang);

r2 = obs_diam/2;  % obstacle radius
xp_obs=r2*cos(ang);
yp_obs=r2*sin(ang);

L = 0.89;
H = 0.56;

x_initial = xx(1,1);
y_initial = xx(2,1);

rob_final = xx(:,end);
x_fin = rob_final(1);
y_fin = rob_final(2);
rob_final_theta = rob_final(3);

leader_final = xs1(:,end);
x_g = leader_final(1);
y_g = leader_final(2);
leader_final_theta = rob_final(3);


figure
obs_plot_x = obs_x(:) + xp_obs;
obs_plot_y = obs_y(:) + yp_obs;

h=0.9; w = 0.5; hh = h/2; ww = w/2;
t_X_initial = [x_initial+hh x_initial-hh x_initial-hh];
t_Y_initial = [y_initial y_initial-ww y_initial+ww];
fill(t_X_initial,t_Y_initial, 'r');hold on

t_X_g = [x_g+hh x_g-hh x_g-hh];
t_Y_g = [y_g y_g-ww y_g+ww]; 
xgc = x_g; ygc = y_g; tg = leader_final_theta;
xg_rot = (t_X_g - xgc)*cos(tg) + (t_Y_g-ygc)*sin(tg) + xgc;
yg_rot = (t_X_g - xgc)*sin(tg) + (t_Y_g-ygc)*cos(tg) + ygc;
fill(xg_rot,yg_rot, 'g');hold on

t_X_final = [x_fin+hh x_fin-hh x_fin-hh];
t_Y_final = [y_fin y_fin-ww y_fin+ww];
xfc = x_fin; yfc = y_fin; tf = rob_final_theta;
xg_rot = (t_X_final - xfc)*cos(tf) + (t_Y_final-yfc)*sin(tf) + xfc;
yg_rot = (t_X_final - xfc)*sin(tf) + (t_Y_final-yfc)*cos(tf) + yfc;
fill(xg_rot,yg_rot, 'g');hold on


% xunit = r*cos(th)+x_initial;
% yunit = r*sin(th)+y_initial;
% fill(xunit,yunit,'r');hold on
% xunit = r*cos(th)+x_g;
% yunit = r*sin(th)+y_g;
% fill(xunit,yunit,'g');hold on
% xunit = r*cos(th)+x_fin;
% yunit = r*sin(th)+y_fin;
% fill(xunit,yunit,'b');hold on

plot(obs_x,obs_y,'.r','linewidth',line_width);hold on
plot(obs_plot_x,obs_plot_y,'.b');hold on
plot(xx(1,:),xx(2,:),'r','linewidth',2);axis([-5 5 -5 5]);hold on
plot(xs1(1,:),xs1(2,:),'g','linewidth',2);

% Create ylabel
ylabel({'Distance along y-axis (in meters)'},'FontSize',18,...
    'FontName','Times New Roman');

% Create xlabel
xlabel({'Distance along x-axis (in meters)'},'FontSize',18,...
    'FontName','Times New Roman');

% Create title
title({'Path followed by robot with goal progression visualized as a path'},...
    'FontWeight','bold',...
    'FontSize',18,...
    'FontName','Times New Roman');


 
figure
stairs(t,u_cl(:,1),'k','linewidth',1.5); 
hold on
grid on
stairs(t,u_cl(:,2),'r','linewidth',1.5); 
xlabel('time (seconds)')
axis([0 t(end) -1.2 1.2])
ylabel('u')
