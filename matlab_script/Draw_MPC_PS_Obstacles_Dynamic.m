function Draw_MPC_PS_Obstacles_Dynamic (t,xx,xx1,xs1,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam)

set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)

line_width = 1.5;
fontsize_labels = 14;

%--------------------------------------------------------------------------
%-----------------------Simulate robots -----------------------------------
%--------------------------------------------------------------------------
x_r_1 = [];
y_r_1 = [];
x_s_1 = [];
y_s_1 = [];



r = rob_diam/2;  % robot radius
ang=0:0.005:2*pi;
xp=r*cos(ang);
yp=r*sin(ang);

r2 = obs_diam/2;  % obstacle radius
xp_obs=r2*cos(ang);
yp_obs=r2*sin(ang);

figure(500)
% Animate the robot motion
%figure;%('Position',[200 200 1280 720]);
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);


tic
for k = 1:size(xx,2)
    h_t = 0.14; w_t=0.09; % triangle parameters
    
    x1 = xs1(1,k,1); y1 = xs1(2,k,1); th1 = xs1(3,k,1);
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
    fill(x1_tri, y1_tri, 'g'); % plot reference state
    x_s_1 = [x_s_1 x1];
    y_s_1 = [y_s_1 y1];
    hold on;
    x1 = xx(1,k,1); y1 = xx(2,k,1); th1 = xx(3,k,1);
    x_r_1 = [x_r_1 x1];
    y_r_1 = [y_r_1 y1];
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];

    plot(x_r_1,y_r_1,'-r','linewidth',line_width);hold on % plot exhibited trajectory
    plot(x_s_1,y_s_1,'-g','linewidth',line_width);hold on
    if k < size(xx,2) % plot prediction
        plot(xx1(1:N,1,k),xx1(1:N,2,k),'r--*')
        for j = 2:N+1
            plot(xx1(j,1,k)+xp,xx1(j,2,k)+yp,'--r'); % plot robot circle
        end
    end
    
    fill(x1_tri, y1_tri, 'r'); % plot robot position
    
    plot(x1+xp,y1+yp,'--r'); % plot robot circle
    obs_plot_x = obs_x(:) + xp_obs;
    obs_plot_y = obs_y(:) + yp_obs;


    plot(obs_x,obs_y,'.r','linewidth',line_width);hold on

    plot(obs_plot_x,obs_plot_y,'.b');
    hold on

    
    hold off
    
    ylabel('$y$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    xlabel('$x$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    axis([-5 5 -5 5])
    box on;
    grid on
    drawnow
    % for video generation
    F(k) = getframe(gcf); % to get the current frame
end
toc
close(gcf)
% video = VideoWriter('exp.avi','Uncompressed AVI');

 video = VideoWriter('MPC_SC.avi','Motion JPEG AVI');
 video.FrameRate = 5;  % (frames per second) this number depends on the sampling time and the number of frames you have
 open(video)
 writeVideo(video,F)
 close (video)

 
figure
stairs(t,u_cl(:,1),'k','linewidth',1.5); 
hold on
grid on
stairs(t,u_cl(:,2),'r','linewidth',1.5); 
xlabel('time (seconds)')
axis([0 t(end) -1.2 1.2])
ylabel('u')
