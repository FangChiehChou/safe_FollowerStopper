

clear all
clc

load("2020-07-08-15-15-54_2T3MWRFVXLW056972_CAN_Messages.mat")
valid_accel = accelx;
valid_relative_dist = lead_distance;
valid_relative_spd = relative_vel;
valid_relative_spd(lead_distance>120) = nan;
valid_speed = speed/3.6;

%% overlay data with different deceleraito range
time_aligned = Time-Time(1);
time_enter = 430;
time_exit = 1301;

[~,highway_s_I] = min(abs(time_aligned-time_enter));
[~,highway_e_I] = min(abs(time_aligned-time_exit));

accelx_highway = accelx(highway_s_I:highway_e_I);
highway_rel_speed = relative_vel(highway_s_I:highway_e_I);
highway_lead_dist = lead_distance(highway_s_I:highway_e_I);
speed_highway = valid_speed(highway_s_I:highway_e_I);



%% pick data from 680~780 and do a 3D scatter
time_init = 680;
time_end = 780;

[~,s_I] = min(abs(time_aligned-time_init));
[~,e_I] = min(abs(time_aligned-time_end));

accelx_T1 = accelx(s_I:e_I);
rel_speed_T1 = relative_vel(s_I:e_I);
lead_dist_T1 = lead_distance(s_I:e_I);
speed_T1 = valid_speed(s_I:e_I);
time_T1 = time_aligned(s_I:e_I);

color_pt_temp = (time_T1-time_T1(1))/(time_T1(end)-time_T1(1));

figure()
scatter3(lead_dist_T1,rel_speed_T1,speed_T1,160,color_pt_temp,...
        'o','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
hold on
plot3(lead_dist_T1,rel_speed_T1,speed_T1)

text(lead_dist_T1(1),rel_speed_T1(1),speed_T1(1),'init')
text(lead_dist_T1(end),rel_speed_T1(end),speed_T1(end),'end')

xlabel('Relative distance[m]')
ylabel('Relative speed[m/s]')
zlabel('Follower speed[m/s]')
set(gca,'FontSize',30)

% ticks_value = [0 1];
% hcb = colorbar('Ticks',ticks_value,...
%          'TickLabels',{'T_0',['T_0+',num2str(ticks_value(2))]});

colormap copper


%%

r = 30;
uMax = 1.5;   %+1.5 m/s^2  = AV acceleration bound
uMin = -3.0;  %-3.0 m/s^2  = AV acceleration bound

v_lead = 10;
d_rel_0 = 30;
v_f_0 = 10;

d_rel_set = [30,40,40,20,15,20,15,20,8];
v_f_set = [10,15,10,17,17,8,6,18,16];

v_lead_set = [5,10,15,20];

%% plot phase portrait and envelopes of the FollowerStopper


cmap1=colormap(hot(15));

for i =1:1:length(d_rel_set)
    for j = 1:1:length(v_lead_set)
        
        
        v_lead = v_lead_set(j);
        d_rel_0 = d_rel_set(i);
        v_f_0 = v_f_set(i);
        x0 = [d_rel_0;v_f_0]; 
        tspan = [0 30];
%         [t,y] = ode45(@(t,x) FS_simulation_model(t,x,uMin,uMax,r,v_lead),tspan,x0);    
%         plot3(y(:,1),v_lead-y(:,2),y(:,2),'Color',[0.8500 0.3250 0.0980],'LineWidth',2);
%         hold on

        [t,y] = ode45(@(t,x) mFS_simulation_model(t,x,uMin,uMax,r,v_lead),tspan,x0);    

%         plot3(y(:,1),v_lead-y(:,2),y(:,2),'Color',[0.4660 0.6740 0.1880],'LineWidth',2);
        
        plot3(y(:,1),v_lead-y(:,2),y(:,2),'Color',cmap1(j,:),'LineWidth',2);

        hold on
        
               
    end

end
grid on
set(gca,'FontSize',30)
set(gcf, 'Color', 'w');


%%
figure()
scatter3(lead_dist_T1,rel_speed_T1,speed_T1,160,color_pt_temp,...
        'o','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
hold on
plot3(lead_dist_T1,rel_speed_T1,speed_T1)

text(lead_dist_T1(1),rel_speed_T1(1),speed_T1(1),'init')
text(lead_dist_T1(end),rel_speed_T1(end),speed_T1(end),'end')

xlabel('Relative distance[m]')
ylabel('Relative speed[m/s]')
zlabel('Follower speed[m/s]')
set(gca,'FontSize',30)

% ticks_value = [0 1];
% hcb = colorbar('Ticks',ticks_value,...
%          'TickLabels',{'T_0',['T_0+',num2str(ticks_value(2))]});

colormap copper
hold on

%%
clear v_lead
lead_speed_T1 = speed_T1 + rel_speed_T1;
v_lead = [(time_T1-time_T1(1));lead_speed_T1];

d_rel_set = [30,40,40,20,15,20,15,20,8,rand(1,3)*50];

v_f_set = lead_speed_T1(1) + (rand(1,length(d_rel_set))*8)-4;


tspan = [0 100];
winter

Simout = [];

for i = 1:1:length(d_rel_set)
    d_rel_0 = d_rel_set(i);
    v_f_0 = v_f_set(i);
    x0 = [d_rel_0;v_f_0]; 
    
    [t,y] = ode45(@(t,x) mFS_simulation_model(t,x,uMin,uMax,r,v_lead),tspan,x0);    
    y_v_lead = interp1(v_lead(1,:),v_lead(2,:),t);
    plot3(y(:,1),y_v_lead-y(:,2),y(:,2),'Color',cmap1(i,:),'LineWidth',2);
%     pause

    temp_sim_out.xrel = y(:,1);
    temp_sim_out.vrel = y_v_lead-y(:,2);
    temp_sim_out.vav = y(:,2);
    Simout = [Simout;temp_sim_out];
end

grid on
set(gca,'FontSize',30)
set(gcf, 'Color', 'w');


%% generate surfaces of envelopes for the modified FollowerStopper
speed_grid = 0:1:30;
v_rel_grid = -10:1:10;
[XX,YY] = meshgrid(speed_grid,v_rel_grid);

[envelope1,envelope2,envelope3] = m_FS_evnvelopes(speed_grid,v_rel_grid);


%%
figure()
for i = 1:1:length(Simout)
    plot3(Simout(i).xrel,Simout(i).vrel,Simout(i).vav,'Color',cmap1(i,:),'LineWidth',2);
    hold on
    text(Simout(i).xrel(1),Simout(i).vrel(1),Simout(i).vav(1),'init')
    text(Simout(i).xrel(end),Simout(i).vrel(end),Simout(i).vav(end),'end')
end
a = get(gca,'Clim');
surf(envelope3',YY,XX,'FaceColor','b', 'EdgeColor','b','Facealpha',0.4,'Edgealpha',0.5)
hold on
set(gca,'Clim',a);
xlabel('Relative distance[m]')
ylabel('Relative speed[m/s]')
zlabel('Speed[m/s]')
set(gca,'FontSize',30)
set(gcf, 'Color', 'w');
% eval(['export_fig ','mFS_envelope1_data',' -jpg']);
grid on

figure()
for i = 1:1:length(Simout)
    plot3(Simout(i).xrel,Simout(i).vrel,Simout(i).vav,'Color',cmap1(i,:),'LineWidth',2);
    hold on
    text(Simout(i).xrel(1),Simout(i).vrel(1),Simout(i).vav(1),'init')
    text(Simout(i).xrel(end),Simout(i).vrel(end),Simout(i).vav(end),'end')
end
%Get the CLim values after plotting scatterplot
a = get(gca,'Clim');
surf(envelope3',YY,XX,'FaceColor',[0 0 0.2], 'EdgeColor','b','Facealpha',0.2,'Edgealpha',0.5)
hold on
surf(envelope2',YY,XX,'FaceColor',[0 0.2 0], 'EdgeColor','g','Facealpha',0.2,'Edgealpha',0.5)

set(gca,'Clim',a);
xlabel('Relative distance[m]')
ylabel('Relative speed[m/s]')
zlabel('Speed[m/s]')
set(gca,'FontSize',30)
set(gcf, 'Color', 'w');
% eval(['export_fig ','mFS_envelope2_data',' -jpg']);
grid on

figure()
for i = 1:1:length(Simout)
    plot3(Simout(i).xrel,Simout(i).vrel,Simout(i).vav,'Color',cmap1(i,:),'LineWidth',2);
    hold on
    text(Simout(i).xrel(1),Simout(i).vrel(1),Simout(i).vav(1),'init')
    text(Simout(i).xrel(end),Simout(i).vrel(end),Simout(i).vav(end),'end')
end
%Get the CLim values after plotting scatterplot
a = get(gca,'Clim');
surf(envelope1',YY,XX,'FaceColor',[0.5 0 0], 'EdgeColor','r','Facealpha',0.2,'Edgealpha',0.5)
hold on
surf(envelope2',YY,XX,'FaceColor',[0 0.2 0], 'EdgeColor','g','Facealpha',0.2,'Edgealpha',0.5)
surf(envelope3',YY,XX,'FaceColor',[0 0 0.2], 'EdgeColor','b','Facealpha',0.2,'Edgealpha',0.5)

set(gca,'Clim',a);
xlabel('Relative distance[m]')
ylabel('Relative speed[m/s]')
zlabel('Speed[m/s]')
set(gca,'FontSize',30)
set(gcf, 'Color', 'w');
% eval(['export_fig ','mFS_envelope3_data',' -jpg']);
grid on



%% help function --- modified FS envelopes
function [envelope1,envelope2,envelope3] = m_FS_evnvelopes(speed_grid,v_rel_grid)
    delta0_x1 = 4.5;
    delta0_x2 = 5.25;
    delta0_x3 = 6.0;
    d_1 = 1.5;
    d_2 = 1.0;
    d_3 = 0.5;
    h_1 = 0.4;
    h_2 = 1.2;
    h_3 = 1.8;

    envelope1 = zeros(length(speed_grid),length(v_rel_grid));
    envelope2 = zeros(length(speed_grid),length(v_rel_grid));
    envelope3 = zeros(length(speed_grid),length(v_rel_grid));
    
    
    for i = 1:1:length(speed_grid)
        for j = 1:1:length(v_rel_grid)
            v_f = speed_grid(i);
            
            dv_ss = v_rel_grid(j);
            dv_ss(dv_ss>=0) = 0;          
            
            del_x_bound1 =  delta0_x1 + (dv_ss.^2)/(2*d_1) + h_1*v_f;
            del_x_bound2 =  delta0_x2 + (dv_ss.^2)/(2*d_2) + h_2*v_f;
            del_x_bound3 =  delta0_x3 + (dv_ss.^2)/(2*d_3) + h_3*v_f;
            envelope1(i,j) =  del_x_bound1;
            envelope2(i,j) =  del_x_bound2;
            envelope3(i,j) =  del_x_bound3;
        end
    end
    
end


