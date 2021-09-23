

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

%% plot 3d FS plane 

speed_grid = 0:1:30;
v_rel_grid = -10:1:10;
[XX,YY] = meshgrid(speed_grid,v_rel_grid);

[envelope1,envelope2,envelope3] = m_FS_evnvelopes(speed_grid,v_rel_grid);


figure()
surf(envelope1',YY,XX,'FaceColor',[1 1 1], 'EdgeColor','r')
alpha 0.1
hold on
surf(envelope2',YY,XX,'FaceColor',[1 1 1], 'EdgeColor','g')
alpha 0.1
surf(envelope3',YY,XX,'FaceColor',[1 1 1], 'EdgeColor','b')
alpha 0.1

scatter3(lead_dist_T1,rel_speed_T1,speed_T1,160,color_pt_temp,...
        'o','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
hold on
plot3(lead_dist_T1,rel_speed_T1,speed_T1)

text(lead_dist_T1(1),rel_speed_T1(1),speed_T1(1),'init')
text(lead_dist_T1(end),rel_speed_T1(end),speed_T1(end),'end')

xlabel('Relative distance[m]')
ylabel('Relative speed[m/s]')
zlabel('Speed[m/s]')
set(gca,'FontSize',30)
colormap copper

%%
figure()
pt_plot_handle = scatter3(lead_dist_T1,rel_speed_T1,speed_T1,160,color_pt_temp,...
        'o','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0); 
hold on 
plot3(lead_dist_T1,rel_speed_T1,speed_T1)
text(lead_dist_T1(1),rel_speed_T1(1),speed_T1(1),'init')
text(lead_dist_T1(end),rel_speed_T1(end),speed_T1(end),'end')
colormap copper
%Get the CLim values after plotting scatterplot
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


figure()
pt_plot_handle = scatter3(lead_dist_T1,rel_speed_T1,speed_T1,160,color_pt_temp,...
        'o','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0); 
hold on 
plot3(lead_dist_T1,rel_speed_T1,speed_T1)
text(lead_dist_T1(1),rel_speed_T1(1),speed_T1(1),'init')
text(lead_dist_T1(end),rel_speed_T1(end),speed_T1(end),'end')
colormap copper
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


figure()
pt_plot_handle = scatter3(lead_dist_T1,rel_speed_T1,speed_T1,160,color_pt_temp,...
        'o','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0); 
hold on 
plot3(lead_dist_T1,rel_speed_T1,speed_T1)
text(lead_dist_T1(1),rel_speed_T1(1),speed_T1(1),'init')
text(lead_dist_T1(end),rel_speed_T1(end),speed_T1(end),'end')
colormap copper
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


%%
[FS_envelope1,FS_envelope2,FS_envelope3] = FS_evnvelopes(speed_grid,v_rel_grid);
figure()
pt_plot_handle = scatter3(lead_dist_T1,rel_speed_T1,speed_T1,160,color_pt_temp,...
        'o','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0); 
hold on 
plot3(lead_dist_T1,rel_speed_T1,speed_T1)
text(lead_dist_T1(1),rel_speed_T1(1),speed_T1(1),'init')
text(lead_dist_T1(end),rel_speed_T1(end),speed_T1(end),'end')
colormap copper
%Get the CLim values after plotting scatterplot
a = get(gca,'Clim');
surf(FS_envelope3',YY,XX,'FaceColor','b', 'EdgeColor','b','Facealpha',0.4,'Edgealpha',0.5)
hold on
set(gca,'Clim',a);

xlabel('Relative distance[m]')
ylabel('Relative speed[m/s]')
zlabel('Speed[m/s]')
set(gca,'FontSize',30)
set(gcf, 'Color', 'w');
% eval(['export_fig ','FS_envelope1_data',' -jpg']);


figure()
pt_plot_handle = scatter3(lead_dist_T1,rel_speed_T1,speed_T1,160,color_pt_temp,...
        'o','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0); 
hold on 
plot3(lead_dist_T1,rel_speed_T1,speed_T1)
text(lead_dist_T1(1),rel_speed_T1(1),speed_T1(1),'init')
text(lead_dist_T1(end),rel_speed_T1(end),speed_T1(end),'end')
colormap copper
%Get the CLim values after plotting scatterplot
a = get(gca,'Clim');
surf(FS_envelope3',YY,XX,'FaceColor',[0 0 0.2], 'EdgeColor','b','Facealpha',0.2,'Edgealpha',0.5)
hold on
surf(FS_envelope2',YY,XX,'FaceColor',[0 0.2 0], 'EdgeColor','g','Facealpha',0.2,'Edgealpha',0.5)
set(gcf, 'Color', 'w');
% eval(['export_fig ','FS_envelope2_data',' -jpg']);



set(gca,'Clim',a);
xlabel('Relative distance[m]')
ylabel('Relative speed[m/s]')
zlabel('Speed[m/s]')
set(gca,'FontSize',30)

figure()
pt_plot_handle = scatter3(lead_dist_T1,rel_speed_T1,speed_T1,160,color_pt_temp,...
        'o','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0); 
hold on 
plot3(lead_dist_T1,rel_speed_T1,speed_T1)
text(lead_dist_T1(1),rel_speed_T1(1),speed_T1(1),'init')
text(lead_dist_T1(end),rel_speed_T1(end),speed_T1(end),'end')
colormap copper
%Get the CLim values after plotting scatterplot
a = get(gca,'Clim');
surf(FS_envelope1',YY,XX,'FaceColor',[0.5 0 0], 'EdgeColor','r','Facealpha',0.2,'Edgealpha',0.5)
hold on
surf(FS_envelope2',YY,XX,'FaceColor',[0 0.2 0], 'EdgeColor','g','Facealpha',0.2,'Edgealpha',0.5)
surf(FS_envelope3',YY,XX,'FaceColor',[0 0 0.2], 'EdgeColor','b','Facealpha',0.2,'Edgealpha',0.5)

set(gca,'Clim',a);
xlabel('Relative distance[m]')
ylabel('Relative speed[m/s]')
zlabel('Speed[m/s]')
set(gca,'FontSize',30)
set(gcf, 'Color', 'w');
% eval(['export_fig ','FS_envelope3_data',' -jpg']);





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


function [envelope1,envelope2,envelope3] = FS_evnvelopes(speed_grid,v_rel_grid)
    delta0_x1 = 4.5;
    delta0_x2 = 5.25;
    delta0_x3 = 6.0;
    d_1 = 1.5;
    d_2 = 1.0;
    d_3 = 0.5;

    envelope1 = zeros(length(speed_grid),length(v_rel_grid));
    envelope2 = zeros(length(speed_grid),length(v_rel_grid));
    envelope3 = zeros(length(speed_grid),length(v_rel_grid));
    
    
    for i = 1:1:length(speed_grid)
        for j = 1:1:length(v_rel_grid)
            v_f = speed_grid(i);
            
            dv_ss = v_rel_grid(j);
            dv_ss(dv_ss>=0) = 0;          
            
            del_x_bound1 =  delta0_x1 + (dv_ss.^2)/(2*d_1);
            del_x_bound2 =  delta0_x2 + (dv_ss.^2)/(2*d_2);
            del_x_bound3 =  delta0_x3 + (dv_ss.^2)/(2*d_3);
            envelope1(i,j) =  del_x_bound1;
            envelope2(i,j) =  del_x_bound2;
            envelope3(i,j) =  del_x_bound3;
        end
    end
    
end


%% help functions --- overlay FS curves
function [h1,h2,h3] = overlay_FS_curves(v_f)

dv_s = -10:0.1:10;
dv_ss = dv_s;
dv_ss(dv_ss>=0) = 0;

delta0_x1 = 4.5;
delta0_x2 = 5.25;
delta0_x3 = 6.0;
d_1 = 1.5;
d_2 = 1.0;
d_3 = 0.5;
h_1 = 0.4;
h_2 = 1.2;
h_3 = 1.8;

del_x_bound1 =  delta0_x1 + (dv_ss.^2)/(2*d_1) + h_1*v_f;
del_x_bound2 =  delta0_x2 + (dv_ss.^2)/(2*d_2) + h_2*v_f;
del_x_bound3 =  delta0_x3 + (dv_ss.^2)/(2*d_3) + h_3*v_f;

h1 = plot(dv_s,del_x_bound1,'LineWidth',2,'Color',[0.3010 0.7450 0.9330]);
h2 = plot(dv_s,del_x_bound2,'LineWidth',2,'Color',[0.3010 0.7450 0.9330]);
h3 = plot(dv_s,del_x_bound3,'LineWidth',2,'Color',[0.3010 0.7450 0.9330]);
end
