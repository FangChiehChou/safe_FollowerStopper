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

%% plot data at different speeds 
fig_handle = figure()
temp_c = accelx_highway;
c = exp(temp_c)./(1+exp(temp_c))-0.5;   %enhance constrast
% c = temp_c;
ticks_value = -4:1:4;
ticks_value = exp(ticks_value)./(1+exp(ticks_value))-0.5;

% plot_handle_driving_data = scatter(highway_rel_speed,highway_lead_dist,20,c,'filled','MarkerFaceAlpha',0.2,'MarkerEdgeAlpha',0.2);

hcb = colorbar('Ticks',ticks_value,...
         'TickLabels',{'-4','-3','-2','-1','0','1','2','3','4'});
colorTitleHandle = get(hcb,'Title');
titleString = 'accel.[m/s^2]';
set(colorTitleHandle ,'String',titleString);
colormap(fig_handle,brewermap([],'RdYlGn'))  %RdYlGn
ylim([0 100])
xlim([-10 10])
ylabel('Relative distance[m]','FontSize',30)
xlabel('Relative speed[m/s]','FontSize',30)
set(gca,'FontSize',30)
grid on
hold on
set(gcf, 'Units', 'centimeters', 'OuterPosition', [1 7 35 21]);
set(gcf, 'Color', 'w');
%%
v_follower_g = 0:1:30;
for frame_index = 2:1:length(v_follower_g)
    %extract field test data around this speed
    speed_lower_bound = v_follower_g(frame_index-1);
    speed_upper_bound = v_follower_g(frame_index);
    accel_pt_temp = accelx_highway(speed_lower_bound<=speed_highway&speed_highway<speed_upper_bound);
    color_pt_temp = c(speed_lower_bound<=speed_highway&speed_highway<speed_upper_bound);
    relative_dist_pt_temp = highway_lead_dist(speed_lower_bound<=speed_highway&speed_highway<speed_upper_bound);
    relative_spd_pt_temp = highway_rel_speed(speed_lower_bound<=speed_highway&speed_highway<speed_upper_bound);
    speed_pt_temp = speed_highway(speed_lower_bound<=speed_highway&speed_highway<speed_upper_bound);
    
    hPlotData = scatter(relative_spd_pt_temp,relative_dist_pt_temp,160,color_pt_temp,...
        'p','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
    
    v_r_boundary_handle.XData=[-v_follower_g(frame_index),-v_follower_g(frame_index)];
    v_r_boundary_handle.YData=[0,120];
    title(['speed = ', num2str(v_follower_g(frame_index-1),'%.1f'),'-', num2str(v_follower_g(frame_index),'%.1f'),'[m/s]'])
    [h1,h2,h3] = overlay_FS_curves(v_follower_g(frame_index-1));
    drawnow
%     filename = ['newFS_drivingData_',num2str(frame_index),'.jpg'];
%     saveas(gcf,filename)
    eval(['export_fig ','newFS_drivingData_',num2str(frame_index),' -pdf']);
    delete(hPlotData);
    delete(h1);
    delete(h2);
    delete(h3);
    
    
%     frame =  getframe(figure_3DsafetySet_On2D_handle) ;
%     MakeGIF(fname,frame,frame_index)
    
end



%%
y = 9;
x = -4.682;

rel_state_dis = (relative_vel-x).^2+(lead_distance-y).^2;
[~,I] = find(rel_state_dis == min(rel_state_dis));

time_aligned(I)

figure()
plot(time_aligned,lead_distance)
hold on
plot(time_aligned(I:I+10),lead_distance(I:I+10),'LineWidth',2)


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

h1 = plot(dv_s,del_x_bound1,'LineWidth',2)
h2 = plot(dv_s,del_x_bound2,'LineWidth',2)
h3 = plot(dv_s,del_x_bound3,'LineWidth',2)

end
