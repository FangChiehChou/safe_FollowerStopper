%% write a script to 1. replay traces of driving data and 2. overlaying the safety envelops of the FS

clear all 
clc


%% load driving data and preprocess

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
time_highway = time_aligned(highway_s_I:highway_e_I);


%% iterate through data points and plot them
% color_bar_top = max(accelx_highway);
% color_bar_bottom = min(accelx_highway);
color_bar_top = 4.5;
color_bar_bottom = -4.5;

v_rel_grid = -10:1:10;
speed = 0; 
[envelope1,envelope2,envelope3] = m_FS_evnvelopes_on_2D(speed,v_rel_grid);

fname = 'humandrivingData_mFS_envs.gif';

figure_handle1 = figure()
handle_FS_env1 = plot(v_rel_grid,envelope1,'Color','k','LineWidth',2);
hold on
handle_FS_env2 = plot(v_rel_grid,envelope2,'Color','k','LineWidth',2);
handle_FS_env3 = plot(v_rel_grid,envelope3,'Color','k','LineWidth',2);

xlabel('Relative speed [m/s]')
ylabel('Relative distance [m]')
set(gca,'FontSize',10)
grid on

ylim([0 80])
xlim([-10 10])

%doing this to force color map linearly to acceleraion value 
temp_scatter = scatter(zeros(100,1),zeros(100,1),100,linspace(color_bar_top,color_bar_bottom,100),...
    'o','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
colormap(figure_handle1,bluewhitered(256))
% colormap(figure_handle1,jet(256))
delete(temp_scatter)

temp_scatter_h = scatter(highway_rel_speed,highway_lead_dist,100,accelx_highway,...
    'o','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);

caxis manual
caxis([color_bar_bottom color_bar_top]);
colorbar

delete(temp_scatter_h)

sub_set_length = 10;
sub_set_for_scatter.rel_speed = highway_rel_speed(1:sub_set_length);
sub_set_for_scatter.rel_distance = highway_lead_dist(1:sub_set_length);
sub_set_for_scatter.accel = accelx_highway(1:sub_set_length);
sub_set_for_scatter.time = time_highway(1:sub_set_length);

% fading_by_time = (sub_set_for_scatter.time-sub_set_for_scatter.time(1))/(sub_set_for_scatter.time(end)- sub_set_for_scatter.time(1));

data_point_h = scatter(sub_set_for_scatter.rel_speed,sub_set_for_scatter.rel_distance,160,sub_set_for_scatter.accel,...
    'o','filled','MarkerFaceAlpha',1,'MarkerEdgeAlpha',1.0);

data_point_h2 = plot(sub_set_for_scatter.rel_speed,sub_set_for_scatter.rel_distance,'-*');

for i = 1:1:length(time_highway)-sub_set_length
    
    temp_time = time_highway(i);
    
%     scatter(highway_rel_speed(i),highway_lead_dist(i),160,accelx_highway(i),...
%         'o','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);

    sub_set_for_scatter.rel_speed = highway_rel_speed(i+1:i+sub_set_length);
    sub_set_for_scatter.rel_distance = highway_lead_dist(i+1:i+sub_set_length);
    sub_set_for_scatter.accel = accelx_highway(i+1:i+sub_set_length);
    sub_set_for_scatter.time = time_highway(i+1:i+sub_set_length);
%     fading_by_time = (sub_set_for_scatter.time-sub_set_for_scatter.time(1))/(sub_set_for_scatter.time(end)- sub_set_for_scatter.time(1));
    
    data_point_h.XData= sub_set_for_scatter.rel_speed;
    data_point_h.YData= sub_set_for_scatter.rel_distance;
    data_point_h.CData= sub_set_for_scatter.accel;
%     data_point_h.MarkerFaceAlpha= fading_by_time;

    data_point_h2.XData= sub_set_for_scatter.rel_speed;
    data_point_h2.YData= sub_set_for_scatter.rel_distance;

    temp_speed = speed_highway(i);
    
    [envelope1,envelope2,envelope3] = m_FS_evnvelopes_on_2D(temp_speed,v_rel_grid);

    handle_FS_env1.YData = envelope1;
    handle_FS_env2.YData = envelope2;
    handle_FS_env3.YData = envelope3;
    
    title(['speed = ',num2str(temp_speed)])
    
    drawnow 
    
    frame =  getframe(figure_handle1) ;
    frame_index = i;
    MakeGIF(fname,frame,frame_index,0.01)
end


%% help function --- modified FS envelopes
function [envelope1,envelope2,envelope3] = m_FS_evnvelopes_on_2D(speed,v_rel_grid)
    delta0_x1 = 4.5;
    delta0_x2 = 5.25;
    delta0_x3 = 6.0;
    
    d_1 = 1.5;
    d_2 = 1.0;
    d_3 = 0.5;
    
    h_1 = 0.4;
    h_2 = 0.6;
    h_3 = 0.8;

    envelope1 = zeros(size(v_rel_grid,1),size(v_rel_grid,2));
    envelope2 = zeros(size(v_rel_grid,1),size(v_rel_grid,2));
    envelope3 = zeros(size(v_rel_grid,1),size(v_rel_grid,2));
 
    for j = 1:1:length(v_rel_grid)
        v_f = speed;

        dv_ss = v_rel_grid(j);
        dv_ss(dv_ss>=0) = 0;          

        del_x_bound1 =  (dv_ss.^2)/(2*d_1) + max(delta0_x1,h_1*v_f);
        del_x_bound2 =  (dv_ss.^2)/(2*d_2) + max(delta0_x2,h_2*v_f);
        del_x_bound3 =  (dv_ss.^2)/(2*d_3) + max(delta0_x3,h_3*v_f);
        envelope1(j) =  del_x_bound1;
        envelope2(j) =  del_x_bound2;
        envelope3(j) =  del_x_bound3;
    end

end
    