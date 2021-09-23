clear all
clc

load("2020-07-08-15-15-54_2T3MWRFVXLW056972_CAN_Messages.mat")

%%
figure()
plot(Time-Time(1),speed)
xlabel('Time[s]','FontSize',30)
ylabel('Speed[km/hr]','FontSize',30)
set(gca,'FontSize',30)

figure()
plot(Time-Time(1),lead_distance)
xlabel('Time[s]','FontSize',30)
hold on
plot(Time-Time(1),relative_vel)
ylabel('Relative distance[m]','FontSize',30)
set(gca,'FontSize',30)

figure()
scatter(speed/3.6,accelx,'.')
xlabel('Speed[m/s]','FontSize',30)
ylabel('Accel.[m/s^2]','FontSize',30)
set(gca,'FontSize',30)

figure()
scatter(relative_vel,lead_distance,'.')
xlabel('Relative speed[m/s]','FontSize',30)
ylabel('Relative distance[m]','FontSize',30)
set(gca,'FontSize',30)

figure()
scatter3(relative_vel,lead_distance,accelx,'.')
xlabel('Relative speed[m/s]','FontSize',30)
ylabel('Relative distance[m]','FontSize',30)
zlabel('Accel.[m/s^2]','FontSize',30)
set(gca,'FontSize',30)

%% filter available data
valid_accel = accelx;
valid_relative_dist = lead_distance;
valid_relative_spd = relative_vel;
valid_relative_spd(lead_distance>120) = nan;
valid_speed = speed/3.6;

valid_relative_dist(valid_relative_dist>150) = nan;

%% compute relative acceleration
rel_accel = valid_relative_spd(2:end) - valid_relative_spd(1:end-1);

figure()
plot(Time(1:end-1),rel_accel)

figure()
plot(Time,valid_speed)
hold on
plot(Time,valid_speed+valid_relative_spd)

%% scenarios accodring to the video

time_aligned = Time-Time(1);
cut_out_time_vid = [478,509,565,586,602,623,640,663,1183,1251,1264];  % a vehicle cut-out 
cut_in_time_vid = [572,613,629,795,925,938,1026,1143,1178,1186];   %a vehicle cut-in in front of current vehicle
lane_change_time_vid = [489,533,565,598,663,868,876,1123,1137,1172,1254];  %subject vehicle making a lane change

index_1 = zeros(length(cut_out_time_vid),1);
index_2 = zeros(length(cut_in_time_vid),1);
index_3 = zeros(length(lane_change_time_vid),1);

for i = 1:1:length(index_1)
    [~,temp_I] = min(abs(time_aligned-cut_out_time_vid(i)));
    index_1(i) = temp_I;
end

for i = 1:1:length(index_2)  
    [~,temp_I] = min(abs(time_aligned-cut_in_time_vid(i)));
    index_2(i) = temp_I;
end

for i = 1:1:length(index_3)  
    [~,temp_I] = min(abs(time_aligned-lane_change_time_vid(i)));
    index_3(i) = temp_I;
end


figure()
plot(Time,valid_relative_dist)
xlabel('Time[s]','FontSize',30)
ylabel('Relative distance[m]','FontSize',30)
set(gca,'FontSize',30)
hold on
h1 = scatter(Time(index_1),valid_relative_dist(index_1),'*');
h2 = scatter(Time(index_2),valid_relative_dist(index_2),'^');
h3 = scatter(Time(index_3),valid_relative_dist(index_3),'+');
legend([h1 h2 h3],{'cut out','cut in','lane change'})


figure()
plot(Time-Time(1),speed)
xlabel('Time[s]','FontSize',30)
ylabel('Speed[km/hr]','FontSize',30)
set(gca,'FontSize',30)
hold on
h1 = scatter(Time(index_1)-Time(1),speed(index_1),'*');
h2 = scatter(Time(index_2)-Time(1),speed(index_2),'^');
h3 = scatter(Time(index_3)-Time(1),speed(index_3),'+');
legend([h1 h2 h3],{'cut out','cut in','lane change'})



%% find times when a target vehicle changes
speed_diff = relative_vel(2:end) - relative_vel(1:end-1);
dist_diff = lead_distance(2:end) - lead_distance(1:end-1);

cut_out_index = find(dist_diff >= 5 & abs(speed_diff)>=1);
cut_in_index = find(dist_diff <= -5 & abs(speed_diff)>=1);

figure()
plot(Time,valid_relative_dist)
xlabel('Time[s]','FontSize',30)
ylabel('Relative distance[m]','FontSize',30)
set(gca,'FontSize',30)
hold on
scatter(Time(cut_out_index),valid_relative_dist(cut_out_index),'*')
scatter(Time(cut_in_index+1),valid_relative_dist(cut_in_index+1),'^')

figure()
plot(Time,valid_relative_spd)
xlabel('Time[s]','FontSize',30)
ylabel('Relative speed[m/s]','FontSize',30)
set(gca,'FontSize',30)
hold on
scatter(Time(cut_out_index),valid_relative_spd(cut_out_index),'*')
scatter(Time(cut_in_index+1),valid_relative_spd(cut_in_index+1),'^')


%%

fig_handle2 = figure()
temp_c = accelx;

c = exp(temp_c)./(1+exp(temp_c))-0.5;   %enhance constrast
ticks_value = [-4:1:4];
ticks_value = exp(ticks_value)./(1+exp(ticks_value))-0.5;
scatter(Time-Time(1),relative_vel,40,c,'filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
xlabel('Time[s]','FontSize',30)
ylabel('Relative speed[m/s]','FontSize',30)
set(gca,'FontSize',30)
hcb = colorbar('Ticks',ticks_value,...
         'TickLabels',{'-4','-3','-2','-1','0','1','2','3','4'});
colorTitleHandle = get(hcb,'Title');
titleString = 'accel.[m/s^2]';
set(colorTitleHandle ,'String',titleString);

colormap(fig_handle2,brewermap([],'RdYlGn'))  %RdYlGn


%%

fig_handle5 = figure()
temp_c = accelx;

c = exp(temp_c)./(1+exp(temp_c))-0.5;   %enhance constrast
ticks_value = [-4:1:4];
ticks_value = exp(ticks_value)./(1+exp(ticks_value))-0.5;
scatter(Time,lead_distance,40,c,'filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
xlabel('Time[s]','FontSize',30)
ylabel('Relative distance[m]','FontSize',30)
set(gca,'FontSize',30)
hcb = colorbar('Ticks',ticks_value,...
         'TickLabels',{'-4','-3','-2','-1','0','1','2','3','4'});
colorTitleHandle = get(hcb,'Title');
titleString = 'accel.[m/s^2]';
set(colorTitleHandle ,'String',titleString);
colormap(fig_handle5,brewermap([],'RdYlGn'))  %RdYlGn
ylim([0 150])

%%
fig_handle6 = figure()
temp_c = accelx;

c = exp(temp_c)./(1+exp(temp_c))-0.5;   %enhance constrast
ticks_value = [-4:1:4];
ticks_value = exp(ticks_value)./(1+exp(ticks_value))-0.5;
scatter(Time,speed/3.6,40,c,'filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
xlabel('Time[s]','FontSize',30)
ylabel('Speed[m/s]','FontSize',30)
set(gca,'FontSize',30)
hcb = colorbar('Ticks',ticks_value,...
         'TickLabels',{'-4','-3','-2','-1','0','1','2','3','4'});
colorTitleHandle = get(hcb,'Title');
titleString = 'accel.[m/s^2]';
set(colorTitleHandle ,'String',titleString);
colormap(fig_handle6,brewermap([],'RdYlGn'))  %RdYlGn



%%
fig_handle3 = figure(3)
temp_c = accelx;
c = exp(temp_c)./(1+exp(temp_c))-0.5;   %enhance constrast
% c = temp_c;
ticks_value = [-4:1:4];
ticks_value = exp(ticks_value)./(1+exp(ticks_value))-0.5;
% plot_handle = scatter(valid_relative_dist,valid_relative_spd,80,c,'.','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
plot_handle = scatter(relative_vel,lead_distance,40,c,'filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);

hcb = colorbar('Ticks',ticks_value,...
         'TickLabels',{'-4','-3','-2','-1','0','1','2','3','4'});
colorTitleHandle = get(hcb,'Title');
titleString = 'accel.[m/s^2]';
set(colorTitleHandle ,'String',titleString);
colormap(fig_handle3,brewermap([],'RdYlGn'))  %RdYlGn
ylim([0 120])
xlim([-15 15])
ylabel('Relative distance[m]','FontSize',30)
xlabel('Relative speed[m/s]','FontSize',30)
set(gca,'FontSize',30)

grid on
speed_grid = 0:0.5:30;
speed_grid = [speed_grid,30:-0.5:0 ];

%%
% fname = 'field_test_data.gif';

for frame_index = 1:1:length(speed_grid)
    speed_lower_bound = speed_grid(frame_index)-1;
    speed_upper_bound = speed_grid(frame_index)+1;
    accel_pt_temp = valid_accel(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    relative_dist_pt_temp = valid_relative_dist(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    relative_spd_pt_temp = valid_relative_spd(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    
%     plot_handle.XData = relative_dist_pt_temp;
%     plot_handle.YData = relative_spd_pt_temp;
    plot_handle.XData = relative_spd_pt_temp;
    plot_handle.YData = relative_dist_pt_temp;    
    plot_handle.CData = exp(accel_pt_temp)./(1+exp(accel_pt_temp))-0.5;
    title(['speed = ', num2str(speed_grid(frame_index),'%.1f'), '[m/s]'])
    drawnow
    pause
%     frame =  getframe(fig_handle3) ;
%     MakeGIF(fname,frame,frame_index)

end


%% 




%% Histogram of the time headway

edges = 0:0.1:2.5;

[~,highway_s_I] = min(abs(time_aligned-450));
[~,highway_e_I] = min(abs(time_aligned-1301));
time_hedaway = valid_relative_dist./valid_speed;
figure()
highway_headway = time_hedaway(highway_s_I:highway_e_I);
histogram(highway_headway,edges,'Normalization','probability')
xlabel('Time-gap[s]','FontSize',30)
ylabel('%','FontSize',30)
set(gca,'FontSize',30)
ytix = get(gca, 'YTick');
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
set(gcf, 'Color', 'w');
set(gcf, 'Units', 'centimeters', 'OuterPosition', [1 7 35 21]);
xlim([0 2.5])
eval(['export_fig ','headway_hist',' -pdf']);


min(highway_headway)
mean(highway_headway(~isnan(highway_headway)))

%%
figure()
% yyaxis left
plot(Time(highway_s_I:highway_e_I)-Time(highway_s_I),speed(highway_s_I:highway_e_I)/3.6,'LineWidth',2)
% ylabel('Speed[m/s]','FontSize',30)
hold on
plot(Time(highway_s_I:highway_e_I)-Time(highway_s_I),valid_relative_dist(highway_s_I:highway_e_I),'LineWidth',2)
% ylabel('Relative distance [m]','FontSize',30)
xlabel('Time[s]','FontSize',30)
set(gca,'FontSize',30)
% hold on
% h1 = scatter(Time(index_1)-Time(1),speed(index_1),'*');
% h2 = scatter(Time(index_2)-Time(1),speed(index_2),'^');
% h3 = scatter(Time(index_3)-Time(1),speed(index_3),'+');
% legend([h1 h2 h3],{'cut out','cut in','lane change'})
legend('Speed[m/s]','Relative distance[m]')
xlim([0 850.7])
set(gcf, 'Color', 'w');
set(gcf, 'Units', 'centimeters', 'OuterPosition', [1 7 35 21]);
eval(['export_fig ','Speed_Space_highway',' -pdf']);



