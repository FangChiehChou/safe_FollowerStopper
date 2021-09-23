%% write a script that shows a heat map of the FS and human driving data and IDM...

clear all
clc

%% load human driving data

load("2020-07-08-15-15-54_2T3MWRFVXLW056972_CAN_Messages.mat")
valid_accel = accelx;
valid_relative_dist = lead_distance;
valid_relative_spd = relative_vel;
valid_relative_spd(lead_distance>120) = nan;
valid_speed = speed/3.6;

lead_distance(lead_distance>120) = nan;

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

%%
figure()
scatter(highway_rel_speed,highway_lead_dist,160,accelx_highway,...
        'p','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);

xlabel('Relative speed [m/s]')
ylabel('Relative distance [m]')
cm = get(gca,'Colormap');
color_bar_handle = colorbar
ylabel(color_bar_handle, 'Acceleration[m/s^2]')

top = max(accelx_highway);
bottom = min(accelx_highway);

%% plot human driving data at different speed 
v_AV_grid = 0:2:30;

temp_c = accelx_highway;
c = exp(temp_c)./(1+exp(temp_c))-0.5;   %enhance constrast
c = temp_c;

fig_handle1 = figure();
% colormap(fig_handle1,cm)
% colormap(fig_handle1,brewermap([],'RdYlGn'))  %RdYlGn
hold on
for AV_speed_index = 1:1:length(v_AV_grid)
    %extract field test data around this speed
    speed_lower_bound = v_AV_grid(AV_speed_index)-1;
    speed_upper_bound = v_AV_grid(AV_speed_index)+1;
    accel_pt_temp = accelx_highway(speed_lower_bound<=speed_highway&speed_highway<speed_upper_bound);
    color_pt_temp = c(speed_lower_bound<=speed_highway&speed_highway<speed_upper_bound);
    relative_dist_pt_temp = highway_lead_dist(speed_lower_bound<=speed_highway&speed_highway<speed_upper_bound);
    relative_spd_pt_temp = highway_rel_speed(speed_lower_bound<=speed_highway&speed_highway<speed_upper_bound);
    speed_pt_temp = speed_highway(speed_lower_bound<=speed_highway&speed_highway<speed_upper_bound);
    
    temp_v_AV = v_AV_grid(AV_speed_index);
    subplot(4,4,AV_speed_index)
    
    title(['Speed = ',num2str(temp_v_AV),'[m/s]'])
    hold on
    scatter(relative_spd_pt_temp +temp_v_AV,relative_dist_pt_temp,100,accel_pt_temp,...
        'p','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);

    xlabel('Leader speed [m/s]')
    ylabel('Relative distance [m]')
    set(gca,'FontSize',10)
    grid on
    xlim([0 30])
    ylim([0 80])
    
    caxis manual
    caxis([bottom top]);
    color_bar_handle = colorbar
    ylabel(color_bar_handle, 'Acceleration[m/s^2]')
end
set(gcf, 'Color', 'w');


%% heatmap of the modified FollowerStopper

uMin = -3;
uMax = 1.5;
r = 30;

v_lead_grid = 0:1:30;
rel_dist_grid = 0:1:80;

v_AV_grid = 0:2:30;

[XX,YY] = meshgrid(v_lead_grid,rel_dist_grid);

fig_handle2 = figure();

for AV_speed_index = 1:1:length(v_AV_grid)

    temp_v_AV = v_AV_grid(AV_speed_index);
    v_AV = temp_v_AV*ones(size(XX,1),size(XX,2));
    
    d_rel = YY;
    v_rel = XX-v_AV;
    
    accel_value = dyn_modified_follower_stopper(d_rel,v_rel,v_AV,uMin,uMax,r);
    subplot(4,4,AV_speed_index)
    
    title(['Speed = ',num2str(temp_v_AV),'[m/s]'])
    hold on
    contourf(XX,YY,accel_value)
    xlabel('Leader speed [m/s]')
    ylabel('Relative distance [m]')
    set(gca,'FontSize',10)
    caxis manual
    caxis([bottom top]);
    
    xlim([0 30]);
    ylim([0 80]);
    color_bar_handle = colorbar
    ylabel(color_bar_handle, 'Acceleration[m/s^2]')

%     pause
end
cm2 = get(gca,'Colormap');
colormap(fig_handle2,cm)
set(gcf, 'Color', 'w');
% colormap(fig_handle,brewermap([],'RdYlGn'))  %RdYlGn
%% heatmap of the FollowerStopper
fig_handle3 = figure();
for AV_speed_index = 1:1:length(v_AV_grid)

    temp_v_AV = v_AV_grid(AV_speed_index);
    v_AV = temp_v_AV*ones(size(XX,1),size(XX,2));
    
    d_rel = YY;
    v_rel = XX-v_AV;
    
    accel_value = dyn_follower_stopper(d_rel,v_rel,v_AV,uMin,uMax,r);
    
    subplot(4,4,AV_speed_index)
    title(['Speed = ',num2str(temp_v_AV),'[m/s]'])
    hold on
    contourf(XX,YY,accel_value)
    xlabel('Leader speed [m/s]')
    ylabel('Relative distance [m]')
    set(gca,'FontSize',10)
    caxis manual
    caxis([bottom top]);
    color_bar_handle = colorbar
    ylabel(color_bar_handle, 'Acceleration[m/s^2]')
    xlim([0 30]);
    ylim([0 80]);
%     pause
end
colormap(fig_handle3,cm)
set(gcf, 'Color', 'w');

%% load driving data 03052020
load('../../circleData/2020-07-28-17-56-48-879520_2020_03_05_relative_distance_relative_speed_Data.mat');
load('../../circleData/2020-07-28-17-40-56-648843_2020_03_05_Acceleration_Speed_Data.mat');

%%
time1 = speed_accel(:,1);
% speed = speed_accel(:,2);
% accel = speed_accel(:,3);

time2 = relative_distance_relative_speed(:,1);
% relative_distance = relative_distance_relative_speed(:,2);
% relative_speed = relative_distance_relative_speed(:,3);

[B,I] = sort(time1); 
time1 = B;
speed = speed_accel(I,2);
accel = speed_accel(I,3);

[B,I] = sort(time2); 
time2 = B;
relative_distance = relative_distance_relative_speed(I,2);
relative_speed = relative_distance_relative_speed(I,3);
%% resample speed and accel
speed = interp1(time1,speed,time2,'nearest');
accel = interp1(time1,accel,time2,'nearest');    

%estimate speed of the lead vehicle.
speed_lead  = speed + relative_speed;
relative_dist_data0 = relative_distance(speed_lead>0 & relative_distance<100);
relative_spd_data0 = relative_speed(speed_lead>0 & relative_distance<100);
speed_data0 = speed(speed_lead>0& relative_distance<100);
accel_data0 = accel(speed_lead>0 & relative_distance<100);

time_headway_data1 = relative_dist_data0./speed_data0;
time_headway_data1 = time_headway_data1(speed_data0>1);

edges = [0:0.2:2.5];
hist(time_headway_data1,edges)

figure()
plot(time2,speed)

figure()
plot(time2,accel)


relative_dist_data0 = [relative_dist_data0;highway_lead_dist'];
relative_spd_data0 = [relative_spd_data0;highway_rel_speed'];
speed_data0 = [speed_data0;speed_highway'];
accel_data0 = [accel_data0;accelx_highway'];



%% get zero acceleration data points ( accel -0.1~0.1)  

relative_dist_data1 = relative_dist_data0(accel_data0> 0.5  | accel_data0<-0.5 );
relative_spd_data1 = relative_spd_data0(accel_data0> 0.5  | accel_data0< -0.5 );
speed_data1 = speed_data0(accel_data0> 0.5 | accel_data0<-0.5 );
accel_data1 = accel_data0(accel_data0> 0.5 | accel_data0<-0.5 );
%% plot human driving data at different speed 
v_AV_grid = 0:2:30;

temp_c = accel_data0;
c = exp(temp_c)./(1+exp(temp_c))-0.5;   %enhance constrast
c = temp_c;

fig_handle6 = figure();
% colormap(fig_handle1,cm)
% colormap(fig_handle1,brewermap([],'RdYlGn'))  %RdYlGn
hold on
for AV_speed_index = 1:1:length(v_AV_grid)
    %extract field test data around this speed
    speed_lower_bound = v_AV_grid(AV_speed_index)-1;
    speed_upper_bound = v_AV_grid(AV_speed_index)+1;
    accel_pt_temp = accel_data0(speed_lower_bound<=speed_data0&speed_data0<speed_upper_bound);
    color_pt_temp = c(speed_lower_bound<=speed_data0&speed_data0<speed_upper_bound);
    relative_dist_pt_temp = relative_dist_data0(speed_lower_bound<=speed_data0&speed_data0<speed_upper_bound);
    relative_spd_pt_temp = relative_spd_data0(speed_lower_bound<=speed_data0&speed_data0<speed_upper_bound);
    speed_pt_temp = speed_data0(speed_lower_bound<=speed_data0&speed_data0<speed_upper_bound);
    
    temp_v_AV = v_AV_grid(AV_speed_index);
    subplot(4,4,AV_speed_index)
    
    title(['Speed = ',num2str(temp_v_AV),'[m/s]'])
    hold on
    scatter(relative_spd_pt_temp +temp_v_AV,relative_dist_pt_temp,100,accel_pt_temp,...
        'p','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);

    xlabel('Leader speed [m/s]')
    ylabel('Relative distance [m]')
    set(gca,'FontSize',10)
    grid on
    xlim([0 30])
    ylim([0 80])
    
    caxis manual
    caxis([bottom top]);
    color_bar_handle = colorbar
    ylabel(color_bar_handle, 'Acceleration[m/s^2]')
    
end
set(gcf, 'Color', 'w');

%% modified FollowerStopper and human driving data

uMin = -3;
uMax = 1.5;
r = 30;

v_lead_grid = 0:1:30;
rel_dist_grid = 0:1:80;

v_AV_grid = 0:2:30;

[XX,YY] = meshgrid(v_lead_grid,rel_dist_grid);

fig_handle2 = figure(8);

for AV_speed_index = 1:1:length(v_AV_grid)

    %extract field test data around this speed
    speed_lower_bound = v_AV_grid(AV_speed_index)-1;
    speed_upper_bound = v_AV_grid(AV_speed_index)+1;
    accel_pt_temp = accel_data1(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound & accel_data1<0);
    color_pt_temp = c(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound& accel_data1<0);
    relative_dist_pt_temp = relative_dist_data1(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound& accel_data1<0);
    relative_spd_pt_temp = relative_spd_data1(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound& accel_data1<0);
    speed_pt_temp = speed_data1(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound& accel_data1<0);
    
    temp_v_AV = v_AV_grid(AV_speed_index);
    v_AV = temp_v_AV*ones(size(XX,1),size(XX,2));
    
    d_rel = YY;
    v_rel = XX-v_AV;
    
    accel_value = dyn_modified_follower_stopper(d_rel,v_rel,v_AV,uMin,uMax,r);
    subplot(4,4,AV_speed_index)
    
    title(['Speed = ',num2str(temp_v_AV),'[m/s]'])
    hold on
    contourf(XX,YY,accel_value)
    
    scatter(relative_spd_pt_temp +temp_v_AV,relative_dist_pt_temp,100,accel_pt_temp,...
        'p','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
    xlabel('Leader speed [m/s]')
    ylabel('Relative distance [m]')
    
    caxis manual
    caxis([bottom top]);
    color_bar_handle = colorbar
    ylabel(color_bar_handle, 'Acceleration[m/s^2]')
    xlim([0 30])
    ylim([0 80])
    set(gca,'FontSize',10)
%     pause
end

set(gcf, 'Color', 'w');

fig_handle2.Position = [1 41 1920 963]
export_fig modified_FollowerStopper_with_human_driving_negative_accel -eps

%% modified FollowerStopper and human driving data

uMin = -3;
uMax = 1.5;
r = 30;

v_lead_grid = 0:1:30;
rel_dist_grid = 0:1:80;

v_AV_grid = 0:2:30;

[XX,YY] = meshgrid(v_lead_grid,rel_dist_grid);

fig_handle2 = figure(9);

for AV_speed_index = 1:1:length(v_AV_grid)

    %extract field test data around this speed
    speed_lower_bound = v_AV_grid(AV_speed_index)-1;
    speed_upper_bound = v_AV_grid(AV_speed_index)+1;
    accel_pt_temp = accel_data1(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound & accel_data1>0);
    color_pt_temp = c(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound& accel_data1>0);
    relative_dist_pt_temp = relative_dist_data1(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound& accel_data1>0);
    relative_spd_pt_temp = relative_spd_data1(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound& accel_data1>0);
    speed_pt_temp = speed_data1(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound& accel_data1>0);
    
    temp_v_AV = v_AV_grid(AV_speed_index);
    v_AV = temp_v_AV*ones(size(XX,1),size(XX,2));
    
    d_rel = YY;
    v_rel = XX-v_AV;
    
    accel_value = dyn_modified_follower_stopper(d_rel,v_rel,v_AV,uMin,uMax,r);
    subplot(4,4,AV_speed_index)
    
    title(['Speed = ',num2str(temp_v_AV),'[m/s]'])
    hold on
    contourf(XX,YY,accel_value)
    
    scatter(relative_spd_pt_temp +temp_v_AV,relative_dist_pt_temp,100,accel_pt_temp,...
        'p','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
    xlabel('Leader speed [m/s]')
    ylabel('Relative distance [m]')
    
    caxis manual
    caxis([bottom top]);
    color_bar_handle = colorbar
    ylabel(color_bar_handle, 'Acceleration[m/s^2]')
    xlim([0 30])
    ylim([0 80])
    set(gca,'FontSize',10)
%     pause
end
set(gcf, 'Color', 'w');

fig_handle2.Position = [1 41 1920 963]
export_fig modified_FollowerStopper_with_human_driving_positive_accel -eps

%% FollowerStopper and human driving data

figure(10)
for AV_speed_index = 1:1:length(v_AV_grid)

            %extract field test data around this speed
    speed_lower_bound = v_AV_grid(AV_speed_index)-1;
    speed_upper_bound = v_AV_grid(AV_speed_index)+1;
    accel_pt_temp = accel_data1(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound & accel_data1>0);
    color_pt_temp = c(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound& accel_data1>0);
    relative_dist_pt_temp = relative_dist_data1(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound& accel_data1>0);
    relative_spd_pt_temp = relative_spd_data1(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound& accel_data1>0);
    speed_pt_temp = speed_data1(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound& accel_data1>0);
    
    
    temp_v_AV = v_AV_grid(AV_speed_index);
    v_AV = temp_v_AV*ones(size(XX,1),size(XX,2));
    
    d_rel = YY;
    v_rel = XX-v_AV;
    
    accel_value = dyn_follower_stopper(d_rel,v_rel,v_AV,uMin,uMax,r);
    
    subplot(4,4,AV_speed_index)
    title(['Speed = ',num2str(temp_v_AV),'[m/s]'])
    hold on
    contourf(XX,YY,accel_value)
    scatter(relative_spd_pt_temp +temp_v_AV,relative_dist_pt_temp,100,accel_pt_temp,...
        'p','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
    xlabel('Leader speed [m/s]')
    ylabel('Relative distance [m]')
    
    caxis manual
    caxis([bottom top]);
    color_bar_handle = colorbar
    ylabel(color_bar_handle, 'Acceleration[m/s^2]')
    xlim([0 30])
    ylim([0 80])
    set(gca,'FontSize',10)
%     pause
end
set(gcf, 'Color', 'w');
fig_handle = gcf
fig_handle.Position = [1 41 1920 963]
export_fig FollowerStopper_with_human_driving_positive_accel -eps

figure(11)
for AV_speed_index = 1:1:length(v_AV_grid)

            %extract field test data around this speed
    speed_lower_bound = v_AV_grid(AV_speed_index)-1;
    speed_upper_bound = v_AV_grid(AV_speed_index)+1;
    accel_pt_temp = accel_data1(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound & accel_data1<0);
    color_pt_temp = c(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound& accel_data1<0);
    relative_dist_pt_temp = relative_dist_data1(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound& accel_data1<0);
    relative_spd_pt_temp = relative_spd_data1(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound& accel_data1<0);
    speed_pt_temp = speed_data1(speed_lower_bound<=speed_data1&speed_data1<speed_upper_bound& accel_data1<0);
    
    
    temp_v_AV = v_AV_grid(AV_speed_index);
    v_AV = temp_v_AV*ones(size(XX,1),size(XX,2));
    
    d_rel = YY;
    v_rel = XX-v_AV;
    
    accel_value = dyn_follower_stopper(d_rel,v_rel,v_AV,uMin,uMax,r);
    
    subplot(4,4,AV_speed_index)
    title(['Speed = ',num2str(temp_v_AV),'[m/s]'])
    hold on
    contourf(XX,YY,accel_value)
    scatter(relative_spd_pt_temp +temp_v_AV,relative_dist_pt_temp,100,accel_pt_temp,...
        'p','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
    xlabel('Leader speed [m/s]')
    ylabel('Relative distance [m]')
    
    caxis manual
    caxis([bottom top]);
    color_bar_handle = colorbar
    ylabel(color_bar_handle, 'Acceleration[m/s^2]')
    xlim([0 30])
    ylim([0 80])
    set(gca,'FontSize',10)
%     pause
end
set(gcf, 'Color', 'w');
fig_handle = gcf
fig_handle.Position = [1 41 1920 963]
export_fig FollowerStopper_with_human_driving_negative_accel -eps


%% Compute the loss of the modified FollowerStopper and the FollowerStopper

d_rel_data = relative_dist_data0;
v_rel_data = relative_spd_data0;
v_AV_data = speed_data0;

accel_value_mFS = dyn_modified_follower_stopper(d_rel_data,v_rel_data,v_AV_data,uMin,uMax,r);
square_error_loss_mFS = norm(accel_data0 - accel_value_mFS);

reward_function_mFS = accel_data0.*accel_value_mFS;

reward_function_mFS(reward_function_mFS>1) = 1;
reward_function_mFS(reward_function_mFS<0) = 0;

sum(reward_function_mFS)

accel_value_FS = dyn_follower_stopper(d_rel_data,v_rel_data,v_AV_data,uMin,uMax,r);
square_error_loss_FS = norm(accel_data0 - accel_value_FS);

reward_function_FS = accel_data0.*accel_value_FS;

reward_function_FS(reward_function_FS>1) = 1;
reward_function_FS(reward_function_FS<0) = 0;

sum(reward_function_FS)

figure()
plot(accel_value_mFS)
hold on
plot(accel_value_FS)
plot(accel_data0)

figure()
subplot(3,1,1)
hist(accel_data0,-3:0.1:1.5)

subplot(3,1,2)
hist(accel_value_mFS)

subplot(3,1,3)
hist(accel_value_FS)

