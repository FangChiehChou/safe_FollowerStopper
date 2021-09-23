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

time_aligned = Time-Time(1);

%% compute relative acceleration
rel_accel = valid_relative_spd(2:end) - valid_relative_spd(1:end-1);

figure()
plot(Time(1:end-1),rel_accel)

figure()
plot(Time,valid_speed)
hold on
plot(Time,valid_speed+valid_relative_spd)


%% FS parameters

w = [4.5,5.25,6.0];


%% Histogram of the time headway

gap_offset = w(1)
edges = 0:0.1:2.5;

% [~,highway_s_I] = min(abs(time_aligned-450));
% [~,highway_e_I] = min(abs(time_aligned-1301));

bar_d_k = valid_relative_dist-gap_offset;
highway_headway = bar_d_k(bar_d_k>0 & valid_speed>0)./valid_speed(bar_d_k>0 & valid_speed>0);
figure()
% highway_headway = time_hedaway(highway_s_I:highway_e_I);
hist_handle = histogram(highway_headway,edges,'Normalization','probability')
xlabel('Time-gap[s]','FontSize',30)
ylabel('%','FontSize',30)
set(gca,'FontSize',30)
ytix = get(gca, 'YTick');
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
set(gcf, 'Color', 'w');
set(gcf, 'Units', 'centimeters', 'OuterPosition', [1 7 35 21]);
xlim([0 2.5])
% eval(['export_fig ','headway_hist',' -pdf']);

[M,I] = max(hist_handle.Values);


like_time_gap = hist_handle.BinEdges(I)
min(highway_headway)
mean(highway_headway(~isnan(highway_headway)))
