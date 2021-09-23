clear all
clc

load("2020-07-08-15-15-54_2T3MWRFVXLW056972_CAN_Messages.mat")

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


%% scatter relative states
time_aligned = Time-Time(1);
time_enter = 430;
time_exit = 1301;
[~,highway_s_I] = min(abs(time_aligned-time_enter));
[~,highway_e_I] = min(abs(time_aligned-time_exit));

fig_handle3 = figure(10)
temp_c = accelx;
c = exp(temp_c)./(1+exp(temp_c))-0.5;   %enhance constrast
% c = temp_c;
ticks_value = [-4:1:4];
ticks_value = exp(ticks_value)./(1+exp(ticks_value))-0.5;
% plot_handle = scatter(valid_relative_dist,valid_relative_spd,80,c,'.','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
plot_handle = scatter(relative_vel(highway_s_I:highway_e_I),lead_distance(highway_s_I:highway_e_I),40,c(highway_s_I:highway_e_I),'filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);

hcb = colorbar('Ticks',ticks_value,...
         'TickLabels',{'-4','-3','-2','-1','0','1','2','3','4'});
colorTitleHandle = get(hcb,'Title');
titleString = 'accel.[m/s^2]';
set(colorTitleHandle ,'String',titleString);
colormap(fig_handle3,brewermap([],'RdYlGn'))  %RdYlGn
ylim([0 100])
xlim([-15 15])
ylabel('Relative distance[m]','FontSize',30)
xlabel('Relative speed[m/s]','FontSize',30)
set(gca,'FontSize',30)
hold on
grid on

%% overlay FS curves
dv_s = -10:0.1:10;
dv_ss = dv_s;
dv_ss(dv_ss>=0) = 0;

delta0_x1 = 4.5;
delta0_x2 = 5.25;
delta0_x3 = 6.0;
d_1 = 1.5;
d_2 = 1.0;
d_3 = 0.5;

del_x_bound1 =  delta0_x1 + (dv_ss.^2)/(2*d_1);
del_x_bound2 =  delta0_x2 + (dv_ss.^2)/(2*d_2);
del_x_bound3 =  delta0_x3 + (dv_ss.^2)/(2*d_3);


speed_grid = 0:0.5:30;
speed_grid = [speed_grid,30:-0.5:0 ];

h1 = plot(dv_s,del_x_bound1,'LineWidth',2)
h2 = plot(dv_s,del_x_bound2,'LineWidth',2)
h3 = plot(dv_s,del_x_bound3,'LineWidth',2)
grid on
ylabel('Gap [m]')
xlabel('Relative speed [m/s]')
legend([h1 h2 h3],{'decel = 1.5','decel = 1.0','decel = 0.5'})


%% distribution of deceleration
figure()
histogram(accelx(highway_s_I:highway_e_I));

%%
accelx_highway = accelx(highway_s_I:highway_e_I);
highway_rel_speed = relative_vel(highway_s_I:highway_e_I);
highway_lead_dist = lead_distance(highway_s_I:highway_e_I);
speed_highway = valid_speed(highway_s_I:highway_e_I);


figure()
scatter(highway_rel_speed(accelx_highway>0),highway_lead_dist(accelx_highway>0),'filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
ylabel('Relative distance[m]','FontSize',30)
xlabel('Relative speed[m/s]','FontSize',30)
set(gca,'FontSize',30)
hold on
grid on
ylim([0 100])
xlim([-10 10])


figure()
scatter(speed_highway,accelx_highway,'filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
ylabel('Accel[m/s^2]','FontSize',30)
xlabel('Speed[m/s]','FontSize',30)
set(gca,'FontSize',30)
hold on
grid on
ylim([0 100])
xlim([-10 10])


%%
decel_bin = [0,-1.0,-2.0,-4.5];
%find data points with decel between decel_bin(i) and decel_bin(i+1)
figure()
for i = 1:1:length(decel_bin)-1
    scatter(highway_rel_speed(accelx_highway>decel_bin(i+1)&accelx_highway<decel_bin(i)),highway_lead_dist(accelx_highway>decel_bin(i+1)&accelx_highway<decel_bin(i)),'filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
    ylabel('Relative distance[m]','FontSize',30)
    xlabel('Relative speed[m/s]','FontSize',30)
    set(gca,'FontSize',30)
    hold on
    grid on
    ylim([0 100])
    xlim([-10 10])
    pause
end




