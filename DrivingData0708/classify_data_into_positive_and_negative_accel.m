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

%%
fig_handle2= figure()
temp_c = valid_accel;

c = exp(temp_c)./(1+exp(temp_c))-0.5;   %enhance constrast
ticks_value = [-4:1:4];
ticks_value = exp(ticks_value)./(1+exp(ticks_value))-0.5;

scatter3(valid_relative_spd,valid_relative_dist,valid_speed,40,c,'filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
xlabel('Relative speed[m/s]','FontSize',30)
ylabel('Relative distance[m]','FontSize',30)
zlabel('speed [m/s]','FontSize',30)
set(gca,'FontSize',30)
colormap(fig_handle2,brewermap([],'RdYlGn'))  %RdYlGn


hist(valid_accel)


%% highlight value < -1

[~,I] = find(valid_accel<-1);

fig_handle2= figure()
temp_c = valid_accel;

c = exp(temp_c)./(1+exp(temp_c))-0.5;   %enhance constrast
ticks_value = [-4:1:4];
ticks_value = exp(ticks_value)./(1+exp(ticks_value))-0.5;

scatter3(valid_relative_spd,valid_relative_dist,valid_speed,40,c,'filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
hold on

xlabel('Relative speed[m/s]','FontSize',30)
ylabel('Relative distance[m]','FontSize',30)
zlabel('speed [m/s]','FontSize',30)
set(gca,'FontSize',30)
colormap(fig_handle2,brewermap([],'RdYlGn'))  %RdYlGn

scatter3(valid_relative_spd(I),valid_relative_dist(I),valid_speed(I),40,[0 0 0])
