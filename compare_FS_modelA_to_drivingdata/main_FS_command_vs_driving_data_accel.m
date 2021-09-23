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

lead_speed = speed_highway + highway_rel_speed;

figure()
plot(time_aligned(highway_s_I:highway_e_I),highway_rel_speed)
hold on
plot(time_aligned(highway_s_I:highway_e_I),highway_lead_dist)

%% compare the speed command of the FollowerStopper and compare to leading vehicle speed
% compute acceleration command of the FollwerStopper and compare to the subject vehicle accelearation

v_des_FS = zeros(length(highway_lead_dist),1);

v_des_mFS = zeros(length(highway_lead_dist),1);

accelx_FS = zeros(length(highway_lead_dist),1);

accelx_mFS = zeros(length(highway_lead_dist),1);

r = 30;
% uMax = max(accelx);   %+1.5 m/s^2
% uMin = min(accelx);   %-3.0 m/s^2

uMax = 1.5;   %+1.5 m/s^2
uMin = -3.0;   %-3.0 m/s^2

for i = 1:1:length(highway_lead_dist)
    if(highway_lead_dist(i)>100)
       v_des_FS(i) = nan;
       v_des_mFS(i) = nan;
       accelx_FS(i) = nan;
       accelx_mFS(i) = nan;
       continue; 
    end
    
    v_des = follower_stopper(highway_lead_dist(i),highway_rel_speed(i),speed_highway(i),r);
    v_des_FS(i) = v_des;
    
    v_des_mFS(i) = modified_FollowerStopper(highway_lead_dist(i),highway_rel_speed(i),speed_highway(i),r);
    
    accelx_FS(i) = dyn_follower_stopper(highway_lead_dist(i),highway_rel_speed(i),speed_highway(i),uMin,uMax,r);
    accelx_mFS(i) = dyn_modified_follower_stopper(highway_lead_dist(i),highway_rel_speed(i),speed_highway(i),uMin,uMax,r);
end

figure()
plot(time_aligned(highway_s_I:highway_e_I),lead_speed)
hold on
plot(time_aligned(highway_s_I:highway_e_I),v_des_FS,'LineWidth',2)
plot(time_aligned(highway_s_I:highway_e_I),v_des_mFS,'LineWidth',2)
ylabel('speed[m/s]')
xlabel('Time[s]')
legend('human','FS','mFS')
set(gca,'FontSize',30)

figure()
plot(time_aligned(highway_s_I:highway_e_I),accelx_highway)
hold on
plot(time_aligned(highway_s_I:highway_e_I),accelx_FS,'LineWidth',2)
plot(time_aligned(highway_s_I:highway_e_I),accelx_mFS,'-.','LineWidth',2)
ylabel('accel[m/s^2]')
xlabel('Time[s]')
legend('human','FS','mFS')
set(gca,'FontSize',30)





