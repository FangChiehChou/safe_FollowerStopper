%% play driving data

clear all
clc

%% load field test data
load('../../circleData/2020-07-28-17-56-48-879520_2020_03_05_relative_distance_relative_speed_Data.mat');
load('../../circleData/2020-07-28-17-40-56-648843_2020_03_05_Acceleration_Speed_Data.mat');

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
%==========resample speed and accel==========
speed = interp1(time1,speed,time2,'nearest');
accel = interp1(time1,accel,time2,'nearest');    
%estimate speed of the lead vehicle.
speed_lead  = speed + relative_speed;
valid_relative_dist = relative_distance(speed_lead>0 & relative_distance<200 & speed>0 );
valid_relative_spd = relative_speed(speed_lead>0 & relative_distance<200&speed>0);
valid_speed = speed(speed_lead>0& relative_distance<200&speed>0);
valid_accel = accel(speed_lead>0 & relative_distance<200&speed>0);


%%
figure()
plot(time2,speed)
hold on
plot(time2,relative_distance)


figure()
scatter(valid_speed,valid_relative_dist)

time_headway = valid_relative_dist./valid_speed;

figure()
scatter(valid_speed,time_headway)
ylim([0 20])


valid_time_headway = time_headway(time_headway<10);

figure()
histogram(valid_time_headway,0:0.1:10,'Normalization', 'count')
xlabel('time headway[s]','FontSize',30)
ylabel('count','FontSize',30)
set(gca,'FontSize',30)


