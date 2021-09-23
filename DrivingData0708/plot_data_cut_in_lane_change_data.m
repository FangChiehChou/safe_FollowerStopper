close all
clear all
clc

load("2020-07-08-15-15-54_2T3MWRFVXLW056972_CAN_Messages.mat")

%% filter available data
valid_accel = accelx;
valid_relative_dist = lead_distance;
valid_relative_spd = relative_vel;
valid_speed = speed/3.6;

valid_relative_dist(valid_relative_dist>150) = nan;


%% scenarios accodring to the video

time_aligned = Time-Time(1);
cut_out_time_vid = [478,509,565,586,602,623,640,663,1183,1251,1264];  % a vehicle cut-out 
cut_in_time_vid = [572,613,629,795,925,938,1026,1143,1178,1186];   %a vehicle cut-in in front of current vehicle
lane_change_time_vid = [489,533,565,598,663,868,876,1123,1137,1172,1254];  %subject vehicle making a lane change

index_1 = zeros(length(cut_out_time_vid),1);
index_cut_in_s = zeros(length(cut_in_time_vid),1);
index_3 = zeros(length(lane_change_time_vid),1);

cut_in_s_e = zeros(length(cut_in_time_vid),2);
cut_out_s_e = zeros(length(cut_out_time_vid),2);
lane_change_s_e=  zeros(length(lane_change_time_vid),2);


for i = 1:1:length(index_1)
    [~,temp_I] = min(abs(time_aligned-cut_out_time_vid(i)));
    
    index_1(i) = temp_I;
    
    cut_out_s_e(i,1) = temp_I;
    [~,temp_I] = min(abs(time_aligned-cut_out_time_vid(i)-10));
    cut_out_s_e(i,2) = temp_I;
end

for i = 1:1:length(index_cut_in_s)  
    [~,temp_I] = min(abs(time_aligned-cut_in_time_vid(i)));
    index_cut_in_s(i) = temp_I;
    
    cut_in_s_e(i,1) = temp_I;
    [~,temp_I] = min(abs(time_aligned-cut_in_time_vid(i)-10));
    cut_in_s_e(i,2) = temp_I;
end

for i = 1:1:length(index_3)  
    [~,temp_I] = min(abs(time_aligned-lane_change_time_vid(i)));
    index_3(i) = temp_I;
    
    lane_change_s_e(i,1) = temp_I;
    [~,temp_I] = min(abs(time_aligned-lane_change_time_vid(i)-10));
    lane_change_s_e(i,2) = temp_I;
end


figure()
plot(Time-Time(1),valid_relative_dist)
xlabel('Time[s]','FontSize',30)
ylabel('Relative distance[m]','FontSize',30)
set(gca,'FontSize',30)
hold on
h1 = scatter(Time(index_1),valid_relative_dist(index_1),'*');
h2 = scatter(Time(index_cut_in_s),valid_relative_dist(index_cut_in_s),'^');
h3 = scatter(Time(index_3),valid_relative_dist(index_3),'+');
legend([h1 h2 h3],{'cut out','cut in','lane change'})

figure()
plot(Time-Time(1),speed)
xlabel('Time[s]','FontSize',30)
ylabel('Speed[km/hr]','FontSize',30)
set(gca,'FontSize',30)
hold on
h1 = scatter(Time(index_1)-Time(1),speed(index_1),'*');
h2 = scatter(Time(index_cut_in_s)-Time(1),speed(index_cut_in_s),'^');
h3 = scatter(Time(index_3)-Time(1),speed(index_3),'+');
legend([h1 h2 h3],{'cut out','cut in','lane change'})

%% Extract 10-seconds-trajectory after cut-in or  lane change scenarios

figure()
plot(Time-Time(1),valid_relative_dist,'LineWidth',2)
xlabel('Time[s]','FontSize',30)
ylabel('Relative distance[m]','FontSize',30)
set(gca,'FontSize',30)
hold on

h2 = plot(Time(cut_in_s_e(1,1):cut_in_s_e(1,2))-Time(1),valid_relative_dist(cut_in_s_e(1,1):cut_in_s_e(1,2)),'Color',[0.8500 0.3250 0.0980],'LineWidth',3);
h3 = plot(Time(lane_change_s_e(1,1):lane_change_s_e(1,2))-Time(1),valid_relative_dist(lane_change_s_e(1,1):lane_change_s_e(1,2)),'-.','Color',[0.9290 0.6940 0.1250],'LineWidth',3);
for i = 2:1:size(cut_in_s_e,1)
    plot(Time(cut_in_s_e(i,1):cut_in_s_e(i,2))-Time(1),valid_relative_dist(cut_in_s_e(i,1):cut_in_s_e(i,2)),'Color',[0.8500 0.3250 0.0980],'LineWidth',3);
end

for i = 2:1:size(lane_change_s_e,1)
    plot(Time(lane_change_s_e(i,1):lane_change_s_e(i,2))-Time(1),valid_relative_dist(lane_change_s_e(i,1):lane_change_s_e(i,2)),'-.','Color',[0.9290 0.6940 0.1250],'LineWidth',3);
end

legend([h2 h3],{'cut in','lane change'})
xlim([430 1301])

%%
time_headway = valid_relative_dist./valid_speed;

figure()
plot(Time-Time(1),time_headway,'LineWidth',2)
xlabel('Time[s]','FontSize',30)
ylabel('Time headway[s]','FontSize',30)
set(gca,'FontSize',30)
hold on

h2 = plot(Time(cut_in_s_e(1,1):cut_in_s_e(1,2))-Time(1),time_headway(cut_in_s_e(1,1):cut_in_s_e(1,2)),'Color',[0.8500 0.3250 0.0980],'LineWidth',3);
h3 = plot(Time(lane_change_s_e(1,1):lane_change_s_e(1,2))-Time(1),time_headway(lane_change_s_e(1,1):lane_change_s_e(1,2)),'-.','Color',[0.9290 0.6940 0.1250],'LineWidth',3);
for i = 2:1:size(cut_in_s_e,1)
    plot(Time(cut_in_s_e(i,1):cut_in_s_e(i,2))-Time(1),time_headway(cut_in_s_e(i,1):cut_in_s_e(i,2)),'Color',[0.8500 0.3250 0.0980],'LineWidth',3);
end

for i = 2:1:size(lane_change_s_e,1)
    plot(Time(lane_change_s_e(i,1):lane_change_s_e(i,2))-Time(1),time_headway(lane_change_s_e(i,1):lane_change_s_e(i,2)),'-.','Color',[0.9290 0.6940 0.1250],'LineWidth',3);
end

legend([h2 h3],{'cut in','lane change'})
xlim([430 1301])


%%
figure()
plot(Time-Time(1),valid_speed,'LineWidth',2)
xlabel('Time[s]','FontSize',30)
ylabel('Speed [m/s]','FontSize',30)
set(gca,'FontSize',30)
hold on

h2 = plot(Time(cut_in_s_e(1,1):cut_in_s_e(1,2))-Time(1),valid_speed(cut_in_s_e(1,1):cut_in_s_e(1,2)),'Color',[0.8500 0.3250 0.0980],'LineWidth',3);
h3 = plot(Time(lane_change_s_e(1,1):lane_change_s_e(1,2))-Time(1),valid_speed(lane_change_s_e(1,1):lane_change_s_e(1,2)),'-.','Color',[0.9290 0.6940 0.1250],'LineWidth',3);
for i = 2:1:size(cut_in_s_e,1)
    plot(Time(cut_in_s_e(i,1):cut_in_s_e(i,2))-Time(1),valid_speed(cut_in_s_e(i,1):cut_in_s_e(i,2)),'Color',[0.8500 0.3250 0.0980],'LineWidth',3);
end

for i = 2:1:size(lane_change_s_e,1)
    plot(Time(lane_change_s_e(i,1):lane_change_s_e(i,2))-Time(1),valid_speed(lane_change_s_e(i,1):lane_change_s_e(i,2)),'-.','Color',[0.9290 0.6940 0.1250],'LineWidth',3);
end
legend([h2 h3],{'cut in','lane change'})
xlim([430 1301])

%%
figure()
plot(Time-Time(1),valid_relative_spd,'LineWidth',2)
xlabel('Time[s]','FontSize',30)
ylabel('Relative speed [m/s]','FontSize',30)
set(gca,'FontSize',30)
hold on

h2 = plot(Time(cut_in_s_e(1,1):cut_in_s_e(1,2))-Time(1),valid_relative_spd(cut_in_s_e(1,1):cut_in_s_e(1,2)),'Color',[0.8500 0.3250 0.0980],'LineWidth',3);
h3 = plot(Time(lane_change_s_e(1,1):lane_change_s_e(1,2))-Time(1),valid_relative_spd(lane_change_s_e(1,1):lane_change_s_e(1,2)),'-.','Color',[0.9290 0.6940 0.1250],'LineWidth',3);
for i = 2:1:size(cut_in_s_e,1)
    plot(Time(cut_in_s_e(i,1):cut_in_s_e(i,2))-Time(1),valid_relative_spd(cut_in_s_e(i,1):cut_in_s_e(i,2)),'Color',[0.8500 0.3250 0.0980],'LineWidth',3);
end

for i = 2:1:size(lane_change_s_e,1)
    plot(Time(lane_change_s_e(i,1):lane_change_s_e(i,2))-Time(1),valid_relative_spd(lane_change_s_e(i,1):lane_change_s_e(i,2)),'-.','Color',[0.9290 0.6940 0.1250],'LineWidth',3);
end
legend([h2 h3],{'cut in','lane change'})
xlim([430 1301])

%% label cut-in samples  highway [460 sec  ~ 1301 sec] 

[~,highway_s_I] = min(abs(time_aligned-450));
[~,highway_e_I] = min(abs(time_aligned-1301));


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
ylim([0 120])
xlim([-15 15])
ylabel('Relative distance[m]','FontSize',30)
xlabel('Relative speed[m/s]','FontSize',30)
set(gca,'FontSize',30)
hold on

%%
% figure()
% h2 = scatter(valid_relative_spd(cut_in_s_e(1,1):cut_in_s_e(1,2)),valid_relative_dist(cut_in_s_e(1,1):cut_in_s_e(1,2)),60,[0 0 0],'^','filled');
% hold on
% for i = 2:1:size(cut_in_s_e,1)
%     scatter(valid_relative_spd(cut_in_s_e(i,1):cut_in_s_e(i,2)),valid_relative_dist(cut_in_s_e(i,1):cut_in_s_e(i,2)),60,[0 0 0],'^','filled');
% end
% ylabel('Relative distance[m]','FontSize',30)
% xlabel('Relative speed[m/s]','FontSize',30)
% ylim([0 120])
% xlim([-10 10])
% set(gca,'FontSize',30)
% title('cut-in')
% hold on

% figure()
% h2 = scatter(valid_relative_spd(cut_out_s_e(1,1):cut_out_s_e(1,2)),valid_relative_dist(cut_out_s_e(1,1):cut_out_s_e(1,2)),60,[0 0 0],'^','filled');
% hold on
% for i = 2:1:size(cut_out_s_e,1)
%     scatter(valid_relative_spd(cut_out_s_e(i,1):cut_out_s_e(i,2)),valid_relative_dist(cut_out_s_e(i,1):cut_out_s_e(i,2)),60,[0 0 0],'^','filled');
% end
% ylabel('Relative distance[m]','FontSize',30)
% xlabel('Relative speed[m/s]','FontSize',30)
% ylim([0 120])
% xlim([-10 10])
% set(gca,'FontSize',30)
% title('cut-out')
% hold on

% 
% figure()
h2 = scatter(valid_relative_spd(lane_change_s_e(1,1):lane_change_s_e(1,2)),valid_relative_dist(lane_change_s_e(1,1):lane_change_s_e(1,2)),60,[0 0 0],'^','filled');
hold on
for i = 2:1:size(lane_change_s_e,1)
    scatter(valid_relative_spd(lane_change_s_e(i,1):lane_change_s_e(i,2)),valid_relative_dist(lane_change_s_e(i,1):lane_change_s_e(i,2)),60,[0 0 0],'^','filled');
end
ylabel('Relative distance[m]','FontSize',30)
xlabel('Relative speed[m/s]','FontSize',30)
ylim([0 120])
xlim([-10 10])
set(gca,'FontSize',30)
title('lane change')
