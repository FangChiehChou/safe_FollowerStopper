
%%
clear all
clc

%% extract field test data : some trajectories

load_fieldTestData

raw_data_time = time2-time2(1);
raw_data_d_rel = relative_distance;
raw_data_v_rel = relative_speed;
raw_data_v_f = speed;
test_data_a_f = accel;
raw_data_v_lead = raw_data_v_rel + raw_data_v_f;

figure()
plot(raw_data_time,raw_data_v_lead)
hold on
plot(raw_data_time,raw_data_v_f)


figure()
plot(raw_data_time,raw_data_d_rel)

%%  High speed data
% temp_t_init = 3329;
% temp_t_end = 3358;
% 
% [~,temp_index_init] = min(abs(raw_data_time-temp_t_init));
% [~,temp_index_end] = min(abs(raw_data_time-temp_t_end));
% 
% test_data_d_rel = raw_data_d_rel(temp_index_init:temp_index_end);
% test_data_v_lead = raw_data_v_lead(temp_index_init:temp_index_end);
% test_data_v_f = raw_data_v_f(temp_index_init:temp_index_end);
% test_data_time = raw_data_time(temp_index_init:temp_index_end);
% test_data_v_rel = raw_data_v_rel(temp_index_init:temp_index_end);
% 
% 
% global data_time data_lead_spd
% data_time = test_data_time-test_data_time(1);
% data_lead_spd = test_data_v_lead;

%% Low speed data
temp_t_init = 18410;
temp_t_end = 18510;

[~,temp_index_init] = min(abs(raw_data_time-temp_t_init));
[~,temp_index_end] = min(abs(raw_data_time-temp_t_end));

test_data_d_rel = raw_data_d_rel(temp_index_init:temp_index_end);
test_data_v_lead = raw_data_v_lead(temp_index_init:temp_index_end);
test_data_v_f = raw_data_v_f(temp_index_init:temp_index_end);
test_data_time = raw_data_time(temp_index_init:temp_index_end);
test_data_v_rel = raw_data_v_rel(temp_index_init:temp_index_end);


global data_time data_lead_spd
data_time = test_data_time-test_data_time(1);
data_lead_spd = test_data_v_lead;


%%
load('.\crash_low_speed.mat')
%% plot these data

Gazebo_simtime_init = 201.26;

figure()
plot(cat_cmd_spd.time-Gazebo_simtime_init,cat_cmd_spd.data,'LineWidth',2);
hold on
plot(cat_vel.time-Gazebo_simtime_init,cat_vel.data,'LineWidth',2)
plot(cat_ref_vel.time-Gazebo_simtime_init,cat_ref_vel.data,'LineWidth',2);
title('AV vehicle speed')
legend('spd_{cmd}','spd','spd_{ref}')
xlabel('Time[s]')
set(gca,'FontSize',30)

figure()
plot(toyota_cmd_vel.time-Gazebo_simtime_init,toyota_cmd_vel.data,'LineWidth',2);
hold on
plot(toyota_vel.time-Gazebo_simtime_init,toyota_vel.data,'LineWidth',2);
plot(v_leader_est.time-Gazebo_simtime_init,v_leader_est.data,'LineWidth',2);
title('Toyota vehicle speed')
legend('spd_{cmd}','spd','spd_{est}')
xlabel('Time[s]')
set(gca,'FontSize',30)

figure()
plot(cat_distEsimtation.time-Gazebo_simtime_init,cat_distEsimtation.data,'LineWidth',2)
xlabel('Time[s]')
ylabel('Relative distance')
set(gca,'FontSize',30)

figure()
plot(cat_v_rel_estimation.time-Gazebo_simtime_init,cat_v_rel_estimation.data,'LineWidth',2)
xlabel('Time[s]')
ylabel('Relative speed[m/s]')
set(gca,'FontSize',30)


figure()
plot(cat_distEsimtation.time-Gazebo_simtime_init,cat_distEsimtation.data,'LineWidth',2)
hold on
plot(cat_v_rel_estimation.time-Gazebo_simtime_init,cat_v_rel_estimation.data,'LineWidth',2)
xlabel('Time[s]')
% ylabel('Relative distance')
set(gca,'FontSize',30)
legend('Relative distance','Relative speed[m/s]')



figure()
plot(data_time,data_lead_spd,'LineWidth',2)
hold on
plot(toyota_cmd_vel.time-Gazebo_simtime_init-22,toyota_cmd_vel.data,'LineWidth',2);

%% figures for paper

t_s_offset = 201.26;
t_s_offset = t_s_offset+22;

x_lim_end = 70;
figure_1=figure()
plot(test_data_time-test_data_time(1),test_data_d_rel,'LineWidth',2)
hold on
plot(cat_distEsimtation.time-t_s_offset,cat_distEsimtation.data,'-.','LineWidth',2)
figure_1.Position = [201 136 760 675];
% plot(t_no_delay,d_rel_no_delay,'-.','LineWidth',2)
xlabel('Time[s]','FontSize',40)
ylabel('Relative distance[m]','FontSize',40)
set(gca,'FontSize',40)
legend('human driver','FollowerStopper')
g = gca;
g.Legend.FontSize =30;
xlim([0 x_lim_end])
grid on


figure_2=figure()
plot(data_time,data_lead_spd,'LineWidth',2)
hold on
plot(data_time,test_data_v_f,'-.','LineWidth',2)
plot((cat_vel.time-t_s_offset),cat_vel.data,'-.','LineWidth',2)
plot((cat_cmd_spd.time-t_s_offset),cat_cmd_spd.data,'LineWidth',2)
figure_2.Position = [201 136 760 675];
xlabel('Time[s]','FontSize',40)
ylabel('Speed[m/s]','FontSize',40)
set(gca,'FontSize',40)
legend('leader','human driver','FollowerStopper','FollowerStopper_{cmd}')
g = gca;
g.Legend.FontSize =30;
xlim([0 x_lim_end])
grid on


figure_3 = figure()
plot((cat_cmd_spd.time-t_s_offset),cat_cmd_spd.data,'LineWidth',2,'Color',[0, 0.4470, 0.7410]);
hold on
plot((cat_vel.time-t_s_offset),cat_vel.data,'-.','LineWidth',2)
% plot((toyota_vel.time-t_s_offset),toyota_vel.data,'LineWidth',2)
% plot(data_time,data_lead_spd,'LineWidth',2)

plot(toyota_cmd_vel.time-t_s_offset,toyota_cmd_vel.data,'LineWidth',2);
plot(data_time,test_data_v_f,'LineWidth',2)
legend('AV command spd','AV speed','lead vehicle speed','human driver')
xlabel('Time[s]')
ylabel('Speed[m/s]')
set(gca,'FontSize',30)
grid on
% figure_3.Position = [201 136 760 675];
set(gcf, 'Units', 'centimeters', 'OuterPosition', [1 7 35 21]);
set(gcf, 'Color', 'w');
xlim([0 60])
% eval(['export_fig ','Sim_newFS_Gazebo_HS',' -pdf']);


figure_4 = figure()
plot((cat_distEsimtation.time-t_s_offset),cat_distEsimtation.data,'-.','LineWidth',2,'Color',[0.8500 0.3250 0.0980])
hold on
% plot((Time-Time(1))-560.5-t_s_offset-5,car.lead_dist,'LineWidth',2,'Color',[0.4940 0.1840 0.5560])
plot(data_time,test_data_d_rel,'LineWidth',2,'Color',[0.4940 0.1840 0.5560])
grid on
xlabel('Time[s]')
ylabel('Relative distance[m]')
set(gca,'FontSize',30)
set(gcf, 'Units', 'centimeters', 'OuterPosition', [1 7 35 21]);
set(gcf, 'Color', 'w');
legend('AV','human driver')
xlim([0 60])
% eval(['export_fig ','Sim_newFS_Gazebo-dist_HS',' -pdf']);





%%
sim_init_time = 6.36;

figure_5 = figure()
plot((cat_cmd_spd.time-t_s_offset-sim_init_time),cat_cmd_spd.data,'LineWidth',2,'Color',[0, 0.4470, 0.7410]);
hold on
plot((cat_vel.time-t_s_offset-sim_init_time),cat_vel.data,'-.','LineWidth',2)
% plot((toyota_vel.time-t_s_offset),toyota_vel.data,'LineWidth',2)
% plot(data_time,data_lead_spd,'LineWidth',2)

plot(toyota_cmd_vel.time-t_s_offset-sim_init_time,toyota_cmd_vel.data,'LineWidth',2);
% plot(data_time,test_data_v_f,'LineWidth',2)
% legend('AV command spd','AV speed','lead vehicle speed','human driver')
legend('AV command spd','AV speed','lead vehicle speed')
xlabel('Time[s]')
ylabel('Speed[m/s]')
set(gca,'FontSize',30)
grid on
% figure_3.Position = [201 136 760 675];
set(gcf, 'Units', 'centimeters', 'OuterPosition', [1 7 35 21]);
set(gcf, 'Color', 'w');
xlim([0 2.0])
eval(['export_fig ','Sim_FS_Gazebo_LS_crash_speed',' -eps']);


figure_6 = figure()
plot((cat_distEsimtation.time-t_s_offset-sim_init_time),cat_distEsimtation.data,'-.','LineWidth',2,'Color',[0.8500 0.3250 0.0980])
hold on
% plot((Time-Time(1))-560.5-t_s_offset-5,car.lead_dist,'LineWidth',2,'Color',[0.4940 0.1840 0.5560])
% plot(data_time,test_data_d_rel,'LineWidth',2,'Color',[0.4940 0.1840 0.5560])
grid on
xlabel('Time[s]')
ylabel('Relative distance[m]')
set(gca,'FontSize',30)
set(gcf, 'Units', 'centimeters', 'OuterPosition', [1 7 35 21]);
set(gcf, 'Color', 'w');
% legend('AV','human driver')
legend('AV')
xlim([0 2.0])
eval(['export_fig ','Sim_FS_Gazebo_LS_crash_distance',' -eps']);
