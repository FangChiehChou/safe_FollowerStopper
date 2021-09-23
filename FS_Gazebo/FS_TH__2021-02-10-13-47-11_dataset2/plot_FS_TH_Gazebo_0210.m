clear all
clc

load('2020-07-26-16-35-44_2T3Y1RFV8KC014025_CAN_Messages.mat');
car.time = Time;
car.lead_dist = lead_distance;
car.speed = speed;
car.relative_vel = relative_vel;

% load('.\FS_TH__2021-01-20-11-29-22\FS_Gazebo.mat')
load('.\FS_Gazebo_0210.mat')
%% plot these data
figure()
plot(cat_cmd_spd.time,cat_cmd_spd.data,'LineWidth',2);
hold on
plot(cat_vel.time,cat_vel.data,'LineWidth',2)
plot(cat_ref_vel.time,cat_ref_vel.data,'LineWidth',2);
title('CAT vehicle speed')
legend('spd_{cmd}','spd','spd_{ref}')
xlabel('Time[s]')
set(gca,'FontSize',30)

figure()
plot(toyota_cmd_vel.time-toyota_cmd_vel.time(1),toyota_cmd_vel.data,'LineWidth',2);
hold on
plot(toyota_vel.time-toyota_vel.time(1),toyota_vel.data,'LineWidth',2);
plot(v_leader_est.time-v_leader_est.time(1),v_leader_est.data,'LineWidth',2);
title('Toyota vehicle speed')
legend('spd_{cmd}','spd','spd_{est}')
xlabel('Time[s]')
set(gca,'FontSize',30)

figure()
plot(cat_distEsimtation.time,cat_distEsimtation.data,'LineWidth',2)
xlabel('Time[s]')
ylabel('Relative distance')
set(gca,'FontSize',30)

figure()
plot(cat_v_rel_estimation.time,cat_v_rel_estimation.data,'LineWidth',2)
xlabel('Time[s]')
ylabel('Relative speed[m/s]')
set(gca,'FontSize',30)

%% figures for paper

t_s_offset = 10;
x_lim_end = 70;
figure_1=figure()
plot((Time-Time(1))-560.5-t_s_offset,car.lead_dist,'LineWidth',2)
hold on
plot((cat_distEsimtation.time-cat_distEsimtation.time(1))/1-16,cat_distEsimtation.data,'-.','LineWidth',2)
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
plot((toyota_vel.time-toyota_vel.time(1))/1-16,toyota_vel.data,'LineWidth',2,'Color',[0.9290, 0.6940, 0.1250])
hold on
plot((Time-Time(1))-560.5-t_s_offset,(car.speed)/3.6,'LineWidth',2,'Color',[0, 0.4470, 0.7410])
plot((cat_vel.time-cat_vel.time(1))/-16,cat_vel.data,'-.','LineWidth',2,'Color',[0.8500, 0.3250, 0.0980])
plot((cat_cmd_spd.time-cat_cmd_spd.time(1))/1-16,cat_cmd_spd.data,'LineWidth',2)
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
plot((cat_cmd_spd.time-cat_cmd_spd.time(1))/1-16-5,cat_cmd_spd.data,'LineWidth',2,'Color',[0, 0.4470, 0.7410]);
hold on
plot((cat_vel.time-cat_vel.time(1))/1-16-5,cat_vel.data,'-.','LineWidth',2)
plot((toyota_vel.time-toyota_vel.time(1))/1-16-5,toyota_vel.data,'LineWidth',2);
plot((Time-Time(1))-560.5-t_s_offset-5,(car.speed)/3.6,'LineWidth',2,'Color',[0.4940 0.1840 0.5560])
legend('CAT command spd','CAT speed','lead vehicle speed','human driver')
xlabel('Time[s]')
ylabel('Speed[m/s]')
set(gca,'FontSize',30)
grid on
% figure_3.Position = [201 136 760 675];
set(gcf, 'Units', 'centimeters', 'OuterPosition', [1 7 35 21]);
set(gcf, 'Color', 'w');
xlim([0 65])
eval(['export_fig ','Sim_newFS_Gazebo',' -pdf']);


figure_4 = figure()
plot((cat_distEsimtation.time-cat_distEsimtation.time(1))/1-16-5,cat_distEsimtation.data,'-.','LineWidth',2,'Color',[0.8500 0.3250 0.0980])
hold on
plot((Time-Time(1))-560.5-t_s_offset-5,car.lead_dist,'LineWidth',2,'Color',[0.4940 0.1840 0.5560])
grid on
xlabel('Time[s]')
ylabel('Relative distance[m]')
set(gca,'FontSize',30)
set(gcf, 'Units', 'centimeters', 'OuterPosition', [1 7 35 21]);
set(gcf, 'Color', 'w');
legend('CAT','human driver')
xlim([0 65])
eval(['export_fig ','Sim_newFS_Gazebo-dist',' -pdf']);



% 
% figure_5 = figure()
% plot((toyota_cmd_vel.time-toyota_cmd_vel.time(1))/1-16,toyota_cmd_vel.data*3.6,'LineWidth',2);
% hold on
% plot((Time-Time(1))-560.5-t_s_offset,car.speed+car.relative_vel)