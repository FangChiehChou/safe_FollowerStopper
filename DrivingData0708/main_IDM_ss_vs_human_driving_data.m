%% write a script that shows a compares the steady state of the IDM and human driving data

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


%% IDM steady state
IDMparams.delta = 4; 
IDMparams.a = 1.3;
IDMparams.b = 2.0;
IDMparams.S0 = 2;
IDMparams.T = 1;
IDMparams.v0 = 30;

delta = IDMparams.delta; 
a = IDMparams.a;
b = IDMparams.b;
S0 = IDMparams.S0;
T = IDMparams.T;
v0 = IDMparams.v0;

IDM_ss_speed = 0:1:30;
IDM_ss_gap = zeros(size(IDM_ss_speed,1),size(IDM_ss_speed,2));

for index = 1:1:length(IDM_ss_speed)
    
    IDM_ss_gap(index) = IDM_equilibrium_spacing(IDM_ss_speed(index),delta,S0,T,v0);

end


%% update FS steady state

delta0_x1 = 4.5;
delta0_x2 = 5.25;
delta0_x3 = 6.0;

d_1 = 1.5;
d_2 = 1.0;
d_3 = 0.5;

h_1 = 0.4;
h_2 = 0.6;
h_3 = 0.8;

FS_ss_speed = 0:1:35;

FS_gap_env1 = zeros(size(FS_ss_speed,1),size(FS_ss_speed,2));
FS_gap_env2 = zeros(size(FS_ss_speed,1),size(FS_ss_speed,2));
FS_gap_env3 = zeros(size(FS_ss_speed,1),size(FS_ss_speed,2));

for index = 1:1:length(FS_ss_speed)
    
    FS_gap_env1(index) = max(delta0_x1,h_1*FS_ss_speed(index));
    FS_gap_env2(index) = max(delta0_x2,h_2*FS_ss_speed(index));
    FS_gap_env3(index) = max(delta0_x3,h_3*FS_ss_speed(index));

end


%%

figure()
scatter(valid_relative_dist,valid_speed)
hold on
plot(IDM_ss_gap,IDM_ss_speed,'LineWidth',2)

plot(FS_gap_env1,FS_ss_speed,'LineWidth',2)
plot(FS_gap_env2,FS_ss_speed,'LineWidth',2)
plot(FS_gap_env3,FS_ss_speed,'LineWidth',2)

xlabel('gap [m]','FontSize',30)
ylabel('speed [m/s]','FontSize',30)
set(gca,'FontSize',30)
xlim([0 80])
grid on
set(gcf, 'Color', 'w');


%% plot the figure in density (x axis = 1/km) vs flow (1/h)

IDM_density = 1000./(IDM_ss_gap+4.5);  
IDM_flow = IDM_density.*(IDM_ss_speed*3.6);

FS_env1_density = 1000./(FS_gap_env1+4.5);  
FS_env1_flow = FS_env1_density.*(FS_ss_speed*3.6);

FS_env2_density = 1000./(FS_gap_env2+4.5);  
FS_env2_flow = FS_env2_density.*(FS_ss_speed*3.6);

FS_env3_density = 1000./(FS_gap_env3+4.5);  
FS_env3_flow = FS_env3_density.*(FS_ss_speed*3.6);

driving_data_density = 1000./(valid_relative_dist(valid_relative_dist<125)+4.5);
driving_data_flow = driving_data_density.*(valid_speed(valid_relative_dist<125)*3.6);

figure()
scatter(driving_data_density,driving_data_flow)
hold on
plot(IDM_density,IDM_flow,'LineWidth',2)
plot(FS_env1_density,FS_env1_flow,'LineWidth',2)
plot(FS_env2_density,FS_env2_flow,'LineWidth',2)
plot(FS_env3_density,FS_env3_flow,'LineWidth',2)
xlabel('density [1/km]','FontSize',30)
ylabel('flow [1/hr]','FontSize',30)
set(gca,'FontSize',30)
grid on




