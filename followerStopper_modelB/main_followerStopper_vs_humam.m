clear all
clc

load_fieldTestData

%% extract field test data : some trajectories

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

%%  TODO == extract a period that v_f is greater than 20 m/s and following a leader for at least 10 seconds

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
%% simulate the follower stopper with the field test data
d_rel_0 = test_data_d_rel(1);
v_lead_0 = test_data_v_lead(1);
v_f_0 = test_data_v_f(1);

uMin = -5.3;
uMax = 3.5;
params.external_r = 30;
tspan = [0 30];
params.delay_size = 0.0;
params.d_Min = -3.5;

x0 = [d_rel_0;v_f_0;0;0;0;0]; 
% opts = odeset('RelTol',1e-10,'AbsTol',1e-10);
% [t,y] = ode45(@(t,x) CFM_CATVEH_model(t,x,uMin,uMax,params,1),tspan,x0,opts);    
[t,y] = ode45(@(t,x) CFM_CATVEH_model_with_lead_speed(t,x,uMin,uMax,params,1),tspan,x0);    

d_rel = y(:,1);
v_f = y(:,2);
v_lead = interp1(data_time,data_lead_spd,t);
v_rel = v_lead-v_f;

%compute follower stopper command speed
v_cmd = zeros(size(d_rel));

for i =1:1:length(v_cmd)
    v_des = follower_stopper(d_rel(i),v_rel(i),v_f(i),params.external_r);
%     [dx_vehicle,v_des] = dyn_follower_stopper(t(i),y(:,i),d_rel(i),v_rel(i),uMin,uMax,exp_num,params.external_r);
    v_cmd(i) = v_des;
end


%% plot follower stopper versus human driving 
figure()
plot(test_data_time-test_data_time(1),test_data_d_rel,'LineWidth',2)
hold on
plot(t,d_rel,'-.','LineWidth',2)
% plot(t_no_delay,d_rel_no_delay,'-.','LineWidth',2)
xlabel('Time[s]','FontSize',30)
ylabel('Relative position[m]','FontSize',30)
set(gca,'FontSize',30)
legend('human driver','follower Stopper')
xlim([0 t(end)])

figure()
plot(data_time,data_lead_spd,'LineWidth',2)
hold on
plot(t,v_f,'-.','LineWidth',2)
plot(t,v_cmd,'LineWidth',2)
xlabel('Time[s]','FontSize',30)
ylabel('Speed[m/s]','FontSize',30)
set(gca,'FontSize',30)
legend('human driver','followerStopper','followerStopper_{cmd}')
xlim([0 t(end)])

%%

fig_handle = followerStopper_boundary()
h1 = plot(test_data_v_rel,test_data_d_rel,'LineWidth',2)
hold on
h2 = plot(v_rel,d_rel,'-.','LineWidth',2)
legend([h1,h2],'human driver','follower Stopper')
ylabel('Relative position[m]','FontSize',30)
xlabel('Relative Speed[m/s]','FontSize',30)

%% follower stopper boundary plot
function fig_handle = followerStopper_boundary()

    w = zeros(3,1);
    alpha = zeros(3,1);
    w(1) = 4.5;
    alpha(1) = 1.5;
    w(3) = 6.0;
    alpha(3) = 0.5;
    w(2) = (w(1)+w(3))/2;
    alpha(2) = (alpha(1)+alpha(3))/2;

    v_rel_grid = -15:1:15;
        
    v_rel_start_grid = v_rel_grid;
    v_rel_start_grid(v_rel_start_grid >= 0) = 0;
   
    boundary1_x =  w(1) + (0.5*v_rel_start_grid.^2)/alpha(1);
    boundary2_x =  w(2) + (0.5*v_rel_start_grid.^2)/alpha(2);
    boundary3_x =  w(3) + (0.5*v_rel_start_grid.^2)/alpha(3);
    
    fig_handle = figure()
    plot(v_rel_grid,boundary1_x,'LineWidth',2)
    hold on
    plot(v_rel_grid,boundary2_x,'LineWidth',2)
    plot(v_rel_grid,boundary3_x,'LineWidth',2)
    set(gca,'FontSize',30)
    xlabel('rel spd [m/s]','FontSize',30)
    ylabel('rel dist[m]','FontSize',30)
    
end
