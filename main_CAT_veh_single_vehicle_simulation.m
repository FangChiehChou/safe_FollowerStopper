clear all
clc

%%
d_rel_0 = 100; 
v_rel_0 = -10;
v_f_0 = 20;
v_lead_0 = v_rel_0+v_f_0;

%% 
global hist y_hist t_hist
y_hist = [];
t_hist = [];
temp_hist=struct();
hist = temp_hist;
params.delay_size = 0.05;

uMin = -5.3;
uMax = 3.5;

params.external_r = 30;
tspan = [0 30];

x0 = [d_rel_0;v_lead_0;v_f_0;0;0;0;0]; 
% opts = odeset('RelTol',1e-10,'AbsTol',1e-10);
% [t,y] = ode45(@(t,x) CFM_CATVEH_model(t,x,uMin,uMax,params,1),tspan,x0,opts);    
[t,y] = ode45(@(t,x) CFM_CATVEH_model(t,x,uMin,uMax,params,1),tspan,x0);    

d_rel = y(:,1);
v_lead = y(:,2);
v_f = y(:,3);
v_rel = v_lead-v_f;

%compute follower stopper command speed
v_cmd = zeros(size(d_rel));

for i =1:1:length(v_cmd)
    v_des = follower_stopper(d_rel(i),v_rel(i),v_f(i),params.external_r);
%     [dx_vehicle,v_des] = dyn_follower_stopper(t(i),y(:,i),d_rel(i),v_rel(i),uMin,uMax,exp_num,params.external_r);
    v_cmd(i) = v_des;
end


%%
figure()
plot(t,d_rel,'LineWidth',2)
xlabel('Time[s]','FontSize',30)
ylabel('relative position','FontSize',30)
set(gca,'FontSize',30)

figure()
plot(t,v_lead,'LineWidth',2)
hold on
plot(t,v_f,'LineWidth',2)
plot(t,v_cmd,'LineWidth',2)
ylabel('relative speed[m/s]')
legend('v_{lead}','v_{f}','v_{cmd}')
set(gca,'FontSize',30)
xlabel('Time[s]','FontSize',30)
ylabel('Speed[m/s]','FontSize',30)



%% 

fig_handle = followerStopper_boundary()
plot(v_rel,d_rel)


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
    plot(v_rel_grid,boundary1_x)
    hold on
    plot(v_rel_grid,boundary2_x)
    plot(v_rel_grid,boundary3_x)
    set(gca,'FontSize',30)
    xlabel('rel spd [m/s]','FontSize',30)
    ylabel('rel dist[m]','FontSize',30)
    
end