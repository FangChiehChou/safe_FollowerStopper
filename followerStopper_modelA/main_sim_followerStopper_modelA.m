%% write a simulator to validate reachability safety set by points
% close all 
% clear all
% clc


%% given initial state 

%% no collision scenario
% d_rel_0 = 80;
% v_rel_0 = -5;
% v_f_0 = 30;

%%1 second crash scenario - this is what HJ reachability told us
% d_rel_0 = 10;
% v_rel_0 = -10;
% v_f_0 = 20;


%%
d_rel_0 = 20;
v_rel_0 = -10;
v_f_0 = 30;


%% 
uMin = -3.5;
uMax = 3.5;

%% run simulation for a given initial state
tspan = [0 30];
x0 = [d_rel_0;v_rel_0;v_f_0];

params.external_r = 5;

opts = odeset('RelTol',1e-10,'AbsTol',1e-10);
[t,y] = ode45(@(t,x) CFM_model(t,x,uMin,uMax),tspan,x0,opts); 
% [t,y] = ode45(@(t,x) CFM_TwoCars_model(t,x,uMin,uMax,params),tspan,x0,opts);  


d_rel = y(:,1);
v_rel = y(:,2);
v_f = y(:,3);

v_lead = v_f + v_rel;

a = dyn_follower_stopper(d_rel,v_rel,v_f,uMin,uMax);

v_des = follower_stopper(d_rel,v_rel,v_f);

% %% check time at  t = 2.623
% 
% t_check = 2.623;
% [~,temp_I] = min(abs(t-t_check));
% 
% v_des(temp_I)
% temp_d_rel = d_rel(temp_I)
% tmep_v_rel = v_rel(temp_I)
% temp_v_f = v_f(temp_I)
% 
% follower_stopper(temp_d_rel,tmep_v_rel,temp_v_f)

%%
figure()
plot(t,d_rel)
ylabel('d_{rel}')

figure()
plot(t,v_rel)
ylabel('v_{rel}')

figure()
plot(t,v_lead)
hold on
plot(t,v_f);
plot(t,v_des)
ylabel('v')
legend('v_{lead}','v_f','v_{des}')

figure()
plot(t,a)
ylabel('u')


