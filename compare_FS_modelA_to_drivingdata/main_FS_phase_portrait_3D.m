%% plot phase portrait of the FollowerStopper on 2D plane (d_rel, v_rel)

clear all
clc

%% 

d_rel = 0:10:50;
v_rel = -10:2:10;
v_AV = 0:2:30;

[XX,YY,ZZ] = meshgrid(d_rel,v_rel,v_AV);   

vec_x = zeros(size(XX,1),size(XX,2),size(XX,3));  %\dot{v_{rel}}  : relative acceleration
vec_y = zeros(size(XX,1),size(XX,2),size(XX,3));  %\dot{d_{rel}} : relative speed
vec_z = zeros(size(XX,1),size(XX,2),size(XX,3));  %\dot{a_{AV}} : AV acceleration

vehicle_speed = 15;
accel_lead = 0;

r = 30;
uMax = 1.5;   %+1.5 m/s^2  = AV acceleration bound
uMin = -3.0;  %-3.0 m/s^2  = AV acceleration bound
%  accelx_FS(i) = dyn_follower_stopper(highway_lead_dist(i),highway_rel_speed(i),speed_highway(i),uMin,uMax,r);
%  accelx_mFS(i) = dyn_modified_follower_stopper(highway_lead_dist(i),highway_rel_speed(i),speed_highway(i),uMin,uMax,r);
for i = 1:1:size(XX,1)   % -> v_rel 
    for j = 1:1:size(XX,2)  % -> d_rel
        for k = 1:1:size(XX,3) % -> v_AV
            d_rel_temp = XX(i,j,k);
            v_rel_temp = YY(i,j,k);
            v_AV_temp = ZZ(i,j,k);
            a_av = dyn_modified_follower_stopper(d_rel_temp,v_rel_temp,v_AV_temp,uMin,uMax,r);
            vec_x(i,j,k) = v_rel_temp;
            vec_y(i,j,k) = accel_lead - a_av;  
            vec_z(i,j,k) = a_av;
        end
    end   
end


%% plot phase portrait and envelopes of the FollowerStopper

figure()
quiver3(XX,YY,ZZ,vec_x,vec_y,vec_z)
hold on
% [h1,h2,h3] = overlay_FS_curves(vehicle_speed);

xlabel('Relative distnace[m]')
ylabel('Relative speed[m/s]')
zlabel('Speed[m/s]')
set(gca,'FontSize',30)
set(gcf, 'Color', 'w');

%%

% accel_lead = -1;
% for i = 1:1:size(XX,1)
%     for j = 1:1:size(XX,2)
%         d_rel_temp = XX(i,j);
%         v_rel_temp = YY(i,j);
%         a_av = dyn_follower_stopper(d_rel_temp,v_rel_temp,vehicle_speed,uMin,uMax,r);
%         vec_x(i,j) = v_rel_temp;
%         vec_y(i,j) = accel_lead - a_av;
%     end   
% end


%% simulate a few trajectories of the FollowerStopper and put them in the figure

v_lead = 10;
d_rel_0 = 30;
v_f_0 = 10;

d_rel_set = [30,40,40,20,15,20,15,20,8];
v_f_set = [10,15,10,17,17,8,6,18,16];


v_lead_set = [5,10,15,20];

%% plot phase portrait and envelopes of the FollowerStopper

figure()
% quiver(YY',XX',vec_y',vec_x')
% hold on
% [h1,h2,h3] = overlay_originalFS_curves();

cmap1=colormap(hot(15));

for i =1:1:length(d_rel_set)
    for j = 1:1:length(v_lead_set)
        
        
        v_lead = v_lead_set(j);
        d_rel_0 = d_rel_set(i);
        v_f_0 = v_f_set(i);
        x0 = [d_rel_0;v_f_0]; 
        tspan = [0 30];
%         [t,y] = ode45(@(t,x) FS_simulation_model(t,x,uMin,uMax,r,v_lead),tspan,x0);    
%         plot3(y(:,1),v_lead-y(:,2),y(:,2),'Color',[0.8500 0.3250 0.0980],'LineWidth',2);
%         hold on

        [t,y] = ode45(@(t,x) mFS_simulation_model(t,x,uMin,uMax,r,v_lead),tspan,x0);    

%         plot3(y(:,1),v_lead-y(:,2),y(:,2),'Color',[0.4660 0.6740 0.1880],'LineWidth',2);
        


        
        plot3(y(:,1),v_lead-y(:,2),y(:,2),'Color',cmap1(j,:),'LineWidth',2);

        hold on
        
               
    end

end
grid on
xlabel('Relative distnace[m]')
ylabel('Relative speed[m/s]')


zlabel('Speed[m/s]')

set(gca,'FontSize',30)
set(gcf, 'Color', 'w');

% xlim([-9 6])
% ylim([0 45])



%% purturb speed of the leading vehicle





%% help functions --- overlay FS curves
function [h1,h2,h3] = overlay_FS_curves(v_f)

dv_s = -10:0.1:10;
dv_ss = dv_s;
dv_ss(dv_ss>=0) = 0;

delta0_x1 = 4.5;
delta0_x2 = 5.25;
delta0_x3 = 6.0;
d_1 = 1.5;
d_2 = 1.0;
d_3 = 0.5;
h_1 = 0.4;
h_2 = 1.2;
h_3 = 1.8;

del_x_bound1 =  delta0_x1 + (dv_ss.^2)/(2*d_1) + h_1*v_f;
del_x_bound2 =  delta0_x2 + (dv_ss.^2)/(2*d_2) + h_2*v_f;
del_x_bound3 =  delta0_x3 + (dv_ss.^2)/(2*d_3) + h_3*v_f;

h1 = plot(dv_s,del_x_bound1,'LineWidth',2,'Color',[0.3010 0.7450 0.9330]);
h2 = plot(dv_s,del_x_bound2,'LineWidth',2,'Color',[0.3010 0.7450 0.9330]);
h3 = plot(dv_s,del_x_bound3,'LineWidth',2,'Color',[0.3010 0.7450 0.9330]);
end


%% help functions --- overlay FS curves
function [h1,h2,h3] = overlay_originalFS_curves()

dv_s = -10:0.1:10;
dv_ss = dv_s;
dv_ss(dv_ss>=0) = 0;

delta0_x1 = 4.5;
delta0_x2 = 5.25;
delta0_x3 = 6.0;
d_1 = 1.5;
d_2 = 1.0;
d_3 = 0.5;
h_1 = 0.4;
h_2 = 1.2;
h_3 = 1.8;

del_x_bound1 =  delta0_x1 + (dv_ss.^2)/(2*d_1) ;
del_x_bound2 =  delta0_x2 + (dv_ss.^2)/(2*d_2) ;
del_x_bound3 =  delta0_x3 + (dv_ss.^2)/(2*d_3) ;

h1 = plot(dv_s,del_x_bound1,'LineWidth',2,'Color',[0.3010 0.7450 0.9330]);
h2 = plot(dv_s,del_x_bound2,'LineWidth',2,'Color',[0.3010 0.7450 0.9330]);
h3 = plot(dv_s,del_x_bound3,'LineWidth',2,'Color',[0.3010 0.7450 0.9330]);
end
