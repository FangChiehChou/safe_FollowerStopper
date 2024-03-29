%% compare value from 2D safety set and 3D safety set
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
valid_relative_dist = relative_distance(speed_lead>0 & relative_distance<200);
valid_relative_spd = relative_speed(speed_lead>0 & relative_distance<200);
valid_speed = speed(speed_lead>0& relative_distance<200);
valid_accel = accel(speed_lead>0 & relative_distance<200);

%%
%% HJI 2D safety set
% [XX_2D,YY_2D,ValFunc_2D] = HJI_2D_safety_value_function_generator();
load('value_function_2D_SafetySet.mat')

%%
%% load follower stopper safety set 
load('value_function_modelA.mat')

%% convert coordinate from (d_rel,v_lead,v_f) to (de_rel,v_rel,v_f)
x_grid = d_rel_g; %0:2:50
y_grid = v_lead_g; %0:1:30;
z_grid = v_follower_g; %0:1:30;

x_origin = x_grid;
y_origin = y_grid;
z_origin = z_grid;
v_origin = min_gap_matrix;
[followerStopper_safetyValue] = absolute2relative(x_origin,y_origin,z_origin,v_origin);

x_new_grid = followerStopper_safetyValue.x_new_grid;
y_new_grid = followerStopper_safetyValue.y_new_grid;
z_new_grid = followerStopper_safetyValue.z_new_grid;
new_val_func = followerStopper_safetyValue.new_val_func;

%% plot value function in the new coordinate
[XX,YY,ZZ] = meshgrid(x_new_grid,y_new_grid,z_new_grid);
V = permute(new_val_func,[2 1 3]);

figure_3dsafetySet_handle2 = figure()
P = patch(isosurface(XX,YY,ZZ,V,0));
isonormals(XX,YY,ZZ,V,P)
P.FaceColor = 'red';
P.EdgeColor = 'none';
daspect([1 1 1])
view(3); 
axis tight
camlight 
lighting gouraud
xlabel('distance[m]','FontSize',20);
ylabel('relative speed[m/s]','FontSize',20);
zlabel('follower speed[m/s]','FontSize',20)
set(gca,'FontSize',20)
title('safety set-FS/ModelA')
grid on

%%
%% HJI 3D Generic safety set
% load('value_function_3D_generic_SafetySet.mat')
% load('value_function_3D_generic_SafetySet_robust_1021.mat')
load('value_function_FS_tool_box.mat')

new_grid.x = x_new_grid;
new_grid.y = y_new_grid;
new_grid.z = z_new_grid;
[safetyValue_3dGeneric] = absolute2relative(safetySet_3dGeneric.x_new_grid,safetySet_3dGeneric.y_new_grid,safetySet_3dGeneric.z_new_grid,-safetySet_3dGeneric.new_val_func,new_grid);

x_grid_3dGeneric = safetyValue_3dGeneric.x_new_grid;
y_grid_3dGeneric = safetyValue_3dGeneric.y_new_grid;
z_grid_3dGeneric = safetyValue_3dGeneric.z_new_grid;
val_func_3dGeneric = safetyValue_3dGeneric.new_val_func;

%% plot value function 3D Generic in the new coordinate
[XX,YY,ZZ] = meshgrid(x_grid_3dGeneric,y_grid_3dGeneric,z_grid_3dGeneric);
V = permute(val_func_3dGeneric,[2 1 3]);

figure_3dGenericsafetySet_handle = figure()
P = patch(isosurface(XX,YY,ZZ,V,0));
isonormals(XX,YY,ZZ,V,P)
P.FaceColor = 'red';
P.EdgeColor = 'none';
daspect([1 1 1])
view(3); 
axis tight
camlight 
lighting gouraud
xlabel('distance[m]','FontSize',20);
ylabel('relative speed[m/s]','FontSize',20);
zlabel('follower speed[m/s]','FontSize',20)
title('safety set - 3D generic model')
set(gca,'FontSize',20)
grid on

[XX,YY,ZZ] = meshgrid(x_new_grid,y_new_grid,z_new_grid);
V = permute(new_val_func,[2 1 3]);
Q = patch(isosurface(XX,YY,ZZ,V,0));
isonormals(XX,YY,ZZ,V,P)
Q.FaceColor = 'blue';
Q.EdgeColor = 'none';


%%
%% plot safety boundary at each speed level
[XX_3dGeneric,YY_3dGeneric] = meshgrid(x_grid_3dGeneric,y_grid_3dGeneric);
safetySlice_3dGeneric = val_func_3dGeneric(:,:,1);

figure_3DsafetySet_On2D_handle = figure()
[XX,YY] = meshgrid(x_new_grid,y_new_grid);
safetySlice = new_val_func(:,:,1);
[~,contourPlot_handle_3d] = contour(YY,XX,safetySlice',[0 0],'LineWidth',2);
contourPlot_handle_3d.Color = [0.9290, 0.6940, 0.1250];
hold on
[~,contourPlot_handle_3dGeneric] = contour(YY_3dGeneric,XX_3dGeneric,safetySlice_3dGeneric',[0 0],'LineWidth',2);
contourPlot_handle_3dGeneric.Color = [0.4940 0.1840 0.5560];

ylabel('distance[m]','FontSize',20);
xlabel('relative speed[m/s]','FontSize',20);
set(gca,'FontSize',20)
grid on
xlim([-15 15])
ylim([-5 120])
v_r_boundary_handle = plot([0,0],[0,120],'LineWidth',2,'Color','black');

%overlay 2D safety set
M_2D = contour(YY_2D,XX_2D,ValFunc_2D,[0 0]);
plot(M_2D(1,2:end),M_2D(2,2:end),'LineWidth',2,'Color',[0    0.4470    0.7410])

%overlay driving data points
plot_handle_driving_data = scatter(valid_relative_spd,valid_relative_dist,'.','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0,'MarkerFaceColor',[0.4660 0.6740 0.1880],'MarkerEdgeColor',[0.4660 0.6740 0.1880]);

fname = 'FS_safetySet_vs_2D_vs_fieldTestData.gif';
for frame_index = 1:1:length(v_follower_g)
    %extract field test data around this speed
    speed_lower_bound = v_follower_g(frame_index)-1;
    speed_upper_bound = v_follower_g(frame_index)+1;
    accel_pt_temp = valid_accel(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    relative_dist_pt_temp = valid_relative_dist(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    relative_spd_pt_temp = valid_relative_spd(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    speed_pt_temp = valid_speed(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    plot_handle_driving_data.XData = relative_spd_pt_temp;
    plot_handle_driving_data.YData = relative_dist_pt_temp;    
    
    safetySlice = new_val_func(:,:,frame_index);
    contourPlot_handle_3d.ZData = safetySlice';
    safetySlice_3dGeneric = val_func_3dGeneric(:,:,frame_index);
    contourPlot_handle_3dGeneric.ZData = safetySlice_3dGeneric';
    v_r_boundary_handle.XData=[-v_follower_g(frame_index),-v_follower_g(frame_index)];
    v_r_boundary_handle.YData=[0,120];
    title(['speed = ', num2str(v_follower_g(frame_index),'%.1f'), '[m/s]'])
    drawnow
%     pause

    frame =  getframe(figure_3DsafetySet_On2D_handle) ;
    MakeGIF(fname,frame,frame_index)
    
end

%%
% [XX,YY] = meshgrid(x_new_grid,y_new_grid);
% safetySlice = new_val_func(:,:,31);
% [~,contourPlot_handle_3d] = contour(YY,XX,safetySlice',[0 0],'LineWidth',2);
% 
% figure()
% surf(YY_2D,XX_2D,-ValFunc_2D)
% ylabel('distance[m]','FontSize',20);
% xlabel('relative speed[m/s]','FontSize',20);
% hold on
% surf(YY,XX,safetySlice')
% 
% figure()
% %overlay 2D safety set
% M_2D = contour(YY_2D,XX_2D,ValFunc_2D,[0 0]);
% plot(M_2D(1,2:end),M_2D(2,2:end),'LineWidth',2,'Color',[0    0.4470    0.7410])
% xlabel('distance[m]','FontSize',20);
% ylabel('relative speed[m/s]','FontSize',20);

