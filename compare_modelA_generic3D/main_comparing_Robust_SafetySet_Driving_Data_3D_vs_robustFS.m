%% compare 3D safety set
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
%% load robust safety set for FS

% load('value_function_robustFS_h=0.4_robust_safetySet_h=0.4_1022.mat');
load('value_function_robustFS_modelA_toolbox_grid=61_1027.mat')
FS_robust_SafetySet = safetySet_3dGeneric;

%% convert coordinate from (d_rel,v_lead,v_f) to (de_rel,v_rel,v_f)
[safetyValue_FS] = absolute2relative(FS_robust_SafetySet.x_new_grid,FS_robust_SafetySet.y_new_grid,FS_robust_SafetySet.z_new_grid,-FS_robust_SafetySet.new_val_func);

x_grid_FS = safetyValue_FS.x_new_grid;
y_grid_FS = safetyValue_FS.y_new_grid;
z_grid_FS = safetyValue_FS.z_new_grid;
val_func_FS = safetyValue_FS.new_val_func;


%% plot value function in the new coordinate
[XX,YY,ZZ] = meshgrid(x_grid_FS,y_grid_FS,z_grid_FS);
V = permute(val_func_FS,[2 1 3]);

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
xlabel('Distance[m]','FontSize',20);
ylabel('Relative speed[m/s]','FontSize',20);
zlabel('Follower speed[m/s]','FontSize',20)
set(gca,'FontSize',20)
title('safety set-Gneric 3D ')
grid on

%%
%% HJI 3D Robust Generic safety set
% load('value_function_3D_generic_SafetySet.mat')
% load('value_function_3D_generic_SafetySet_robust_1021.mat')
% load('safetySetComputatoin_generic3D_h=1.2_10_22_20_1503.mat')
% load('safetySetComputatoin_generic3D_h=0.4_10_22_20_1537.mat')
load('safetySetComputatoin_generic3D_h=0.4_10_23_20_1831.mat')


% new_grid.x = x_new_grid;
% new_grid.y = y_new_grid;
% new_grid.z = z_new_grid;
[safetyValue_robust3D] = absolute2relative(robust_safetySet_3dGeneric.x_new_grid,robust_safetySet_3dGeneric.y_new_grid,robust_safetySet_3dGeneric.z_new_grid,-robust_safetySet_3dGeneric.new_val_func);

x_grid_robust3D = safetyValue_robust3D.x_new_grid;
y_grid_robust3D = safetyValue_robust3D.y_new_grid;
z_grid_robust3D = safetyValue_robust3D.z_new_grid;
val_func_robust3D = safetyValue_robust3D.new_val_func;

%% plot value function 3D Generic in the new coordinate
[XX,YY,ZZ] = meshgrid(x_grid_robust3D,y_grid_robust3D,z_grid_robust3D);
V = permute(val_func_robust3D,[2 1 3]);

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
xlabel('Distance[m]','FontSize',20);
ylabel('Relative speed[m/s]','FontSize',20);
zlabel('Follower speed[m/s]','FontSize',20)
title('robust safety set')
set(gca,'FontSize',20)
grid on

%overlay FS safety set
[XX,YY,ZZ] = meshgrid(x_grid_FS,y_grid_FS,z_grid_FS);
V = permute(val_func_FS,[2 1 3]);
Q = patch(isosurface(XX,YY,ZZ,V,0));
isonormals(XX,YY,ZZ,V,P)
Q.FaceColor = 'blue';
Q.EdgeColor = 'none';

%%
%% plot safety boundary at each speed level

figure_3DsafetySet_On2D_handle = figure()
[XX_generic3D,YY_generic3D] = meshgrid(x_grid_robust3D,y_grid_robust3D);
safetySlice_generic3D = val_func_robust3D(:,:,1);
[~,contourPlot_handle_3d] = contour(YY_generic3D,XX_generic3D,safetySlice_generic3D',[0 0],'LineWidth',2);
contourPlot_handle_3d.Color = [0.4940 0.1840 0.5560];
hold on

[XX_3dGeneric,YY_3dGeneric] = meshgrid(x_grid_FS,y_grid_FS);
safetySlice_FS = val_func_FS(:,:,1);
[~,contourPlot_handle_3dGeneric] = contour(YY_3dGeneric,XX_3dGeneric,safetySlice_FS',[0 0],'LineWidth',2);
contourPlot_handle_3dGeneric.Color = [0.9290, 0.6940, 0.1250];

ylabel('Distance[m]','FontSize',20);
xlabel('Relative speed[m/s]','FontSize',20);
set(gca,'FontSize',20)
grid on
xlim([-15 15])
ylim([-5 100])
v_r_boundary_handle = plot([0,0],[0,120],'LineWidth',2,'Color','black');

%overlay driving data points
plot_handle_driving_data = scatter(valid_relative_spd,valid_relative_dist,'.','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0,'MarkerFaceColor',[0.4660 0.6740 0.1880],'MarkerEdgeColor',[0.4660 0.6740 0.1880]);

plot_handle_driving_data.MarkerFaceColor = [0 0.4470 0.7410];
plot_handle_driving_data.MarkerEdgeColor = [0 0.4470 0.7410];
set(gcf, 'Color', 'w');
figure_3DsafetySet_On2D_handle.Position =  [201 36 760*2 675*2];
v_follower_g = z_grid_FS;
legend('3D','robustFS')

fname = 'robust_safetySet_3D_robustFS_vs_fieldTestData_h=0.4.gif';
for frame_index = 1:1:length(v_follower_g)
    %extract field test data around this speed
    speed_lower_bound = v_follower_g(frame_index)-1;
    speed_upper_bound = v_follower_g(frame_index)+1;
    accel_pt_temp = valid_accel(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    relative_dist_pt_temp = valid_relative_dist(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    relative_spd_pt_temp = valid_relative_spd(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    speed_pt_temp = valid_speed(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    
    relative_dist_pt_temp = relative_dist_pt_temp(relative_spd_pt_temp+v_follower_g(frame_index)>=0);
    relative_spd_pt_temp = relative_spd_pt_temp(relative_spd_pt_temp+v_follower_g(frame_index)>=0);
    
    plot_handle_driving_data.XData = relative_spd_pt_temp;
    plot_handle_driving_data.YData = relative_dist_pt_temp;    
    
    safetySlice_generic3D = val_func_robust3D(:,:,frame_index);
    contourPlot_handle_3d.ZData = safetySlice_generic3D';
    
    safetySlice_FS = val_func_FS(:,:,frame_index);
    contourPlot_handle_3dGeneric.ZData = safetySlice_FS';
    v_r_boundary_handle.XData=[-v_follower_g(frame_index),-v_follower_g(frame_index)];
    v_r_boundary_handle.YData=[0,120];
    title(['speed = ', num2str(v_follower_g(frame_index),'%.1f'), '[m/s]'])
    drawnow
%     pause

    frame =  getframe(figure_3DsafetySet_On2D_handle) ;
    MakeGIF(fname,frame,frame_index)
    
end



%% generate figure for the paper 

[XX_FS,YY_FS] = meshgrid(x_grid_FS,y_grid_FS);
paperfig_handle = figure()
subplot_m = 2;
subplot_n = 2;

speed_grid = v_follower_g;

speed_for_subplot = [7,15,23,30];
frame_ = zeros(size(speed_for_subplot));
for i = 1:1:length(speed_for_subplot)
    [~,temp_index] = min(abs(speed_grid-speed_for_subplot(i)));
    frame_(i) =  temp_index;
end

ValFunc_FS = val_func_FS;

for i = 1:1:subplot_m*subplot_n
    subplot(subplot_m,subplot_n,i)
    frame_index = frame_(i);
    
    [~,M1] = contour(YY_FS,XX_FS,ValFunc_FS(:,:,frame_index)',[0 0],'LineWidth',2);
    M1.Color = [0.9290, 0.6940, 0.1250];
    hold on
%     safetySlice_3dGeneric = val_func_3dGeneric(:,:,frame_index);
%     [~,contourPlot_handle_3dGeneric] = contour(YY_3dGeneric,XX_3dGeneric,safetySlice_3dGeneric',[0 0],'LineWidth',2);
%     contourPlot_handle_3dGeneric.Color = [0.4940 0.1840 0.5560];
    
%     legend('3D','FS')
   
    ylim([0 80])
%     xlim([max([-12,-speed_for_subplot(i)]) min([12,speed_for_subplot(i)])])
    xlim([-12 12])
    ylabel('Distance[m]','FontSize',30)
    xlabel('Relative speed[m/s]','FontSize',30)
    set(gca,'FontSize',30)    
    grid on
    
    v_r_boundary_handle = plot([-speed_grid(frame_index),-speed_grid(frame_index)],[0,120],'LineWidth',2,'Color','black');
    title(['Speed = ', num2str(speed_grid(frame_index),'%.1f'), '[m/s]'])
    ax = gca;
    ax.TitleFontSizeMultiplier = 1.0;
    
    %% plot data points
    temp_speed_ = speed_for_subplot(i);
    speed_lower_bound = temp_speed_;
    speed_upper_bound = temp_speed_+1;
    accel_pt_temp = valid_accel(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    relative_dist_pt_temp = valid_relative_dist(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    relative_spd_pt_temp = valid_relative_spd(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    speed_pt_temp = valid_speed(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    
    relative_dist_pt_temp = relative_dist_pt_temp(relative_spd_pt_temp+speed_lower_bound>=0);
    relative_spd_pt_temp = relative_spd_pt_temp(relative_spd_pt_temp+speed_lower_bound>=0);
    
    scatter(relative_spd_pt_temp,relative_dist_pt_temp,'.','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0,'MarkerFaceColor',[0 0.4470 0.7410],'MarkerEdgeColor',[0 0.4470 0.7410]);


end


paperfig_handle.Position =  [201 36 760*2 675*2];
set(gcf, 'Color', 'w');