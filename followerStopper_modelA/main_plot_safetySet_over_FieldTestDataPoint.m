%% this script is for overlaying field test data and FollowerStopper safe set

clear all
clc


%% load field test data

load('../../circleData/2020-07-28-17-56-48-879520_2020_03_05_relative_distance_relative_speed_Data.mat');
load('../../circleData/2020-07-28-17-40-56-648843_2020_03_05_Acceleration_Speed_Data.mat');


%% load follower stopper safety set
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
%% resample speed and accel
speed = interp1(time1,speed,time2,'nearest');
accel = interp1(time1,accel,time2,'nearest');    

%estimate speed of the lead vehicle.
speed_lead  = speed + relative_speed;
valid_relative_dist = relative_distance(speed_lead>0 & relative_distance<200);
valid_relative_spd = relative_speed(speed_lead>0 & relative_distance<200);
valid_speed = speed(speed_lead>0& relative_distance<200);
valid_accel = accel(speed_lead>0 & relative_distance<200);


%% load value safety value function.
load('value_function_modelA.mat');

%% plot the 3D safety set
[XX,YY,ZZ] = meshgrid(d_rel_g,v_lead_g,v_follower_g);
V = permute(min_gap_matrix,[2 1 3]);

figure_3dsafetySet_handle = figure()
P = patch(isosurface(XX,YY,ZZ,V,0));
isonormals(XX,YY,ZZ,V,P)
P.FaceColor = 'red';
P.EdgeColor = 'none';
daspect([1 1 1])
view(3); 
axis tight
camlight 
lighting gouraud
xlabel('distance[m]');
ylabel('lead speed[m/s]');
zlabel('follower speed[m/s]')
grid on

%% convert coordinate from (d_rel,v_lead,v_f) to (de_rel,v_rel,v_f)
x_grid = d_rel_g; %0:2:50
y_grid = v_lead_g; %0:1:30;
z_grid = v_follower_g; %0:1:30;

x_origin = x_grid;
y_origin = y_grid;
z_origin = z_grid;
v_origin = min_gap_matrix;
[new_data] = absolute2relative(x_origin,y_origin,z_origin,v_origin);

x_new_grid = new_data.x_new_grid;
y_new_grid = new_data.y_new_grid;
z_new_grid = new_data.z_new_grid;
new_val_func = new_data.new_val_func;

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
grid on


%% plot safety boundary at each speed level
%% overlay the safety value function on the same plot
%% make gif 

figure_3DsafetySet_On2D_handle = figure()
[XX,YY] = meshgrid(x_new_grid,y_new_grid);
safetySlice = new_val_func(:,:,1);
[~,contourPlot_handle] = contour(YY,XX,safetySlice',[0 0],'LineWidth',2);
xlabel('distance[m]','FontSize',20);
ylabel('relative speed[m/s]','FontSize',20);
set(gca,'FontSize',20)
grid on
hold on
xlim([-15 15])
v_r_boundary_handle = plot([0,0],[0,120],'LineWidth',2,'Color','black');
plot_handle = scatter(valid_relative_spd,valid_relative_dist,'.','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);

fname = 'FS_safetySet_vs_fieldTestData.gif';
for frame_index = 1:1:length(v_follower_g)
    %extract field test data around this speed
    speed_lower_bound = v_follower_g(frame_index)-1;
    speed_upper_bound = v_follower_g(frame_index)+1;
    accel_pt_temp = valid_accel(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    relative_dist_pt_temp = valid_relative_dist(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    relative_spd_pt_temp = valid_relative_spd(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    speed_pt_temp = valid_speed(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    plot_handle.XData = relative_spd_pt_temp;
    plot_handle.YData = relative_dist_pt_temp;    
    
    
    safetySlice = new_val_func(:,:,frame_index);
    contourPlot_handle.ZData = safetySlice';
    v_r_boundary_handle.XData=[-v_follower_g(frame_index),-v_follower_g(frame_index)];
    v_r_boundary_handle.YData=[0,120];
    title(['speed = ', num2str(v_follower_g(frame_index),'%.1f'), '[m/s]'])
    drawnow
%     pause

    frame =  getframe(figure_3DsafetySet_On2D_handle) ;
    MakeGIF(fname,frame,frame_index)
end



%% generate figure for paper 
paperfig_handle = figure(3)
subplot_m = 3;
subplot_n = 1;

speed_grid = v_follower_g;
speed_to_show_paper = [7,15,23];
frame_ = [find(speed_grid==7);find(speed_grid==15);find(speed_grid==23)];

[XX_FS,YY_FS] = meshgrid(x_new_grid,y_new_grid);
ValFunc_FS = new_val_func;

for i = 1:1:subplot_m
    speed_lower_bound = speed_to_show_paper(i)-1;
    speed_upper_bound = speed_to_show_paper(i)+1;
    relative_dist_pt_temp = valid_relative_dist(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    relative_spd_pt_temp = valid_relative_spd(speed_lower_bound<=valid_speed&valid_speed<speed_upper_bound);
    
    subplot(subplot_m,subplot_n,i)
%     M = contour(YY_2D,XX_2D,ValFunc_2D,[0 0]);
%     plot(M(1,2:end),M(2,2:end),'LineWidth',2,'Color',[0    0.4470    0.7410])
    frame_index = frame_(i);
    [~,M1] = contour(YY_FS,XX_FS,ValFunc_FS(:,:,frame_index)',[0 0],'LineWidth',2);
    M1.Color = [0.9290, 0.6940, 0.1250];
    hold on
    scatter(relative_spd_pt_temp,relative_dist_pt_temp,'.');
    ylim([0 60])
    xlim([max([-15,-speed_grid(frame_index)]) -max([-15,-speed_grid(frame_index)])])
    ylabel('distance[m]','FontSize',10)
    xlabel('rel_{spd}[m/s]','FontSize',10)
    set(gca,'FontSize',10)
    
    grid on
    
    
    
    v_r_boundary_handle = plot([-speed_grid(frame_index),-speed_grid(frame_index)],[0,120],'LineWidth',2,'Color','black');
    title(['speed = ', num2str(speed_grid(frame_index),'%.1f'), '[m/s]'])
    ax = gca;
    ax.TitleFontSizeMultiplier = 1.0;
end

