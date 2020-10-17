%% This script loads in the safety value computed and plot it on 3D and 2D space.
clear all
clc

%% HJI 2D safety set
% [XX_2D,YY_2D,ValFunc_2D] = HJI_2D_safety_value_function_generator();
load('value_function_2D_SafetySet.mat')

%% 

load('value_function_modelB_w_delay.mat')


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
grid on


%% plot safety boundary at each speed level

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

for frame_index = 1:1:length(v_follower_g)
    safetySlice = new_val_func(:,:,frame_index);
    contourPlot_handle.ZData = safetySlice';
    v_r_boundary_handle.XData=[-v_follower_g(frame_index),-v_follower_g(frame_index)];
    v_r_boundary_handle.YData=[0,120];
    title(['speed = ', num2str(v_follower_g(frame_index),'%.1f'), '[m/s]'])
    drawnow
    pause
    
end


%% generate figure for paper 
paperfig_handle = figure(3)
subplot_m = 4;
subplot_n = 1;

speed_grid = v_follower_g;
frame_ = [find(speed_grid==7);find(speed_grid==15);find(speed_grid==23);find(speed_grid==30)];

[XX_FS,YY_FS] = meshgrid(followerStopper_safetyValue.x_new_grid,followerStopper_safetyValue.y_new_grid);
ValFunc_FS = followerStopper_safetyValue.new_val_func;

for i = 1:1:subplot_m
    subplot(subplot_m,subplot_n,i)
    M = contour(YY_2D,XX_2D,ValFunc_2D,[0 0]);
    plot(M(1,2:end),M(2,2:end),'LineWidth',2,'Color',[0    0.4470    0.7410])
    hold on
    ylim([0 60])
    xlim([-15 15])
    ylabel('distance[m]','FontSize',10)
    xlabel('rel_{spd}[m/s]','FontSize',10)
    set(gca,'FontSize',10)
    
    grid on
    
    frame_index = frame_(i);
    
    [~,M1] = contour(YY_FS,XX_FS,ValFunc_FS(:,:,frame_index)',[0 0],'LineWidth',2);
    M1.Color = [0.9290, 0.6940, 0.1250];
    v_r_boundary_handle = plot([-speed_grid(frame_index),-speed_grid(frame_index)],[0,120],'LineWidth',2,'Color','black');
    title(['speed = ', num2str(speed_grid(frame_index),'%.1f'), '[m/s]'])
    ax = gca;
    ax.TitleFontSizeMultiplier = 1.0;
end



