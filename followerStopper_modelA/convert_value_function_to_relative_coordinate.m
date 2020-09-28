%% proejct value function from absolute coordinate to relative coordinate
clear all
clc

%% load precommuted value results
load('sim_CFM_main_multi_v.mat')
%%
%x_new = x;
%y_new = y-z;
%z_new = z;

%% plot the value function in the original coordinate 
value_func = min_gap_matrix;
x_grid = d_rel_g; %0:2:50
y_grid = v_lead_g; %0:1:30;
z_grid = v_follower_g; %0:1:30;

[XX_old,YY_old,ZZ_old] = meshgrid(d_rel_g,v_lead_g,v_follower_g);
V_old = permute(min_gap_matrix,[2 1 3]);

figure_original_handle = figure()
P = patch(isosurface(XX_old,YY_old,ZZ_old,V_old,0));
isonormals(XX_old,YY_old,ZZ_old,V_old,P)
P.FaceColor = 'red';
P.EdgeColor = 'none';
daspect([1 1 1])
view(3); 
axis tight
camlight 
lighting gouraud
xlabel('distance[m]');
ylabel('leader speed[m/s]');
zlabel('follower speed[m/s]')
grid on


%%
x_new_grid = x_grid;
y_new_grid = -15:1:15;
z_new_grid = z_grid;
new_val_func = zeros(length(x_new_grid),length(y_new_grid),length(z_new_grid));


for i = 1:1:length(x_new_grid)
    temp_d_rel = x_new_grid(i);
    for j = 1:1:length(y_new_grid)
        temp_v_rel = y_new_grid(j);
        for k = 1:1:length(z_new_grid)
            temp_v_f = z_new_grid(k);
            temp_v_lead = temp_v_rel + temp_v_f;
            if(temp_v_lead < 0)
                new_val = -999;
            else

                j_origin = find( y_grid == temp_v_lead);
                if(temp_v_lead <min(y_grid) )
                    j_origin = 1;
                end
                if(temp_v_lead > max(y_grid))
                    j_origin = length(y_grid);
                end
                
                new_val = value_func(i,j_origin,k);               
            end
            
            new_val_func(i,j,k) = new_val;
        end
    end    
end


%% plot value function in new coordinate
[XX,YY,ZZ] = meshgrid(x_new_grid,y_new_grid,z_new_grid);
V = permute(new_val_func,[2 1 3]);

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
ylabel('relative speed[m/s]');
zlabel('follower speed[m/s]')
grid on


%% plot safety boundary at each speed level

figure_3DsafetySet_On2D_handle = figure()
[XX,YY] = meshgrid(x_new_grid,y_new_grid);
safetySlice = new_val_func(:,:,1);
[~,contourPlot_handle] = contour(YY,XX,safetySlice',[0 0])
xlabel('distance[m]');
ylabel('relative speed[m/s]');

grid on
hold on


for frame_index = 1:1:length(v_follower_g)
    safetySlice = new_val_func(:,:,frame_index);
  
%     contour(YY,XX,safetySlice')
    contourPlot_handle.ZData = safetySlice';
    title(['speed = ', num2str(v_follower_g(frame_index),'%.1f'), '[m/s]'])
    drawnow
    pause
    
end
    
    
    
    
    
    
