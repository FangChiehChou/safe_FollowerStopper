function [new_data] = absolute2relative(x_origin,y_origin,z_origin,v_origin,new_grid)

if(nargin>4)
    x_new_grid = new_grid.x;
    y_new_grid = new_grid.y;
    z_new_grid = new_grid.z;
else
    x_new_grid = x_origin;
%     y_new_grid = -15:1:15;
    y_new_grid = z_origin-15;
    z_new_grid = z_origin;    
end

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
                i_o = find(x_origin == temp_d_rel);
                j_origin = find(y_origin == temp_v_lead);
                k_o = find(z_origin == temp_v_f);
                
                if(temp_d_rel <min(x_origin))
                    i_o = 1;
                end
                if(temp_d_rel > max(x_origin))
                    i_o = length(i_o);
                end
                
                if(temp_v_lead <min(y_origin))
                    j_origin = 1;
                end
                if(temp_v_lead > max(y_origin))
                    j_origin = length(y_origin);
                end
                
                if(temp_v_f <min(z_origin))
                    k_o = 1;
                end
                if(temp_v_f > max(z_origin))
                    k_o = length(z_origin);
                end
                
                if(isempty(i_o)||isempty(j_origin)||isempty(k_o))
                    %% find the closest 8 grid points on the original coordinates.
                    if(~isempty(i_o))
                        x_grid_index = [i_o;i_o];
                    else
                        temp_I1 = find(temp_d_rel>=x_origin);
                        temp_I2 = find(temp_d_rel<=x_origin);
                        x_grid_index = [temp_I1(end);temp_I2(1)];
                    end
                    
                    
                    if(~isempty(j_origin))
                        y_grid_index = [j_origin;j_origin];
                    else
                        temp_I1 = find(temp_v_lead>=y_origin);
                        temp_I2 = find(temp_v_lead<=y_origin);
                        y_grid_index = [temp_I1(end);temp_I2(1)];
                    end
                    
                    if(~isempty(k_o))
                        z_grid_index = [k_o;k_o];
                    else
                        temp_I1 = find(temp_v_f>=z_origin);
                        temp_I2 = find(temp_v_f<=z_origin);
                        z_grid_index = [temp_I1(end);temp_I2(1)];
                    end
                    
                    
                    %% extract value of each point and distance to each point
                    value_around = zeros(8,1);
                    distance_to_points = zeros(8,1);
                    count = 0;
                    
                    for x_grid_index_i = x_grid_index'
                        for y_grid_index_j = y_grid_index'
                            for z_grid_index_k = z_grid_index'
                               count = count + 1;
                               temp_point = [x_origin(x_grid_index_i);y_origin(y_grid_index_j);z_origin(z_grid_index_k)];
                               value_around(count) = v_origin(x_grid_index_i,y_grid_index_j,z_grid_index_k);
                               distance_to_points(count) = sqrt(sum((temp_point-[temp_d_rel;temp_v_lead;temp_v_f]).^2)); 
                            end
                        end
                    end
                    
                    total_distance = sum(distance_to_points);
                    
                    %% take weighted average of the values of the closiest 8 points
                    new_val = total_distance*sum(value_around./distance_to_points);
                    
                else
                    new_val = v_origin(i_o,j_origin,k_o);
                end
            end
          
            new_val_func(i,j,k) = new_val;
        end
    end    
end

new_data.x_new_grid = x_new_grid;
new_data.y_new_grid = y_new_grid;
new_data.z_new_grid = z_new_grid;
new_data.new_val_func = new_val_func;

end