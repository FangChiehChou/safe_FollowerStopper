function [new_data] = absolute2relative(x_origin,y_origin,z_origin,v_origin)


x_new_grid = x_origin;
y_new_grid = -15:1:15;
z_new_grid = z_origin;
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

                j_origin = find( y_origin == temp_v_lead);
                if(temp_v_lead <min(y_origin) )
                    j_origin = 1;
                end
                if(temp_v_lead > max(y_origin))
                    j_origin = length(y_origin);
                end
                
                new_val = v_origin(i,j_origin,k);               
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