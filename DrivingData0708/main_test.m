

speed = 30; 

v_rel_grid = -10:1:10;

[envelope1,envelope2,envelope3] = o_FS_evnvelopes_on_2D(speed,v_rel_grid);


%% assume initial relative diatance is a constant. Compute the relative distances changing by time considering a few constant relative speeds.

d_rel_0 = 80;
v_rel = [-2;-4;-6;-8];

t_s = 0:0.1:30;

d_rel = d_rel_0 + v_rel*t_s;

figure()
plot(t_s,d_rel(1,:))
hold on
plot(t_s,d_rel(2,:))
plot(t_s,d_rel(3,:))
plot(t_s,d_rel(4,:))
set(gca,'FontSize',30)
xlabel('Time[s]','FontSize',30)
ylabel('relative distance[m]','FontSize',30)


v_cmd_FS =   %get speed command of the FS given the state



%% help function --- original FS envelopes
function [envelope1,envelope2,envelope3] = o_FS_evnvelopes_on_2D(speed,v_rel_grid)
    delta0_x1 = 4.5;
    delta0_x2 = 5.25;
    delta0_x3 = 6.0;
    
    d_1 = 1.5;
    d_2 = 1.0;
    d_3 = 0.5;
    
    h_1 = 0.4;
    h_2 = 0.6;
    h_3 = 0.8;

    envelope1 = zeros(size(v_rel_grid,1),size(v_rel_grid,2));
    envelope2 = zeros(size(v_rel_grid,1),size(v_rel_grid,2));
    envelope3 = zeros(size(v_rel_grid,1),size(v_rel_grid,2));
 
    for j = 1:1:length(v_rel_grid)
        v_f = speed;

        dv_ss = v_rel_grid(j);
        dv_ss(dv_ss>=0) = 0;          

        del_x_bound1 =  (dv_ss.^2)/(2*d_1) + delta0_x1;
        del_x_bound2 =  (dv_ss.^2)/(2*d_2) + delta0_x2;
        del_x_bound3 =  (dv_ss.^2)/(2*d_3) + delta0_x3;
        envelope1(j) =  del_x_bound1;
        envelope2(j) =  del_x_bound2;
        envelope3(j) =  del_x_bound3;
    end

end




%% help function --- modified FS envelopes
function [envelope1,envelope2,envelope3] = m_FS_evnvelopes_on_2D(speed,v_rel_grid)
    delta0_x1 = 4.5;
    delta0_x2 = 5.25;
    delta0_x3 = 6.0;
    
    d_1 = 1.5;
    d_2 = 1.0;
    d_3 = 0.5;
    
    h_1 = 0.4;
    h_2 = 0.6;
    h_3 = 0.8;

    envelope1 = zeros(size(v_rel_grid,1),size(v_rel_grid,2));
    envelope2 = zeros(size(v_rel_grid,1),size(v_rel_grid,2));
    envelope3 = zeros(size(v_rel_grid,1),size(v_rel_grid,2));
 
    for j = 1:1:length(v_rel_grid)
        v_f = speed;

        dv_ss = v_rel_grid(j);
        dv_ss(dv_ss>=0) = 0;          

        del_x_bound1 =  (dv_ss.^2)/(2*d_1) + max(delta0_x1,h_1*v_f);
        del_x_bound2 =  (dv_ss.^2)/(2*d_2) + max(delta0_x2,h_2*v_f);
        del_x_bound3 =  (dv_ss.^2)/(2*d_3) + max(delta0_x3,h_3*v_f);
        envelope1(j) =  del_x_bound1;
        envelope2(j) =  del_x_bound2;
        envelope3(j) =  del_x_bound3;
    end

end