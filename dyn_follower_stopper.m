function [dx_vehicle,v_des] = dyn_follower_stopper(t,x,d_rel,v_rel,uMin,uMax,delay_size,exp_num,r)
% x(1) : CAT vehicle speed
% x(2) : PID1 state - yi
% x(3) : PID1 state - YD
% x(4) : PID2 state - yi
% x(5) : PID2 state - YD

    v = x(1);
    % desired speed from follower stopper contorller
    
    if(nargin < 9 )
        v_des = follower_stopper(d_rel,v_rel,v);
    else
        v_des = follower_stopper(d_rel,v_rel,v,r);
    end
    
    %%vehicle longitudinal dynamics
    dx_vehicle = CAT_veh_with_delay(t,x,v_des,exp_num,delay_size);
    a = dx_vehicle(1);

    a(a>=uMax) = uMax;
    a(a<=uMin) = uMin;
    dx_vehicle(1) = a;
    
end


function dx = CAT_veh_with_delay(t,x,u,exp_num,delay_size)
% u : command speed
% x(1) : CAT vehicle speed
% x(2) : PID1 state - yi
% x(3) : PID1 state - YD
% x(4) : PID2 state - yi
% x(5) : PID2 state - YD

e = u - x(1);
PID_x = x(2:5);

n = 0; % additive noise to speed feedback
PID_in = e + n;

[PID_dx,PID_out] = control(PID_x,PID_in);

if(delay_size>0)
    vehicle_in = delay(PID_out,t,delay_size,exp_num);
else
    vehicle_in = PID_out;
end


dx = zeros(size(x));
dx(1) = 0.133*vehicle_in-0.5*x(1);
dx(2:5) = PID_dx; 

end



function output = delay(y_new,t_new,delay_size,exp_num)
    global hist y_hist t_hist
%     y_hist = hist(exp_num).y_hist;
%     t_hist = hist(exp_num).t_hist;
        
    if(t_new<=delay_size)
        output = y_new;
    else
        [~,temp_index] = min(abs(t_new-t_hist-delay_size));  %find the index of the delayed signal in y_hist
        output = y_hist(temp_index);
        
        if(t_new - t_hist(1) >= delay_size*2)
            t_hist = [t_hist(temp_index:end)];  %remove too old data and append new data at the end
            y_hist = [y_hist(temp_index:end)];
        end        
    end    
    y_hist = [y_hist;y_new];  %append new data at the end of the historic array
    t_hist = [t_hist;t_new];


%     hist(exp_num).y_hist = y_hist;
%     hist(exp_num).t_hist = t_hist;

end


% function dx = CAT_veh(t,x,u)
% % u : command speed
% % x(1) : CAT vehicle speed
% % x(2) : PID1 state - yi
% % x(3) : PID1 state - YD
% % x(4) : PID2 state - yi
% % x(5) : PID2 state - YD
% 
% e = u - x(1);
% PID_x = x(2:5);
% 
% n = 0; % additive noise to speed feedback
% PID_in = e + n;
% 
% [PID_dx,PID_out] = control(PID_x,PID_in);
% 
% vehicle_in = PID_out;
% 
% dx = zeros(size(x));
% dx(1) = 0.133*vehicle_in-0.5*x(1);
% dx(2:5) = PID_dx; 
% 
% end



function [dx,u] = control(x,e)
    sat_upper = 40;  %clampping anti-windup upper bound
    sat_lower = -60; %clampping anti-windup lower bound
    
    params.sat_upper = sat_upper;
    params.sat_lower = sat_lower;
    
    PID1_state = x(1:2);
    PID2_state = x(3:4);
    
%     [PID1_dx,PID1_y] = PID_SpeedUp(PID1_state,e,[],params);
%     TR = PID1_y;
%     [PID2_dx,~] = PID_SlowDown(PID2_state,e,TR,params);
%     PID2_dx = [0,0];
%     u = PID1_y;
    
    
    if(e>-0.25)
        %use controller 1
        [PID1_dx,PID1_y] = PID_SpeedUp(PID1_state,e,[],params);
        TR = PID1_y;
        [PID2_dx,~] = PID_SlowDown(PID2_state,e,TR,params);
        u = PID1_y;
    else
        %use controller 2
        [PID2_dx,PID2_y] = PID_SlowDown(PID2_state,e,[],params);
        TR = PID2_y;
        [PID1_dx,~] = PID_SpeedUp(PID1_state,e,TR,params);
        u = PID2_y;
    end
    
    dx = zeros(size(x));
    dx(1) = PID1_dx(1);
    dx(2) = PID1_dx(2);
    dx(3) = PID2_dx(1);
    dx(4) = PID2_dx(2);
    
end


function [dx,y] = PID_SpeedUp(x,u,TR,params)
%% This is PID controller with bumpless control and anti-windup
% x(1) = yi
% x(2) = YD
% u : input to the PID controller
% y : output from the PID controller
% TR: tracking signal

P = 7.00379813340796;
I = 6.07606353611095;
D = 0.656379900606905;
N = 2.27284583867859;
Kt = 1; %signal tracking gain 

yi = x(1); %integration output
YD = x(2);
yd = u*D*N - N*YD;

yp = P*u;
y = yp+yi+yd;  %PID block output


if(isempty(TR))
    TR = y;
end

dx = zeros(size(x));
dx(1) = I*u+Kt*(TR-y);  %integration input
dx(2) = yd;

% anti-windup clampping method
% freeze the integration if the output y is exceeding saturation bounds.
if(y > params.sat_upper && sign(y) == sign(u) )
    dx(1) = 0;
end

if(y < params.sat_lower && sign(y) == sign(u) )
    dx(1) = 0;
end

end


function [dx,y] = PID_SlowDown(x,u,TR,params)
% x(1) = yi
% x(2) = YD
% u : input to the PID controller
% y : output from the PID controller
% TR: tracking signal

P = 31.5137498864335;
I = 38.8451000083051;
D = -2.26736704240256;
N = 4.84135397787565;
Kt = 1; %signal tracking gain 

yi = x(1); %integration output
YD = x(2);
yd = u*D*N - N*YD;

yp = P*u;
y = yp+yi+yd;  %PID block output

if(isempty(TR))
    TR = y;
end

dx = zeros(size(x));
dx(1) = I*u+Kt*(TR-y);  %integration input
dx(2) = yd;


% anti-windup clampping method
% freeze the integration if the output y is exceeding saturation bounds.
if(y > params.sat_upper && sign(y) == sign(u) )
    dx(1) = 0;
end

if(y < params.sat_lower && sign(y) == sign(u) )
    dx(1) = 0;
end

end
