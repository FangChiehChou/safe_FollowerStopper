%% 
function dx = CFM_CATVEH_model_with_lead_speed(t,x,uMin,uMax,params,exp_num)

%% system states
%d_rel = x(1) : relative distance between lead vehicle and the following vehicle
%v_follower = x(2) : speed of the follower vehicle
% auxiliary states
% x(3) : PID1 state - yi
% x(4) : PID1 state - YD


%% system inputs
% u : acceleration of the following vehicle
% data_time/data_lead_spd : lead vehicle speed time series

%% parameters 
% uMin/uMax : acceleraiton lower and upper bound of the follower vehicle
% non_negative_speed : whether imposing state constraint or not

global data_time data_lead_spd
v_lead = interp1(data_time,data_lead_spd,t);

non_negative_speed = 1;

dx = zeros(size(x));

d_rel = x(1);
v_f = x(2);
v_rel = v_lead-v_f;
x_vehicle = x(2:4);
delay_size = params.delay_size;
if(~isfield(params,'external_r'))
%     u = dyn_follower_stopper(d_rel,v_rel,v_f,uMin,uMax);
    [dx_vehicle,~] = dyn_follower_stopper(t,x_vehicle,d_rel,v_rel,uMin,uMax,delay_size,exp_num);
else
    if(isempty(params.external_r))
%         u = dyn_follower_stopper(d_rel,v_rel,v_f,uMin,uMax,v_lead);
        [dx_vehicle,~] = dyn_follower_stopper(t,x_vehicle,d_rel,v_rel,uMin,uMax,delay_size,exp_num,v_lead);
    else
%         u = dyn_follower_stopper(d_rel,v_rel,v_f,uMin,uMax,params.external_r);
         [dx_vehicle,~] = dyn_follower_stopper(t,x_vehicle,d_rel,v_rel,uMin,uMax,delay_size,exp_num,params.external_r);
    end
end

u = dx_vehicle(1);

if(non_negative_speed == 1)
%     lambda = leadVehAccel(x(1),x(2)-x(3),x(3));
    kappa = followVehAccel(x(1),v_rel,v_f);
else
%     lambda = 1;
    kappa = 1;
end


dx(1) = v_rel;
dx(2) = kappa*u;
dx(3) = dx_vehicle(2);
dx(4) = dx_vehicle(3);


end

% function lambda = leadVehAccel(d_rel,v_rel,v_av)
%     v_lead = v_rel+v_av;
%     lambda = zeros(size(v_rel));
%     time_costant = 5;
%     lambda(v_lead>0) = 1 - exp(-time_costant*v_lead);
% end


function kappa = followVehAccel(d_rel,v_rel,v_av)
    
    kappa = zeros(size(v_av));
    
    kappa(v_av>0) = 1 ;

end









