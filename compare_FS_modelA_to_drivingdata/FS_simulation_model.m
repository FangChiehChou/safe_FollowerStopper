%% 
function dx = FS_simulation_model(t,x,uMin,uMax,r,v_lead)

%% system states
%d_rel = x(1) : relative distance between lead vehicle and the following vehicle
%v_rel = x(2) : relative speed between the leader and follower

%% system inputs
% u : acceleration of the following vehicle
% data_time/data_lead_spd : lead vehicle speed time series

%% parameters 
% uMin/uMax : acceleraiton lower and upper bound of the follower vehicle
% non_negative_speed : whether imposing state constraint or not

% v_lead = 10;

non_negative_speed = 0;

dx = zeros(size(x));

d_rel = x(1);
v_f = x(2);

v_rel = v_lead-v_f;


u = dyn_follower_stopper(d_rel,v_rel,v_f,uMin,uMax,r);

if(non_negative_speed == 1)
%     lambda = leadVehAccel(x(1),x(2)-x(3),x(3));
    kappa = followVehAccel(x(1),v_rel,v_f);
else
%     lambda = 1;
    kappa = 1;
end


dx(1) = v_lead-x(2);
dx(2) = kappa*u;


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









