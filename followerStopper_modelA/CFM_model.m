function dx = CFM_model(t,x,uMin,uMax)

%% system states
%d_rel = x(1) : relative distance between lead vehicle and the following vehicle
%v_rel = x(2) : relative speed between lead vehicle and the following vehicle : (v_{lead} - v_{follower})
%v_f = x(3) : speed of the follower vehicle

%% system inputs
% d : acceleration of the lead vehicle
% u : acceleration of the following vehicle

%% parameters 
% dMin/dMax : acceleration lower and upper bound of the lead vehicle
% uMin/uMax : acceleraiton lower and upper bound of the follower vehicle
% non_negative_speed : whether imposing state constraint or not

no_negative_speed = 1;

dMax = 3.5;  %acceleraiton lower bound and upper bound of the lead vehicle
dMin = -3.5;

d = dMin;

dx = zeros(3,1);

u = dyn_follower_stopper(x(1),x(2),x(3),uMin,uMax);

if(no_negative_speed == 1)
    lambda = leadVehAccel(x(1),x(2),x(3));
    kappa = followVehAccel(x(1),x(2),x(3));
else
    lambda = 1;
    kappa = 1;
end

dx(1) = x(2);
dx(2) = lambda*d-kappa*u;
dx(3) = kappa*u;


end


function lambda = leadVehAccel(d_rel,v_rel,v_av)
    v_lead = v_rel+v_av;
    lambda = zeros(size(v_rel));
    time_costant = 5;
    
    speed_smoothing = 0;
    if(speed_smoothing == 1)
        lambda(v_lead>0) = 1 - exp(-time_costant*v_lead);
    else
        lambda(v_lead>0) = 1;
    end
end


function kappa = followVehAccel(d_rel,v_rel,v_av)
    
    kappa = zeros(size(v_av));
    
    kappa(v_av>0) = 1 ;

end


