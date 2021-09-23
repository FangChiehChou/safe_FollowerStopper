function dx = CFM_TwoCars_model(t,x,uMin,uMax,params,disturbance_sequence)

%% system states
%d_rel = x(1) : relative distance between lead vehicle and the following vehicle
%v_lead = x(2) : lead vehicle speed
%v_follower = x(3) : speed of the follower vehicle

%% system inputs
% d : acceleration of the lead vehicle
% u : acceleration of the following vehicle

%% parameters 
% dMin/dMax : acceleration lower and upper bound of the lead vehicle
% uMin/uMax : acceleraiton lower and upper bound of the follower vehicle
% non_negative_speed : whether imposing state constraint or not

non_negative_speed = 1;

dMax = 3.5;  %acceleraiton lower bound and upper bound of the lead vehicle
dMin = -3.5;

d = dMin;
d = interp1();  %Vq = interp1(X,V,Xq);

dx = zeros(3,1);

v_lead = x(2);
if(~isfield(params,'external_r'))
    u = dyn_follower_stopper(x(1),x(2)-x(3),x(3),uMin,uMax);
else
    if(isempty(params.external_r))
        u = dyn_follower_stopper(x(1),x(2)-x(3),x(3),uMin,uMax,v_lead);
    else
        u = dyn_follower_stopper(x(1),x(2)-x(3),x(3),uMin,uMax,params.external_r);
    end
end

if(non_negative_speed == 1)
    lambda = leadVehAccel(x(1),x(2)-x(3),x(3));
    kappa = followVehAccel(x(1),x(2)-x(3),x(3));
else
    lambda = 1;
    kappa = 1;
end

dx(1) = x(2)-x(3);
dx(2) = lambda*d;
dx(3) = kappa*u;


end


function lambda = leadVehAccel(d_rel,v_rel,v_av)
    v_lead = v_rel+v_av;
    lambda = zeros(size(v_rel));
    time_costant = 5;
    lambda(v_lead>0) = 1 - exp(-time_costant*v_lead);
end


function kappa = followVehAccel(d_rel,v_rel,v_av)
    
    kappa = zeros(size(v_av));
    
    kappa(v_av>0) = 1 ;

end


