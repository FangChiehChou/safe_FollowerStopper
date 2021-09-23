function [a, v_des] = dyn_modified_follower_stopper_v_cmd(d_rel,v_rel,v,uMin,uMax,r)
    % desired speed from follower stopper contorller

    v_des = modified_FollowerStopper(d_rel,v_rel,v,r);
    %%vehicle longitudinal dynamics
    tau = 0.1;
    a = -v/tau + v_des/tau;
    
    a(a>=uMax) = uMax;
    a(a<=uMin) = uMin;
%     a = zeros(size(v_des));
end



