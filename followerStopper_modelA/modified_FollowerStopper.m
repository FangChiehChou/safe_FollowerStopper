function v_des = modified_FollowerStopper(d_rel,v_rel,v_av,r)
% d_rel : relative distance (x_l - x_f)
% v_rel : relative speed  (v_l-v_f)
% v_av : subject vehicle absolute speed
% v_des : desired speed of the subject vehicle at next time step.
% d_rel,v_rel,v_av, and v_des are double arrays (matrices) of same dimension

%% follower stopper parmaeters
    w = zeros(3,1);
    alpha = zeros(3,1);
    w(1) = 4.5;
    alpha(1) = 1.5;
    w(3) = 6.0;
    alpha(3) = 0.5;
    w(2) = (w(1)+w(3))/2;
    alpha(2) = (alpha(1)+alpha(3))/2;
    if(nargin<4)
        r = 30;
    end
    
    %v_rel_star = min(v_rel,0)
    v_rel_star = v_rel;
    v_rel_star(v_rel >= 0) = 0;
   
    h = 0.4;
    
    x1 = w(1) + (0.5*v_rel_star.^2)/alpha(1) + h*v_av;
    x2 = w(2) + (0.5*v_rel_star.^2)/alpha(2) + h*v_av;
    x3 = w(3) + (0.5*v_rel_star.^2)/alpha(3) + h*v_av;
    
    v_lead = v_av+v_rel;
    
    
    %v = min([max([v_lead,0]),r]);
    v = v_lead;
    v(v_lead<0) = 0;
    
    v(v_lead>r) = r;
  
    v_des = zeros(size(d_rel));  
    %safe region
    v_des(d_rel>x3) = r;  
    %adaptation region 1
    temp_v_des = v+(r-v).*(d_rel-x2)./(x3-x2);
    v_des(d_rel>x2 & d_rel<=x3) = temp_v_des(d_rel>x2 & d_rel<=x3);   
    %adaptation region 2
    temp_v_des = v.*(d_rel-x1)./(x2-x1);
    v_des(d_rel>x1 & d_rel<=x2) = temp_v_des(d_rel>x1 & d_rel<=x2); 
    
%     if(d_rel>x(3))
%         v_des = r;
%     elseif (d_rel>x(2) && d_rel<=x(3))
%         v_des = v*(d_rel-x(1))/(x(2)-x(1));  
%     elseif (d_rel>x(1) && d_rel <= x(2))
%         v_des = v+(r-v)*(d_rel-x(2))/(x(3)-x(2));
%     else
%         v_des = 0;
%     end
     
    
end