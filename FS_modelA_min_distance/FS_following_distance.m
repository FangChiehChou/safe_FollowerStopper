function cost =  FS_following_distance(x0,params)
%optimize d_s 
%x0: initial condition 
%t_s: time steps 
%d_s: disturbance at each time step

tspan = [0 30];
uMin = -5.3;
uMax = 3.5;


[t,y] = ode45(@(t,x) CFM_TwoCars_model(t,x,uMin,uMax,params),tspan,x0);    

d_rel = y(:,1);
v_follower = y(:,3);

h = 0.4;  %desired time headway
cost = min(d_rel-h*v_follower);




