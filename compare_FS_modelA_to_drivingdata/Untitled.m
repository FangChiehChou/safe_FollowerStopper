

r = 30;
uMax = 1.5;   %+1.5 m/s^2
uMin = -3.0;  %-3.0 m/s^2

d_rel_temp = 35;
v_rel_temp = -2;

vehicle_speed = 15;
dyn_follower_stopper(d_rel_temp,v_rel_temp,vehicle_speed,uMin,uMax,r)