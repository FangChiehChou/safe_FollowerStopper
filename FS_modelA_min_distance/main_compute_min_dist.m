
clear all
clc

%%

params.external_r = 30;
params.delay_size = 0.0;
params.d_Min = -3.5;


%% test function


x0 = [100;12;20];   %relative distance / lead vehicle speed/ following vehicle speed


cost =  FS_following_distance(x0,params)