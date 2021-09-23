clear all
clc

%% Load driving data  
%% data set = 07082020  / OR Matt Nice's I-24 dataset


filename = '2021-03-23-21-50-02_2T3MWRFVXLW056972_masterArray.csv';
T = readtable(filename);


time = T.Time;
speed = T.Velocity;

sim_time = time - time(1);
sim_speed = speed/3.6;

figure()
plot(sim_time,sim_speed)



