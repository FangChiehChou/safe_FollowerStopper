clear all
clc

%%
file  = dir('./catvehicle-cmd_vel.csv');
data = readtable([file.folder,'/',file.name]);
cat_cmd_spd.time = data{:,1};
cat_cmd_spd.data = data{:,2};


file  = dir('./catvehicle-distanceEstimator-dist.csv');
data = readtable([file.folder,'/',file.name]);
cat_distEsimtation.time = data{:,1};
cat_distEsimtation.data = data{:,2};

file  = dir('./catvehicle-region.csv');
data = readtable([file.folder,'/',file.name]);
cat_region.time = data{:,1};
cat_region.data = data{:,2};

file  = dir('./catvehicle-velocity_ref.csv');
data = readtable([file.folder,'/',file.name]);
cat_ref_vel.time = data{:,1};
cat_ref_vel.data = data{:,2};

file  = dir('./catvehicle-v_relative_est.csv');
data = readtable([file.folder,'/',file.name]);
cat_v_rel_estimation.time = data{:,1};
cat_v_rel_estimation.data = data{:,2};

file  = dir('./catvehicle-vel.csv');
data = readtable([file.folder,'/',file.name]);
cat_vel.time = data{:,1};
cat_vel.data = data{:,2};


file = dir('./toyota-cmd_vel.csv');
data = readtable([file.folder,'/',file.name]);
toyota_cmd_vel.time = data{:,1};
toyota_cmd_vel.data = data{:,2};

file = dir('./toyota-vel.csv');
data = readtable([file.folder,'/',file.name]);
toyota_vel.time = data{:,1};
toyota_vel.data = data{:,2};

file = dir('./catvehicle-v_leader_est.csv');
data = readtable([file.folder,'/',file.name]);
v_leader_est.time = data{:,1};
v_leader_est.data = data{:,2};

