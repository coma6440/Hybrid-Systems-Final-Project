%% Housekeeping
close all
clear
clc

%% Obstacles
obstacles(1).pts = make_box(1.5, 5, 1, 8);
obstacles(1).name = "o1";
obstacles(2).pts = make_box(3.5, 5, 1, 8);
obstacles(2).name = "o2";
obstacles(3).pts = make_box(5.5, 5, 1, 8);
obstacles(3).name = "o2";
%% Regions
regions(1).pts = make_box(0.5,0.5,1,1);
regions(1).name = "p0";
regions(2).pts = make_box(2.5,4,1,0.5);
regions(2).name = "p1";
regions(3).pts = make_box(4.5,7,1,0.5);
regions(3).name = "p2";
regions(4).pts = make_box(9,5,2,2);
regions(4).name = "p3";
%% Making the Environment Object
data.obstacles = obstacles;
data.regions = regions;
data.env_name = "env1";

figure
hold on
plot_env(data);

%% Save the environment
fname = "../envs" + filesep + data.env_name + ".json";
fid = fopen(fname, 'w');
if fid == -1
    error('Cannot create JSON file'); 
end
fwrite(fid, jsonencode(data), 'char');
fclose(fid);