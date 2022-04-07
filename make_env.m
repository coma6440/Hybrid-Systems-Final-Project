%% Housekeeping
close all
clear
clc

%% Obstacles
o1.pts = [0,0.9;
          1.1,0.9;
          1.1,1.1;
          0,1.1];
o1.name = "o1";

obstacles = [o1];
%% Regions
r1.pts = [0.9, 0.3;
          1.1, 0.3;
          1.1, 0.5;
          0.9, 0.5];
r1.name = "p0";
r2.pts = [1.5, 1.6;
          1.6, 1.6;
          1.6, 1.7;
          1.5, 1.7];
r2.name = "p1";
r3.pts = [0.2, 1.7;
          0.3, 1.7;
          0.3, 1.8;
          0.2, 1.8];
r3.name = "p2";
regions = [r1, r2, r3];
%% Making the Environment Object
data.obstacles = obstacles;
data.regions = regions;
data.env_name = "env0";

plot_env(data);

%% Save the environment
fname = data.env_name + ".json";
fid = fopen(fname, 'w');
if fid == -1
    error('Cannot create JSON file'); 
end
fwrite(fid, jsonencode(data), 'char');
fclose(fid);