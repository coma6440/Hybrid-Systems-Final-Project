%% Housekeeping
close all
clear
clc

%% Obstacles
obstacles(1).pts = [0,0.9;
          1.1,0.9;
          1.1,1.1;
          0.5, 1.4;
          0,1.1];
obstacles(1).name = "o1";
obstacles(2).pts = [0.9, 0;
                    0.9, 0.2;
                    1, 0.2;
                    1, 0];
obstacles(2).name = "o2";
%% Regions
regions(1).pts = [0.9, 0.3;
                  1.1, 0.3;
                  1.1, 0.5;
                  0.9, 0.5];
regions(1).name = "p0";
regions(2).pts = [1.5, 1.6;
                  1.6, 1.6;
                  1.6, 1.7;
                  1.5, 1.7];
regions(2).name = "p1";
regions(3).pts = [0.2, 1.7;
                  0.3, 1.7;
                  0.3, 1.8;
                  0.2, 1.8];
regions(3).name = "p2";
%% Making the Environment Object
data.obstacles = obstacles;
data.regions = regions;
data.env_name = "env0";

plot_env(data);

%% Save the environment
fname = "envs" + filesep + data.env_name + ".json";
fid = fopen(fname, 'w');
if fid == -1
    error('Cannot create JSON file'); 
end
fwrite(fid, jsonencode(data), 'char');
fclose(fid);