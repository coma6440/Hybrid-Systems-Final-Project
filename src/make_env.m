%% Housekeeping
close all
clear
clc

%% Obstacles
obstacles(1).pts = make_box(15, 9.5, 30, 1);
obstacles(1).name = "o1";
obstacles(2).pts = make_box(15, 0.5, 30, 1);
obstacles(2).name = "o2";
obstacles(3).pts = make_box(4.5, 3.5, 1, 5);
obstacles(3).name = "o4";
obstacles(4).pts = make_box(9.5, 6.5, 1, 5);
obstacles(4).name = "o5";
obstacles(5).pts = make_box(14.5, 3.5, 1, 5);
obstacles(5).name = "o5";
obstacles(6).pts = make_box(19.5, 6.5, 1, 5);
obstacles(6).name = "o7";
obstacles(7).pts = make_box(24.5, 3.5, 1, 5);
obstacles(7).name = "o8";
%% Regions
regions(1).pts = make_box(2,5,4,1);
regions(1).name = "p0";
regions(2).pts = make_box(7,5,4,1);
regions(2).name = "p1";
regions(3).pts = make_box(12,5,4,1);
regions(3).name = "p2";
regions(4).pts = make_box(17,5,4,1);
regions(4).name = "p3";
regions(5).pts = make_box(22,5,4,1);
regions(5).name = "p4";
regions(6).pts = make_box(29,5,1,1);
regions(6).name = "p5";
regions(7).pts = make_box(4.5,8,9,2);
regions(7).name = "p6";
regions(8).pts = make_box(14.5,8,9,2);
regions(8).name = "p7";
regions(9).pts = make_box(24.5,8,9,2);
regions(9).name = "p8";
regions(10).pts = make_box(9.5,2,9,2);
regions(10).name = "p9";
regions(11).pts = make_box(19.5,2,9,2);
regions(11).name = "p10";

%% Making the Environment Object
data.obstacles = obstacles;
data.regions = regions;
data.env_name = "env2";

figure
hold on
plot_env(data);
axis tight
grid on


%% Save the environment
fname = "../envs" + filesep + data.env_name + ".json";
fid = fopen(fname, 'w');
if fid == -1
    error('Cannot create JSON file'); 
end
fwrite(fid, jsonencode(data), 'char');
fclose(fid);