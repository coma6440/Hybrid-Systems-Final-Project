%% Housekeeping
close all
clear
clc

%% Load the Configuration file
c = load_json("../configs/config0.json");
env = load_json("../envs/" + c.env);
sol_file = "../sols/" + c.sol;
if isfile(sol_file)
    path = load(sol_file);
    figure
    hold on
    grid on
    plot_env(env)
    plot_sol(path)
    set_ltl_title(c.safety, c.cosafety)
else
    figure
    hold on
    grid on
    plot_env(env)
end