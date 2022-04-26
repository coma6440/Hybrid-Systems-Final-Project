%% Housekeeping
close all
clear
clc

%% Load the Configuration file
c = load_json("../configs/config0.json");
env = load_json("../envs/" + c.env);
sol_file = "../sols/" + c.sol;
decomp_file = "../envs/" + c.decomp;
if isfile(sol_file)
    decomp = load_decomp(decomp_file);
    path = load(sol_file);
    figure
    hold on
    plot_env(env)
    plot_decomp(decomp)
    plot_sol(path)
    set_ltl_title(c.safety, c.cosafety)
else
    figure
    hold on
    plot_env(env)
end