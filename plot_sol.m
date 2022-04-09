function plot_sol(env_file, sol_file)
fname = env_file;
fid = fopen(fname, 'r');
raw = fread(fid);
str = char(raw');
fclose(fid);
env = jsondecode(str);
path = load(sol_file);
plot_env(env)
plot(path(:,1), path(:,2), 'k', 'DisplayName','Robot Path')
scatter(path(1,1), path(1,2), 'g', 'filled', 'DisplayName','Robot Start')
scatter(path(end,1), path(end,2), 'r', 'filled', 'DisplayName','Robot End')
end