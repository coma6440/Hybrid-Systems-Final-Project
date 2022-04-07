function plot_env(env)
fprintf("Plotting environment: %s\n", env.env_name);
figure
hold on
grid on

% Plot obstacles
for i = 1:length(env.obstacles)
    o = env.obstacles(i);
    x = o.pts(:,1);
    y = o.pts(:,2);
    fill(x,y,'k', 'DisplayName',o.name)
end

% Plot regions
n_regions = length(env.regions);
cm = cool(n_regions);
for i = 1:n_regions
    r = env.regions(i);
    x = r.pts(:,1);
    y = r.pts(:,2);
    fill(x,y,cm(i,:),'DisplayName', r.name)
end

xlabel("X Position")
ylabel("Y Position")
legend('Location','eastoutside')
end