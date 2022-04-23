function plot_sol(path)
plot(path(:,1), path(:,2), 'k', 'DisplayName','Robot Path')
scatter(path(1,1), path(1,2), 'g', 'filled', 'DisplayName','Robot Start')
scatter(path(end,1), path(end,2), 'r', 'filled', 'DisplayName','Robot End')
end