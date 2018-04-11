function [map] = load_store_map(plot_map, fignum)
% Question #1 - EKF SLAM with bearing only for features

%% Map 
s = rng('default'); % Fix randomness to a standard map.

% Store boundary
Ox = 0; Oy = 0; Lx = 45; Ly = 60;
store = [Ox Oy; Ox+Lx Oy; Ox+Lx Oy+Ly; Ox Oy+Ly; Ox Oy];

% Shelves
SOx = 10; SOy = 10; SLx = 5; SLy = 35;
shelving1 = [SOx SOy; SOx+SLx SOy; SOx+SLx SOy+SLy; SOx SOy+SLy; SOx SOy];
SOx = 30; SOy = 10; SLx = 5; SLy = 35;
shelving2 = [SOx SOy; SOx+SLx SOy; SOx+SLx SOy+SLy; SOx SOy+SLy; SOx SOy];
shelves = [shelving1; NaN NaN; shelving2];

% Features
N = 200;
features =  [Ox + Lx*rand(N,1), Oy + Ly*rand(N,1),[1:N]']; %(x,y,label)
beacon = [0 0]; % Homing beacon location
% Feature map
inshelves = inpolygon(features(:,1), features(:,2), shelves(:,1), shelves(:,2));
map = features(find(~inshelves),:);

% Plotting of map
    if (plot_map == true)
        figure(fignum); clf; hold on;
        plot (store(:,1), store(:,2), 'g', 'LineWidth', 2);
        plot (map(:,1), map(:,2),'bx','MarkerSize', 4);
        plot (shelves(:,1), shelves(:,2), 'r', 'LineWidth', 1)
        plot (beacon(1), beacon(2), 'mo', 'MarkerSize',6,'LineWidth', 2)
        legend('Boundary', 'Features', 'Shelving', 'Beacon')
        title('Feature map')
        axis equal
    end
end