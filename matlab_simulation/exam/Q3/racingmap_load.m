function [map] = racingmap_load(plot_map, fignum) 
% Question 3 - Aerial Drone Racing Map

%s = rng(10); % Fix randomness to a standard course.

% Course Map
env = [0 0; 1000 0; 1000 500; 0 500; 0 0];
MME_M = [0 0; 0 100; 20 100; 30 80; 40 100; 60 100; 60 0; 45 0; 45 80; 30 60; 15 80; 15 0; 0 0];
MME_M1 = [4*MME_M(:,1)+100 4*MME_M(:,2)+50];
MME_M2 = [4*MME_M(:,1)+400 4*MME_M(:,2)+50];
MME_E = [0 0; 0 100; 60 100; 60 80; 20 80; 20 60; 60 60; 60 40; 20 40; 20 20; 60 20; 60 0; 0 0];
MME_E1 = [4*MME_E(:,1)+700 4*MME_E(:,2)+50];
MME = [MME_M1; NaN NaN; MME_M2; NaN NaN; MME_E1];
map = [env; NaN NaN; MME];

if(plot_map == true)
figure(fignum); clf; hold on;
plot (map(:,1), map(:,2),'b','LineWidth', 2);
fill (MME_M1(:,1), MME_M1(:,2),'b');
fill (MME_M2(:,1), MME_M2(:,2),'b');
fill (MME_E1(:,1), MME_E1(:,2),'b');
axis([-200 1200 -200 700])
axis equal
end
end

