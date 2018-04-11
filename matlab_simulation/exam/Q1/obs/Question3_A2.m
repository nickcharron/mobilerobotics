clc; clear; close all;
% -------------------------------------------------------------------------
% Assignment 2                                   Nicholas Charron, 20440122
% -------------------------------------------------------------------------
% Using the map provided with the homework (IGVCmap.m and IGVCmap.jpg), 
% identify a planning strategy that can be used in conjunction with the 
% carrot planner of part 2) to navigate the Intelligent Ground Vehicle 
% Competition auto-navigation course. The files provided construct the IGVC
% map and store it in a 926x716 element grid, where each cell represents a 
% 10 cm X 10 cm area of the course. Implement your planner and present a 
% combined motion planning solution that finds a straight-line path through
% the environment, then uses the carrot planner to locally drive the 
% vehicle to the goal location. The start and end locations are 
% (40, 5, 3.14159) and (50, 10, not defined) for x,y and ?.

%% ------------------------------------------------------------------------
% Inputs
startAngle = 3.14159;
startPos = [400, 50];
endPos = [500, 100];
motion = 'urdl';
wheelBase = 0.3; % 30cm 
lookaheadDistance = 1; % 1m
vel = 4; % 4 m/s
dxy = 0.1; % convert from m to map scale

% Load Map
[curMap] = IGVCmap();

% Dilate map
    se = strel('square',14);
    dilatedMap = imdilate(curMap,se);
    figure(3)
    %imshowpair(1-curMap,1-dilatedMap,'montage')
    dilatedMap_plot = imdilate(curMap,se);
    colormap('gray')
    imagesc(1-dilatedMap_plot')


% Wavefront planning 
[wavefrontmap, path] = wavefront(dilatedMap, startPos, endPos, motion);

% Removing some vertices on the path to help with line following
keep = 25; % keep every nth number of vertices
verticesIter = 1;
counter = 1;
pathShort = [];
while(verticesIter < size(path,1))
    if(counter == keep)
        pathShort = [pathShort; path(verticesIter,:)];
        counter = 1;
    end
    counter = counter +1;
    verticesIter = verticesIter +1;
end


hold on, plot(pathShort(:,1),pathShort(:,2),'r'), hold off
error('stop')
% Run carrot controller
runCarrotController(pathShort, wheelBase, lookaheadDistance, vel, [startPos,startAngle]', endPos, dxy);










































%% Example
% Convert obstacles to array
% obs = [];
% for i=1:length(obsts)
%     obs = [obs; obsts{i}; obsts{i}(1,:); NaN NaN];
% end
% 
% % Convert map to occupancy grid
% M = max(boundary(:,1));
% N = max(boundary(:,2));
% map = ones(M,N);
% 
% for i=1:M
%     for j = 1:N
%         if (inpolygon(i,j,boundary(:,1), boundary(:,2)))
%             map(i,j) = inpolygon(i,j,obs(:,1), obs(:,2));
%         end
%     end
% end
% 
% %% Set up simulation
% 
% Tmax = 200;
% startPos = tour(1,:);
% endCount = length(tour(:,1));
% curGoal = 1;
% x = zeros(2,Tmax);
% x(:,1) = startPos';
% dx = 0.01;
% t = 1;
% gVcur = [1 1];
% newplan = 1;
% t_start = 1;
% 
% while ((t<Tmax))
%     t=t+1 % Update time
%     pos = x(:,t-1)'
%     tour(curGoal,:)
%     norm(pos-tour(curGoal,:))
%     % Check if it's time to pause and change targets
%     if (norm(pos-tour(curGoal,:))<1)
%         % Pretend to pause and talk
%         x(:,t:t+4) = x(:,t-1)*ones(1,5);
%         t = t+5
%  
%         % Update goal unless at end
%         if (curGoal==endCount)
%             break;
%         end
% 
%         % Compute path to next goal
%         startPos = tour(curGoal,:);
%         curGoal = curGoal+1;
%         endPos = tour(curGoal,:);
%         [wave,curpath] = wavefront(map,startPos,endPos, 'urdl');
%         t_start=t-1;
%     end
%     x(:,t) = curpath(t-t_start,:)'
%     figure(1);clf;hold on
%     imagesc(wave'); 
%     plot(x(1,t_start:t),x(2,t_start:t),'r')
%     plot(tour(curGoal,1), tour(curGoal,2), 'go')
%     pause(0.01);
% end
 