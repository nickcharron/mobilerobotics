%clc; clear; close all;
% -------------------------------------------------------------------------
%% -------------------------------------------------------------------------
% ME 640 Autonomous Mobile Robotics                        Nicholas Charron
% Winter 2018 - Final Exam                                         20440122
%
%                   Question 2 - DRONE RACING
%
% -------------------------------------------------------------------------

%% ------------------------------------------------------------------------
clc; close all;
% Inputs
startPos = [30, 30];
endPos = [950, 450];
motion = 'urdl';
wheelBase = 0.3; % 30cm 
lookaheadDistance = 1; % 1m
vel = 4; % 4 m/s
dxy = 0.1; % convert from m to map scale

% Load Environment
plot_map = true;
fignum = 1;
env = racingmap_load(plot_map,fignum);

% recalculate map
    recalcMap = false;
    if(recalcMap ==true)
    curMap = zeros(1000,1000);
        for i = 1:1000
            for j = 1:500
                if(~inpolygon(i, j, env(:,1),env(:,2)))
                    curMap(i,j) = 1;
                end
            end
        end 
    end

% inpolygon(samples(:,1), samples(:,2), env(:,1),env(:,2));
% Dilate map
    se = strel('square',14);
    dilatedMap = imdilate(1-curMap,se);
    figure(2)
    %imshowpair(1-curMap,1-dilatedMap,'montage')
    dilatedMap_plot = imdilate(curMap,se);
    colormap('gray')
    imagesc(1-dilatedMap_plot')


% Wavefront planning 
[wavefrontmap, path] = wavefront(1-dilatedMap, startPos, endPos, motion);

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

figure(1)
hold on
plot(pathShort(:,1),pathShort(:,2),'r')
plot(pathShort(1,1),pathShort(1,2),'o',pathShort(end,1),pathShort(end,2),'x')
hold off

%% Non-Holonomic Path

x0 = [startPos , 0, 0]';
xf = endPos;
umax = 17;
vmax = 10;
u_o = [ 0      -umax;
       -umax    0;
        0       umax;
        umax    0;
       -umax/2  umax/2;
        umax/2 -umax/2];
dt = 0.1;
Tf = 100;
T = 0:dt:Tf;
done = 0;
% iterators and others    
count=0;    % count of detected features in the map
edgeIter = 1;        % set iterator to track which edge you are following
end_sim = false;
e_previous = 0;     % set the initial error
x = zeros(4,length(T));
u = zeros(2,length(T));
x(:,1) = x0;
xi = zeros(4,1);
cost = zeros(size(u_o,1),1);
A = [1 dt 0 0;
     0 1  0 0;
     0 0  1 dt;
     0 0  0 1];

B = [0 0; dt 0; 0 0; 0 dt];

R = [0.001 0     0 ;
     0     0.001 0 ;
     0     0     0.0001];
[RE, Re] = eig(R);

Kc = 1;
Kg = 0.05;

updateTimes = 5;
 
figure(1)
hold on
grid on
title('Robot Constrained Path')
xlabel('x')
xlabel('y')

for t = 2:length(T)
 
    for i = 1:size(u_o)
        ui = u_o(i,:);
        x_last = x(:,t-1);        
        for k = 1:updateTimes
            xi = A*x_last+B*ui';
            x_last = xi;
            % Check velocity constraints
                if(xi(2)>vmax)
                   xi(2) = vmax ;
                end
                if(xi(4)>vmax)
                   xi(4)=vmax;
                end
                if(xi(2)<-vmax)
                   xi(2) = -vmax;
                end
                if(xi(4)<-vmax)
                   xi(4)=-vmax;
                end
        end
        
        % compute minimal distance to edge:
        while (edgeIter < size(pathShort,1))
            edges = [pathShort(edgeIter,:) , pathShort(edgeIter+1,:)];
            [distanceToPt, closestPt, condition] = distToEdge([xi(1),xi(3)],edges);
            if(condition == 0 || condition == 1)
                break
                % break while loop, we have found our closest point
            else
                edgeIter = edgeIter + 1;
                % else, check next edge
                if(edgeIter == size(pathShort,1))
                    end_sim = true;
                    disp('Ending Sim')
                end
            end
        end
            
        e_g = sqrt((xf(1)-xi(1))^2 + (xf(2)-xi(3))^2);
        cost(i,1) = Kc * distanceToPt + Kg * e_g; 
    end  
    [min_cost, min_index] = min(cost);
     u_chosen = u_o(min_index,:)';

        x(:,t) = A*x(:,t-1)+B*u_chosen;
        u(:,t) = u_chosen;
        
        % Check velocity constraints
        if(x(2,t)>vmax)
           x(2,t) = vmax ;
        end
        if(x(4,t)>vmax)
           x(4,t)=vmax;
        end
        if(x(2,t)<-vmax)
           x(2,t) = -vmax; 
        end
        if(x(4,t)<-vmax)
           x(4,t)=-vmax;
        end

if(end_sim == true)
   break
end    

plot(x(1,t),x(2,t),'o')
drawnow
end
hold off

figure(3)
hold on
grid on
plot(T,u(1,:),T,u(2,:),'-.')
legend('ux', 'uy')
title('Acceleration Commands')
xlabel('t (s)')
ylabel('a (m/s^2)')
hold off


figure(4)
hold on
grid on
plot(T,x(2,:),T,x(4,:),'-.')
legend('Vx', 'Vy')
title('Velocity')
xlabel('t (s)')
ylabel('v (m/s)')
hold off







 