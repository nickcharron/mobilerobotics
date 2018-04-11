% -------------------------------------------------------------------------
% ME 640 Autonomous Mobile Robotics                        Nicholas Charron
% Winter 2018 - Final Exam                                         20440122
%
%                   Question 2 - Bicycle State Estimation
%
% -------------------------------------------------------------------------
clear;clc; close all;
%% Question 2 - Part a)
% -------------------------------------------------------------------------
% Given Information:

% Time
f = 2;  % 2Hz
Tf = 100' % s
dt = 1/f; % 0.5s
t = 0:dt:Tf; % time steps
plotTimeSteps = 10; % plot every 10 timesteps

% Wheel base
L = 1; % 1m
robotScale = 5;

% Gaussian Disturbances - Neglect for part a
    % Sx1 = 0.01;
    % Sx2 = 0.01;
    % Sx3 = pi/180;
    % 
    % Rt = [ Sx1^2 0 0; 0 Sx2^2 0; 0 0 Sx3^2];
    % [V,D] = eig(Rt);

% Given Inputs
v = 8; % m/s
delta = 0.1*sin(t./10)+0.05*cos(t./12)+t./16; % rad

% Turning constraints - ignore for part a
    % delta_max = pi/6;
    % delta_min = -pi/6;

% Building input matrices
n = length(t);
u1 = v*ones(1,n);
u2 = delta.*ones(1,n);
   
    % for i = 1:n   % Vehicle movement constraints
    %     if u2(i) > delta_max
    %         u2(i) = delta_max;
    %     elseif u2(i) < delta_min
    %         u2(i) = delta_min;
    %     end    
    % end

% Inital pose
x0 = [0;0;0];

xi = [x0, zeros(3,n)]; % init pose matrix

figure(1)
% Draw start position:
drawrobot(xi(1,1),xi(2,1),xi(3,1),L,u2(1),robotScale,1)
grid on
currTimeStep = 1;

for i=2:length(t)
    hold on
    currTimeStep = currTimeStep+1;
    
    e = zeros(3,1); % set zero error for part a
    %e = V*sqrt(D)*randn(3,1);
    
    xi(:,i) = xi(:,i-1) + u1(i)*[cos(xi(3,i-1))*dt; sin(xi(3,i-1))*dt; tan(u2(i))/L*dt] + e;   
    %plot(xi(1,i),xi(2,i),'color','k')
    plot(xi(1,i),xi(2,i),'.')

    % Draw robot at given timesteps
    if(currTimeStep == plotTimeSteps)
        currTimeStep = 0;
        %fprintf("Drawing Robot pose no. %d\n",i)
        drawrobot(xi(1,i),xi(2,i),xi(3,i),L,u2(i),robotScale,1)
        point_text = ['\theta:' ,num2str(angleWrap(xi(3,i))*180/pi,'%.1f') , ...
                            '\delta: ', num2str(angleWrap(u2(i))*180/pi,'%.1f')];
        text(xi(1,i),xi(2,i), point_text)
    end
    drawnow
end

% Plot all poses:
plot(xi(1,:),xi(2,:),'r')
title('Fig. 2a-1: Simulation')
xlabel('x (m)')
ylabel('y (m)')
robotLegend = ['Robot, Scale=',num2str(robotScale,'%d')];
legend(robotLegend ,'Heading & Stearing Angle (deg)','Entire Path')



