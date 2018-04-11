addpath('./lib');
% -------------------------------------------------------------------------
% ME 640 Autonomous Mobile Robotics                        Nicholas Charron
% Winter 2018 - Final Exam                                         20440122
%
%                   Question 2 - Bicycle State Estimation
%
% -------------------------------------------------------------------------
clear;clc; close all;
%% Question 2 - Part d)
% -------------------------------------------------------------------------
% Given Information:

% Time
f = 2;  % 2Hz
Tf = 100; % s
dt = 1/f; % 0.5s
T = 0:dt:Tf; % time steps
%plotTimeSteps = 10; % plot every 10 timesteps

% Wheel base
L = 1; % 1m
robotScale = 5;


%% Trajectory Generation
    % Turning constraints
    delta_max = 25*pi/180;
    delta_min = -25*pi/180;
    

    % Control inputs - these are only used to generate the trajectory, not used
    % in particle filter update (no motion update)
    v = 5; % m/s
    delta = 0.05*sin(T*0.1); % rad
    
    % Building input matrices
    n = length(T);
    u1 = v*ones(1,n);
    u2 = delta.*ones(1,n);

    for i = 1:n   % Vehicle movement constraints
        if u2(i) > delta_max
            u2(i) = delta_max;
        elseif u2(i) < delta_min
            u2(i) = delta_min;
        end    
    end
    
    % Inital pose
    x0 = [0;0;0;0;0]; % [x,y,theta,v,delta]

    xi = [x0, zeros(5,n)]; % init pose matrix
    
    figure(1)
    % Draw start position:
    drawrobot(xi(1,1),xi(2,1),xi(3,1),L,u2(1),robotScale,1)
    grid on
    currTimeStep = 1;
    plotTimeSteps = 10; % plot every 10 timesteps

    for i=2:length(T)
    hold on
    currTimeStep = currTimeStep+1;
    
    xi(1:3,i) = xi(1:3,i-1) + u1(i)*[cos(xi(3,i-1))*dt; sin(xi(3,i-1))*dt; tan(u2(i))/L*dt];   
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
    plot(xi(1,1:end-1),xi(2,1:end-1),'r')
    title('Fig. 2d-1: Full trajectory')
    xlabel('x (m)')
    ylabel('y (m)')
    robotLegend = ['Robot, Scale=',num2str(robotScale,'%d')];
    legend(robotLegend ,'Heading & Stearing Angle (deg)','Entire Path')

%% Particle Filter Implementation

% Noise parameters

% Disturbance model
R = [0.001 0     0      0    0;
     0     0.001 0      0    0;
     0     0     0.0005 0    0;
     0     0     0      0.01 0;
     0     0     0      0    0.0004];
[RE, Re] = eig(R);

% Measurement noise
    
    % GPS
        Qg = [0.5^2 0      0    0;
              0     0.5^2  0    0;
              0     0      1    0;
              0     0      0    1]; % m^2 & (m/s)^2
        [QEg, Qeg] = eig(Qg);
    % Steering
        Qs = (2*pi/180)^2; % rad^2
        
    % Wheel encoder
        p_miss = 1/100; % probability of missing a pulse
        T_meas = 1; % [s] time between measurements
        c_tire = 2.1 % [m] circumference of tire

% Number of particles
D = 100;

% Prior - uniform over a given range (-5 5 position, and 0 2*pi heading - changed)
X = zeros(5, D); % initialize all particles at pose 0
X(1:2, :) = 2 * rand(2, D) - 1; % set random x and y positions for all samples
X(3, :) = pi / 4 * rand(1, D) - pi / 8; % set random headings for all samples
X(4:5,:) = zeros(2,D);  % set velocity and delta to zero - fair assumption
X0 = X;  % assign to x0
Xp = X;  % assign to current predicted x
w = zeros(1, D); % init weight functions (equal probabilities)

%% HERE

% Simulation Initializations
n = length(x0); % number of states
x = zeros(n, length(T)); % init all states
x(:, 1) = x0;   % set initial state
m = length(Q(:, 1)); % number of measurements at each timestep
y = zeros(m, length(T));    % init measurements
mf = zeros(2, length(T));   % measurement at each timestep
muParticle_S = zeros(1, D, length(T)); % state of each particle at each timestep

% plot intial state
plot_localization_state(1, map, x, X, D);
axis([-4 6 -1 7]);
title('Particle Filter Localization')

% Main loop
for t = 2:length(T)    
    % update state
    e = RE * sqrt(Re) * randn(n, 1);
    x(:, t) = [x(1, t - 1) + u(1, t) * cos(x(3, t - 1)) * dt;
              x(2, t - 1) + u(1, t) * sin(x(3, t - 1)) * dt;
              x(3, t - 1) + u(2, t) * dt] + e;
    
    % take measurement
    % pick feature
    mf(:, t) = closestfeature(map, x(:, t));
    % Select a motion disturbance
    d = QE * sqrt(Qe) * randn(m, 1);
    % determine measurement
    switch(meas) 
        case 1  % 1 - range
            r = range(mf(1, t), x(1, t), mf(2, t), x(2, t));
            y(:, t) = r + d;
        case 2  % 2 - bearing
            b = bearing(mf(1, t), mf(2, t), x(1, t), x(2, t), x(3, t));
            y(:, t) = b + d;
        case 3  % 3 - both
            r = range(mf(1, t), x(1, t), mf(2, t), x(2, t));
            b = bearing(mf(1, t), mf(2, t), x(1, t), x(2, t), x(3, t));
            y(:, t) = [r; b] + d;
    end

    % particle filter estimation
    [X] = pf_localization(t, dt, meas, n, D, R, Q, d, X, Xp, mf, y, u);

    % store particle filter states
    muParticle = mean(X);
    SParticle = var(X);    
    muParticle_S(1, :, t) = muParticle;
    
    % record
    if (makemovie) 
        % plot results
        plot_localization_results(1, meas, map, t, x, X, y, mf, D);
        axis equal
        axis([-5 7 -2 8]);
        title('Particle Filter Localization')

        % record plot frame
        writeVideo(vidObj, getframe(gca)); 
    end
end

% close video object
if (makemovie) 
    close(vidObj); 
end