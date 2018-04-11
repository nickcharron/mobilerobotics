% -------------------------------------------------------------------------
% ME 640 Autonomous Mobile Robotics                        Nicholas Charron
% Winter 2018 - Final Exam                                         20440122
%
%                   Question 1 - EKF SLAM
%
% -------------------------------------------------------------------------
clear;clc; close all;
%% Question 1 - Part a)
clear;clc;close all

%% Create AVI object
makemovie = 0;
if(makemovie)
    vidObj = VideoWriter('ekfSLAM.mp4');
    vidObj.Quality = 100;
    vidObj.FrameRate = 2;
    open(vidObj);
end

% Load Map   
    plot_map = true;
    MapFigNum = 1;
    map = load_store_map(plot_map, MapFigNum);
    
% Given Information/inputs:

    % Vehicle Dia
    L = 1; % wheel span (dia) [m] 
    
    % Motion Disturbance model
    %R = [0.00001 0     0 ;
    %     0     0.00001 0 ;
    %     0     0     0.000001];
    R = [0.001 0     0 ;
         0     0.001 0 ;
         0     0     0.0001];
    [RE, Re] = eig(R);

    % Inputs
    v = 2.0; % m/s

    % Carrot Distance
    r = 3; % look 3m ahead
    
    % Control Gains
    Kp = 0.5;
    Kd = 0.2;

    % Time
    Tf = 100;   % total 150s sim
    updateFreq = 5; % [Hz]
    dt = 1/updateFreq;
    T = 0:dt:Tf;   
    
    % Number of features
    M = size(map,1);
    
    % Initial Robot State: (x,y,heading)
    %x0 = [1 1 pi/4]';
    x0 = [5 5 0]';
    
    % Prior over robot state
    mu0r = x0; % mean (mu)
    S0rr = 0.00000000001*eye(3);% covariance (Sigma)
    %S0rr = eye(3); % covariance (Sigma)
    
    % Prior over feature map
    S0mm=eye(2);  %predefined covariance for each feature when it is just detected;
    newfeature = 2*ones(M,1);
    featureInitStore = zeros(M,3);  % container to keep measurements before initializing
    
    %Measurement model
    rmax = 15; % Max range
    thmax = 60*pi/180; % 1/2 Field of view

    % Measurement noise
    Qi = [0.001^2 0; 
          0       0.001^2];
    [QiE, Qie] = eig(Qi);

% Generate Path Through Map
    pathPoints = [0   0;
                  5   30;
                  5   55
                  18  55
                  22  30
                  28  5
                  40  5
                  40  30];

%   Test Controller and Path 
    %{
    lookaheadDistance = 3; % 1m
    runCarrotController(pathPoints, L, lookaheadDistance, v, x0, pathPoints(end,:), MapFigNum);
    %}

%% Main Code

% Simulation Initializations
    % size calcs
    n = length(R(:,1)); % Number of vehicle states
    N = n+2*M;  % number of veh + map states
    %m = length(Qi(:,1)); % Number of measurements per feature 
    m = 1; % Number of measurements per feature 
    
    % inputs
    u1 = v*ones(1,length(T));       % velocity input remains constant
    u2 = zeros(1,length(T));        % init. stearing angle inputs
    u = [u1;u2];
    
    % State
    x = [x0 , zeros(3,length(T)-1)];  % init. state matrix
    
    % measurements
    y = zeros(M+1,length(T)); % Num Measurements is size of map + 1 for beacon
    
    % carrot points
    carrot = zeros(2,length(T));    % init. carrot point matrix

    % covariance matrix
    S=zeros(N,N);
    S(1:n,1:n) = S0rr; % initially the map is empty, so the estimate and covariance only have the vehicle info

    % current belief
    mu = zeros(N,1); % current belief
    mu(1:n) = mu0r;

    % belief matrix storage
    mu_S = zeros(N,length(T)); % Belief
    mu_S(:,1) = mu;
    
    % iterators and others    
    count=0;    % count of detected features in the map
    edgeIter = 1;        % set iterator to track which edge you are following
    end_sim = false;
    e_previous = 0;     % set the initial error
    draw_iter = 0;
    
% Plotting setup
    figNum = 1;
    figure(figNum)
    hold on;
    plot(pathPoints(:,1), pathPoints(:,2),'r')
    plot(x0(1),x0(2),'o')
    %plot(endLoc(1),endLoc(2),'x')
    title('Question 1a - Testing Controller & Path')
    %legend('start','end','robot', 'path')
    %legend('path','robot','closest pt', 'carrot')
    grid on
    axis equal
    xlabel('x')
    ylabel('y')
    legend off
    
% Main Loop
for t=2:length(T)
    
 %% Find closest  point
    xi = (x(1:2,t-1))';
    
    % compute minimal distance to edge:
    while (edgeIter < size(pathPoints,1))
        edges = [pathPoints(edgeIter,:) , pathPoints(edgeIter+1,:)];
        [distanceToPt, closestPt, condition] = distToEdge(xi(1:2),edges);
        if(condition == 0 || condition == 1)
            break
            % break while loop, we have found our closest point
        else
            edgeIter = edgeIter + 1;
            % else, check next edge
            if(edgeIter == size(pathPoints,1))
                end_sim = true;
                disp('Ending Sim')
            end

        end
    end
    
    % Calculating carrot point
        % Calculate distance between end point and closest point
        endDist = norm(pathPoints(edgeIter + 1, :) - closestPt);
        
        % Check if carrot point lands on current edge or next edge
        if(endDist < r)
%             disp('Switching to edge num:')
%             disp(edgeIter+1)
            if(edgeIter == size(pathPoints,1)-1)
                edgesNext = [pathPoints(1,:) , pathPoints(2,:)];
                carrot(:,t) = calculateCarrot(edgesNext(1:2), edgesNext, r-endDist)';
            else
                edgesNext = [pathPoints(edgeIter +1,:) , pathPoints(edgeIter+2,:)];
                carrot(:,t) = calculateCarrot(edgesNext(1:2), edgesNext, r-endDist)';
%                disp('Carrot Point (on next line):')
%                disp(carrot(i-1,:))
            end
        else
           carrot(:,t) = calculateCarrot(closestPt, edges, r)'; 
%            disp('Carrot Point (on current line):')
%            disp(carrot(i-1,:))
        end
    
%% Calculate Error and Set Input
        % calculate carrot angle and error
        theta_c = atan2((carrot(2,t) - x(2, t-1)),(carrot(1,t) - x(1, t-1)));
        e =  angleWrap(theta_c - x(3, t-1));
        edot = (e-e_previous); % calculate the difference in error 
        e_previous = e; % set the current error to the previous error
    
    % Calculate Input
    u(2,t) = Kp*e + Kd*edot; 
    
%% Motion projection
    error = RE*sqrt(Re)*randn(3,1);
    x(:,t) = x(:,t-1) + [ u(1,t)*cos(x(3,t-1))*dt;
                          u(1,t)*sin(x(3,t-1))*dt;
                          u(2,t)*dt] + error;

%% Take measurements
    % Beacon range measurement
    y(1,t) = sqrt((x(1,t))^2+(x(2,t))^2) + sqrt(Qi(2,2))*rand(1,1);
    
    % For each feature
    flist = zeros(M,1);
    for i=1:M
        % If feature is visible
        if (inview(map(i,1:2),x(:,t),rmax,thmax))
            flist(i) = 1;

            fStart = i + 1; % changed
            %fEnd = fStart + 1; % end is at same part
            
            % Select a motion disturbance
            d = sqrt(Qi(1,1))*rand(1,1);
            % Determine measurement
            y(fStart,t) = atan(( map(i,2) - x(2,t) )/( map(i,1) - x(1,t) )) - x(3,t) + d;
        end
    end
    
%% Extended Kalman Filter Estimation
    % Prediction update
    mu(1:3) = simpleRobotMotionModel(mu(1:3),u(:,t),dt);
    Gt = simpleLinearizedRobotMotionModel(mu,u(:,t),dt);
    S(1:n,1:n) = Gt*S(1:n,1:n)*Gt' + R;
    
    % Measurement update - First update from beacon
        Ht = range_meas_linearized_model_exam(mu);

        % Measurement update
        K = S*Ht'* inv(Ht*S*Ht'+Qi(1,1));
        mu = mu + K*(y(1,t) - ...
                        range_meas_model_exam(mu(1:3)));
        S = (eye(length(S))-K*Ht)*S;

    % Next Update From all Features
    for i=1:M
        if flist(i) == 1
            fStart = n + (i-1)*2 + 1;
            fEnd = fStart + 1;

            % Feature initialization
            if newfeature(i) == 2
                newfeature(i) = 1;
                % Store first measurement and current pose estimate
                featureInitStore(i,:) = [mu(1), mu(2), y(i+1,t)]; 
            elseif newfeature(i) == 1
                count=count+1;
                xr_1 = featureInitStore(i,1);
                yr_1 = featureInitStore(i,2);
                theta_1 = featureInitStore(i,3);
                xr_2 = mu(1);
                yr_2 = mu(2);
                theta_2 = y(i+1,t);
                
                % Calculate map feature position
                phi = pi + theta_2 - atan2((yr_2-yr_1),(xr_2-xr_1));
                mxi = xr_1 + sin(phi)/sin(theta_1-theta_2) * ... 
                             sqrt((xr_2-xr_1)^2+(yr_2-yr_1)^2)*cos(theta_1);
                myi = yr_1 + sin(phi)/sin(theta_1-theta_2) * ... 
                             sqrt((xr_2-xr_1)^2+(yr_2-yr_1)^2)*sin(theta_1);
                % initialize the measurement
                    %mu(fStart:fEnd) = [mxi ; myi];
                    mu(fStart:fEnd) = [map(i,1) ; map(i,2)];
                % add the initial covariance estimate (I) for the new feature
                S(fStart:fEnd,fStart:fEnd) = S0mm;

                % don't initialize this feature again
                newfeature(i) = 0;

                disp([num2str(count) ' of the ' num2str(M) ' features have been initiated'])
            end

            % Linearization
            % Predicted range
            Ht = bearing_meas_linearized_model_exam(mu,i);

            % Measurement update
            K = S*Ht'* inv(Ht*S*Ht'+Qi(2,2));
            mu = mu + K*(y(i,t) - ...
                        bearing_meas_model_exam(mu(1:3),mu(fStart:fEnd)));
            S = (eye(length(S))-K*Ht)*S;
            
            % Store mu to storage matrix
            mu_S(:,t) = mu;
            
            % In cases if S bemoes not positive definite, manually make it
            % P.D.
            if min(eig(S))<0
               S=S-eye(length(S)).*min(eig(S));
               warning('S was manually made positive definite')
            end

            % warn the user that linearization may not be accurate if the 
            % change in vehicle position is too large compared to input speed
            if norm(mu(1:2)-mu_S(1:2,t-1))>2*u(1,t)*dt
                warning('Linearization may have failed')
            end
        end
    end
    
%% Plotting

    draw_iter = draw_iter + 1;
    if(draw_iter == 1)
        drawbot(mu(1),mu(2),mu(3),L,figNum)
        draw_iter = 0;
    end

    drawnow;
    
    %if(end_sim == true)
    %    hold off
    %    break
    %end
end


