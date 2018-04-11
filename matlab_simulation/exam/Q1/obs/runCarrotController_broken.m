function runCarrotController(pathPoints, r, v, x0, endLoc, figNum )
%% Description
% This function takes a set of path vertices, vehicle
% wheelbase, a look ahead distance, velocity, a start and end location and 
% runs a carrot controller using a bicycle model to follow the line on 
% the map.
%
% pathPoints - the planned path points in the form of an x and y column vector
% L - distance between the front and rear wheel
% r - carrot lookahead distance
% v - robot velocity
% x0 - initial pose
% endLoc - final pose
% -------------------------------------------------------------------------
%% Function Inputs:
    
    % Gaussian Disturbances
    Sx1 = 0.01;
    Sx2 = 0.01;
    Sx3 = pi/180;

    Rt = [ Sx1^2 0 0; 0 Sx2^2 0; 0 0 Sx3^2];
    [V,D] = eig(Rt);
    
    % Control Gains
    Kp = 0.3;
    Kd = 0.2;
    
    % Turning constraints
    delta_max = pi/6;
    delta_min = -pi/6;

%% Solution
% Define Time of Simulation
Tf = 100; 
dt = 0.2; % 10Hz
t = 0:dt:Tf;

% Building input, state, and error matrices
n = length(t);
u1 = v*ones(1,n);       % velocity input remains constant
u2 = zeros(1,n);        % init. stearing angle inputs
u = [u1;u2];
x = [x0 , zeros(3,n)];  % init. state matrix
%e = zeros(1,n);        % init. error matrix
carrot = zeros(2,n);    % init. carrot point matrix
edgeIter = 1;    % set iterator to track which edge you are following
end_sim = false;
e_previous = 0; % set the initial error
draw_iter = 0;

% Plot 
    figure(1)
    hold on;
    plot(x0(1),x0(2),'o')
    plot(endLoc(1),endLoc(2),'x')
    plot(pathPoints(:,1), pathPoints(:,2),'r')
    title('Question 1 - Testing Controller')
    legend('start','end','robot', 'path')
    %legend('path','robot','closest pt', 'carrot')
    grid on
    axis equal
    xlabel('x')
    ylabel('y')
% Main Loop
for i=2:n
    % Find closest  point
    xi = (x(1:2,i-1))';
    
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
                carrot(:,i) = calculateCarrot(edgesNext(1:2), edgesNext, r-endDist)';
            else
                edgesNext = [pathPoints(edgeIter +1,:) , pathPoints(edgeIter+2,:)];
                carrot(:,i) = calculateCarrot(edgesNext(1:2), edgesNext, r-endDist)';
%                disp('Carrot Point (on next line):')
%                disp(carrot(i-1,:))
            end
        else
           carrot(:,i) = calculateCarrot(closestPt, edges, r)'; 
%            disp('Carrot Point (on current line):')
%            disp(carrot(i-1,:))
        end
    
    % Error
        % calculate location of front tire
        x_r = x(1, i-1);
        y_r = x(2, i-1);
        
        % calculate carrot angle and error
        theta_c = atan2((carrot(2,i) - y_r),(carrot(1,i) - x_r));
        e =  angleWrap(theta_c - x(3, i-1));
        edot = (e-e_previous); % calculate the difference in error 
        e_previous = e; % set the current error to the previous error
    
    % Calculate Input (Steering Angle)
    u(2,i) = Kp*e + Kd*edot;
    %u2(i) = atan(Kp*e/u1(i));
     
    % Check Max/Min Steering Angle
    if u(2,i) > delta_max
        u(2,i) = delta_max;
        disp('max angle +ve')
    elseif u(2,i) < delta_min
        u(2,i) = delta_min;
        disp('max angle -ve')
    end    
    
    % Motion projection
    error = V*sqrt(D)*randn(3,1);
    x(:,i) = x(:,i-1) + [ u(1,i)*cos(x(3,i-1))*dt;
                          u(1,i)*sin(x(3,i-1))*dt;
                          u(2,i)*dt] + error;
    
% Plotting
    draw_iter = draw_iter + 1;
    if (draw_iter == 10)
        drawbot(x(1,n),x(2,n),x(3,n),1,figNum)
        draw_iter = 0;
    end
    %plot(closestPt(1), closestPt(2), 'o')
    %plot((carrot(1,i))',(carrot(2,i))','v')

    drawnow;
    
    if(end_sim == true)
        hold off
        break
    end
end


