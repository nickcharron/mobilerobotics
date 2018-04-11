%% Flyover Localization
clear;clc;close all

% Time
f = 2;  % 2Hz
Tf = 100; % s
dt = 1/f; % 0.5s
T = 0:dt:Tf; % time steps

% Wheel base
L = 1; % 1m
v = 5;

% Initial State
x0 = [0 0 0 v 0]';

% Disturbance model
R = [0.001 0     0      0    0;
     0     0.001 0      0    0;
     0     0     0.0005 0    0;
     0     0     0      0.01 0;
     0     0     0      0    0.0004];
[RE, Re] = eig(R);


% Measurement noise
    
    % GPS & steering
        Q = [0.5^2  0      0    0    0;
              0     0.5^2  0    0   0;
              0     0      1    0   0;
              0     0      0    1   0
              0     0      0    0   (pi/90)^2]; 
        [QE, Qe] = eig(Q);

% Number of particles
D = 100;

% Prior - uniform over position, heading and wind ranges
r_pos = 5;
r_head = pi/6;
r_vel = 1;
r_steer = pi/8;

X = zeros(5,D);
X(1,:) = x0(1) + r_pos*rand(1,D) - r_pos/2;
X(2,:) = x0(2) + r_pos*rand(1,D) - r_pos/2;
X(3,:) = x0(3) + r_head*rand(1,D) - r_head/2;
X(4,:) = x0(4) + r_vel*rand(1,D) - r_vel/2;
X(5,:) = x0(5) + r_steer*rand(1,D) - r_steer/2;

% Particle filter initialization
X0 = X;
Xp = X;
w = zeros(1,D);

% Simulation Initializations
n = length(x0);
x = zeros(n,length(T));
x(:,1) = x0;
m = length(Q(:,1))*5;
y = zeros(5,length(T));
Qm = Q*eye(5);

figure(1);clf; hold on;
plot(x(1,1),x(2,1), 'ro--')
for dd=1:D
    drawrobot(X(1,dd),X(2,dd),X(3,dd),L,X(5,dd),1,1)
    %plot(X(1,dd),X(2,dd),'b.')
end
axis equal
grid on
%axis([-5 30 -30 30]);
title('PF Localization - Plot 2')

%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update state
    x(:,t) = [x(1,t-1)+x(4,t-1)*cos(x(3,t-1))*dt;
              x(2,t-1)+x(4,t-1)*sin(x(3,t-1))*dt;
              x(3,t-1)+x(4,t-1)*tan(x(5,t-1))*dt/L;
              x(4,t-1);
              x(5,t-1)] + e;
    
    % Select a motion disturbance
    d = QE*sqrt(Qe)*randn(5,1);
    % Determine measurement
        y(:,t) = [x(1,t);
                  x(2,t);
                  x(4,t)*cos(x(3,t));
                  x(4,t)*sin(x(3,t));
                  x(5,t)] + d;

    %% Particle filter estimation
    for dd=1:D
        e = RE*sqrt(Re)*randn(n,1);
        Xp(:,dd) = [X(1,dd)+X(4,dd)*cos(X(3,dd))*dt;
                    X(2,dd)+X(4,dd)*sin(X(3,dd))*dt;
                    X(3,dd)+X(4,dd)*tan(X(5,dd))*dt/L;
                    X(4,dd);
                    X(5,dd)] + e;
        hXp = [Xp(1,dd);
               Xp(2,dd);
               Xp(4,dd)*cos(Xp(3,dd));
               Xp(4,dd)*sin(Xp(3,dd));
               Xp(5,dd)] + d;
        w(dd) = mvnpdf(y(:,t),hXp,Qm);
    end
    W = cumsum(w);
    for dd=1:D
        seed = max(W)*rand(1);
        X(:,dd) = Xp(:,find(W>seed,1));
    end
    muParticle = mean(X');
    SParticle = var(X');

    muParticle_S(:,t) = muParticle;
    
     %% Plot results
     if (t == 10 || t==30 || t==50 || t==70 || t==90 || t==100)
        plot(x(1,1:t),x(2,1:t), ':')
        for dd=1:D
            drawrobot(X(1,dd),X(2,dd),X(3,dd),L,X(5,dd),1,1)
            %plot(X(1,dd),X(2,dd),'b.')
        end
     end
     drawnow;
end
legend('robot state','particles')
xlabel('x (m)')
ylabel('y (m)')
hold off

figure(2)
grid on
axis equal
title('PF Localization - Plot 1')
plot(x(1,:), x(2,:), 'o', y(1,:), y(2,:), 'x')
legend('True xy', 'Measured xy')
xlabel('x (m)')
ylabel('y (m)')
hold off

figure(3)
title('PF Localization - Plot 1')

subplot(2,3,1)
plot(T, x(1,:), T, muParticle_S(1,:),'--')
legend('True', 'Particle Mean')
xlabel('t (s)')
ylabel('x (m)')

subplot(2,3,2)
plot(T, x(2,:), T, muParticle_S(2,:),'--')
legend('True', 'Particle Mean')
xlabel('t (s)')
ylabel('y (m)')

subplot(2,3,3)
plot(T, x(3,:), T, muParticle_S(3,:),'--')
legend('True', 'Particle Mean')
xlabel('t (s)')
ylabel('Heading (rad)')

subplot(2,3,4)
plot(T, x(4,:), T, muParticle_S(4,:),'--')
legend('True', 'Particle Mean')
xlabel('t (s)')
ylabel('vel (m/s)')

subplot(2,3,5)
plot(T, x(5,:), T, muParticle_S(5,:),'--')
legend('True', 'Particle Mean')
xlabel('t (s)')
ylabel('steering angle (rad)')