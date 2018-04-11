function [X] = pf_localization_exam(X,L, t, dt, n, D, R, Qg, mg, yg,Qs, ms, ys, p_miss, yw)
% Particle filter localization
% inputs:
%   - X: current pose of all particles (from last time step)
%   - t: current time
%   - dt: timestep
%   - n: number of states
%   - D: number of particles
%   - R: motion model noise
%   - Q(g,s,w): measurement model noise (for gps, steering, wheel encoder)
%   - m(g,s,w): number of measurements (for gps, steering, wheel encoder)
%   - y(g,s,w): lastest measurement (for gps, steering, wheel encoder)
%
%   - meas: REMOVED  - measurement type (1-range, 2-bearing, 3-both)
%   - u: REMOVED - history of all inputs (only need latest) - no. of inputs x no of
%        time steps
%   - d: - REMOVED - motion disturbance (NOT NEEDED)
%   - Xp: predicted pose of all particles (NOT NEEDED)
%
% -------
% Note: motion model hard coded into this example
% -------
%
% Outputs:
%   - X: state of all particles - no. of states x M
% -------------------------------------------------------------------------

    [RE, Re] = eig(R);
    w = zeros(1, D);
    Xp = zeros(size(X));
    
% iterate through all M samples
    % Prediction step 
    for dd = 1:D
        e = RE * sqrt(Re) * randn(n, 1);
        % apply motion model
        Xp(:,dd) = X(:,dd) + [X(4,dd)*cos(X(3,dd))*dt;
                              X(4,dd)*sin(X(3,dd))*dt;
                              X(4,dd)*tan(X(5,dd))*dt/L;
                              0;
                              0] + e;
           
        % Calculate the measurements
        
           % Gps
           [QEg, Qeg] = eig(Qg);
           dg = QEg * sqrt(Qeg) * randn(mg, 1);
           ygXp = [Xp(1,dd);
                   Xp(2,dd);
                   (Xp(4,dd)) /sqrt(1+(tan(Xp(3,dd)))^2);
                   tan(Xp(3,dd))*(Xp(4,dd)) /sqrt(1+(tan(Xp(3,dd)))^2)] + dg;
           % Steering
           ds = sqrt(Qs)*rand(ms,1);
           ysXp = Xp(5,dd) + ds;
           
           % Wheel Encoder
           Qw = 0.01^2;
           dw = sqrt(Qw) * rand(1,1);
           ywXp = Xp(4,dd)/2.1 - dw;
           
        % Build Combined Measurements
        hXp = [ygXp;ysXp;ywXp];
        Q = zeros(mg + ms + 1, mg + ms + 1);
        Q(1:4,1:4) = Qg;
        Q(5,5) = Qs;
        Q(6,6) = Qw;
        y = [yg;ys;yw];
        % Calculate weights of each particle based on measurement and
        % measurement equation. Set min so it doesn't get to close to zero
        w(dd) = max(1e-8, mvnpdf(y, hXp, Q));
    end
    
    % Measurement Update importance re-sample
    W = cumsum(w);
    for dd = 1:D
        seed = max(W) * rand(1);
        X(:, dd) = Xp(:, find(W > seed, 1));
    end
end

