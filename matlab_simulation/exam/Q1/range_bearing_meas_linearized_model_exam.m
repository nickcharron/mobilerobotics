function Ht = range_bearing_meas_linearized_model_exam(mu,i)
    % linearized measurement model of range and bearing
    % can be found P.22 of Mapping II slides
    % Inputs:
    %   - mu: pose at current timestep including pose of robot & map
    %   elements
    %   - i: map element we are updating. i.e.: 1 for map element 1
    
    dx = mu(3+2*(i-1)+1)-mu(1); % M^i_x - xr
    dy = mu(3+2*i)-mu(2);       % M^i_y - yr
    rp = sqrt((dx)^2+(dy)^2);
    r = sqrt((mu(1))^2+(mu(2))^2);
    
    N=length(mu); % number of states
    Vstates = 3; % Number of vehicle states - UPDATE THIS
    Mstates = 2; % Number of map states - UPDATE THIS
    Fi = zeros(Vstates+Mstates,N);    
    Fi(1:Vstates,1:Vstates) = eye(Vstates);
    Fi(Vstates+1:Vstates+Mstates,Vstates+Mstates*(i-1)+1:Vstates+Mstates*i) = eye(Mstates);
    % Multiplying Ht by Fi maps Ht into the correct space
    if i == 1
        Ht = [ mu(1)/r,     mu(2)/r,     0,   0,   0;
               0            0,           0,   0,   0 ] * Fi;
    else
        Ht = [ 0,     0,     0,   0,   0;
               dy/rp^2,   -dx/rp^2,  -1,  -dy/rp^2, dx/rp^2 ] * Fi;
    end
    %Ht = [ -dx/rp,     -dy/rp,     0,   dx/rp,   dy/rp;
    %        dy/rp^2,   -dx/rp^2,  -1,  -dy/rp^2, dx/rp^2 ] * Fi;
end
