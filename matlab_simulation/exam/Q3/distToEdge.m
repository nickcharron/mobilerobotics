function [d,pt, condition] = distToEdge(v,edge)
%% Computes the shortest distance to a line segment, returns distance and
% closest point, and if one of the end conditions was found.
% Inputs: 
%   - v: vehicle position
%   - edge: 1 x 4 matrix containing the ends of the edge i.e.:[Xs,Ys,Xe,Ye]
% Output:
%   - d: distance to edge
%   - pt: 1 x 2 matrix containing the closest point to v. i.e.: [Xp Yp]
%   - condition: 0 - point within line
%                1 - point is before line segment start
%                2 - point is after line segment end
%% main

S = edge(3:4) - edge(1:2);
S1 = v - edge(1:2);
m1 = S1*S';
if (m1 <= 0 )
    d =  norm(v-edge(1:2));
    pt = edge(1:2);
    condition = 1;
else
    m2 = S*S';
    if ( m2 <= m1 )
        d = norm(v-edge(3:4));
        pt = edge(3:4);
        condition = 2;
    else
        b = m1 / m2;
        pt = edge(1:2) + b*S;
        d = norm(v-pt);
        condition = 0;
    end
end


