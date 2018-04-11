function carrotPt1 = calculateCarrot(closestPt1, edges1, r1)
%% Computes the carrot point at distance r away from closestPt along edge 
% defined by 'edges' 
% Inputs: 
%   - closestPt
%   - edge: 1 x 4 matrix containing the ends of the edge i.e.:[Xs,Ys,Xe,Ye]
%   - r: look ahead length for carrot
% Output:
%   - carrotPt: 1 x 2 matrix point of carrot. i.e.: [Xcarrot, Ycarrot]
%% main

normVec1 = (edges1(3:4) - closestPt1)/norm(edges1(3:4) - closestPt1);
carrotPt1 = closestPt1 + normVec1 * r1;
end




