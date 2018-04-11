function drawrobot(x,y,h,L,s,scale,fig)
% function drawcar(x,y,h,L,s,scale,fig)
% This function plots a bicycle at position x,y heading h, wheel span L, 
% steering angle s, and size scale on figure number fig.
% The default x,y = (0,0), h = 0 points to the right, and scale=1 plots a
% car with a body of length L.

% Make a straight line of length L for the body
t=0:0.01:L;
len = length(t);
bx = t;
by = zeros(1,len);

% Wheel locations on body
wh1 = 1;
wh2 = len;

% Draw the wheels
wwidth= 0.03;
wheight = 0.08;
w1 = [-wheight/2 0;-wheight/2 -wwidth/2; wheight/2 -wwidth/2; wheight/2 wwidth/2;-wheight/2 wwidth/2;-wheight/2 0; 0 0];
R = [cos(s) -sin(s); sin(s) cos(s)]; % define rotation matrix for front wheel
w2 = (R*w1')';

% Car outline
car = [bx(1:wh1)' by(1:wh1)';
    bx(wh1)+w1(:,1) by(wh1)+w1(:,2);
    bx(wh1:wh2)' by(wh1:wh2)';
    bx(wh2)-w2(:,1) by(wh2)-w2(:,2);
    bx(wh2:end)' by(wh2:end)'];

%Size scaling
car = scale*car;

% Rotation matrix
R = [cos(h) -sin(h); sin(h) cos(h)];
car = (R*car')';

% Centre
car(:,1) = car(:,1)+x;
car(:,2) = car(:,2)+y;

% Plot
figure(fig);
plot(car(:,1), car(:,2), 'b');
end