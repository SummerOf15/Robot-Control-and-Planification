function [ u ] = BicycleToPathControl( xTrue, Path )
%Computes a control to follow a path for bicycle
%   xTrue is the robot current pose : [ x y theta ]'
%   Path is set of points defining the path : [ x1 x2 ... ;
%                                               y1 y2 ...]
%   u is the control : [v phi]'

% TODO
kp = 10;
ka = 7;
kb = -5;

global last_destination;
angle = [0.3, 0.3, 0.8,0.45,0.45];
for i = 1:length(Path)-1
	if sqrt((xTrue(1,1) - Path(1,i))^2 + (xTrue(2,1) - Path(2,i))^2) <angle(i)
        last_destination = i + 1;
        ka = 15;
	end
end
if last_destination > 6
    last_destination = 6;
end
xGoal = [Path(1,last_destination);Path(2,last_destination);Path(3,last_destination)];
xPre = [Path(1,last_destination-1);Path(2,last_destination-1);Path(3,last_destination-1)];

p = sqrt((xTrue(1,1) - xGoal(1,1))^2 + (xTrue(2,1) - xGoal(2,1))^2);
alpha = atan2((xGoal(2,1) - xTrue(2,1)),(xGoal(1,1) - xTrue(1,1))) - xTrue(3,1);
alpha = AngleWrap(alpha);
beta = atan2((xGoal(2,1) - xPre(2,1)),(xGoal(1,1) - xPre(1,1))) - xTrue(3,1);
beta = AngleWrap(beta);

u(2,1) = ka * alpha + kb * beta;
u(1,1) = kp * p;
end

