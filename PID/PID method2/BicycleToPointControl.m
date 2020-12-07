function [ u ] = BicycleToPointControl( xTrue,xGoal )
%Computes a control to reach a pose for bicycle
%   xTrue is the robot current pose : [ x y theta ]'
%   xGoal is the goal point
%   u is the control : [v phi]'


% TODO
kp = 20;
ka = 5;

p = sqrt((xTrue(1,1) - xGoal(1,1))^2 + (xTrue(2,1) - xGoal(2,1))^2);
alpha = atan2((xGoal(2,1) - xTrue(2,1)),(xGoal(1,1) - xTrue(1,1))) - xTrue(3,1);
alpha = AngleWrap(alpha);
   
%phi = min(ka * alpha, 1.2);
%v = min(kp * p,1);
u(2,1) = ka * alpha;
u(1,1) = kp * p;
end

