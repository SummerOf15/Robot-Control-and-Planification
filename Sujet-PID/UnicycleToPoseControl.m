function [ u ] = UnicycleToPoseControl( xTrue,xGoal )
%Computes a control to reach a pose for unicycle
%   xTrue is the robot current pose : [ x y theta ]'
%   xGoal is the goal point
%   u is the control : [v omega]'

% TODO
%constant kp et ka
kp = 15;
ka = 10;
kb = 25;

p = sqrt((xTrue(1,1) - xGoal(1,1))^2 + (xTrue(2,1) - xGoal(2,1))^2);
alpha = atan2((xGoal(2,1) - xTrue(2,1)),(xGoal(1,1) - xTrue(1,1))) - xTrue(3,1);
alpha = AngleWrap(alpha);
beta = xGoal(3,1) - xTrue(3,1);
beta = AngleWrap(beta);
if p > 0.05    
    omega = ka * alpha;
    if abs(alpha) > pi/4
        v = 0;
    else
        v = kp * p;
    end
else
    omega = kb * beta;
    if abs(alpha) > pi/4
        v = 0;
    else
        v = kp * p;
    end
end

u = [v, omega];
end

