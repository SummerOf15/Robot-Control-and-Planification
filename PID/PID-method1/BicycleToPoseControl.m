function [ u ] = BicycleToPoseControl( xTrue,xGoal,Kp,Ka,Kb )
%Computes a control to reach a pose for bicycle
%   xTrue is the robot current pose : [ x y theta ]'
%   xGoal is the goal point
%   u is the control : [v phi]'


p=sqrt((xGoal(1)-xTrue(1))^2+(xGoal(2)-xTrue(2))^2);
alpha=AngleWrap(atan2((xGoal(2)-xTrue(2)),(xGoal(1)-xTrue(1)))-xTrue(3));
beta=AngleWrap(xGoal(3)-atan2((xGoal(2)-xTrue(2)),(xGoal(1)-xTrue(1))));
u(1)=Kp*p;
u(2)=Ka*alpha+Kb*beta;

end
