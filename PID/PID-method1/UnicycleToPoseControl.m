function [ u ] = UnicycleToPoseControl( xTrue,xGoal, Kp, Ka, Kbeta )
%Computes a control to reach a pose for unicycle
%   xTrue is the robot current pose : [ x y theta ]'
%   xGoal is the goal point
%   u is the control : [v omega]'
% 21 12 22
% Kp=21;
% Ka=7;
% Kbeta=12;

alpha_max=0.7;

p=sqrt((xGoal(1)-xTrue(1))^2+(xGoal(2)-xTrue(2))^2);
alpha=AngleWrap(atan2((xGoal(2)-xTrue(2)),(xGoal(1)-xTrue(1)))-xTrue(3));

if p>0.05
    v=Kp*p;
    omega=Ka*alpha;
    if abs(alpha)>alpha_max
        v=0;
    end
else
    
    if abs(alpha)>alpha_max
        v=0;
    else
        v=Kp*p;
    end
    
    
    beta=(xGoal(3)-xTrue(3));
    omega=Kbeta*beta;
end
    
u=[v omega]';
end

