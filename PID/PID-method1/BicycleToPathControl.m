function [ u ] = BicycleToPathControl( xTrue, Path, Kp, Ka )
%Computes a control to follow a path for bicycle
%   xTrue is the robot current pose : [ x y theta ]'
%   Path is set of points defining the path : [ x1 x2 ... ;
%                                               y1 y2 ...]
%   u is the control : [v phi]'

% TODO
% Path = [.1,0,0; 4,0,0; 4,4,0; 3.75,2.5,0; 3.5,1,0; 0,4,0;0,1,-1.57]';
l=size(Path);
path_vector=Path(1:2,2:end)-Path(1:2,1:end-1);
theta_vector=[cos(xTrue(3)),sin(xTrue(3))];
cos_theta=[];
goalWaypointId=1;
xBigGoal=0;
% distance=[0.3,0.3,0.5,0.2, 0.4,0.5,0];
distance=[0.3,0.3,1.15,0.4,0.5,0.4];
for i=1:l(2)-1
    % change the bigGoal point
    if sqrt((xTrue(1)-Path(1,i))^2+(xTrue(2)-Path(2,i))^2)<distance(i)
        xBigGoal=Path(:,i+1);
        goalWaypointId=i+1;
        cos_theta=[];
        break;
    else
        cos_theta(i)=(theta_vector*path_vector(:,i))/norm(path_vector(:,i));
    end
end
if length(cos_theta)==l(2)-1
    ind=find(cos_theta>max(cos_theta)-0.01);
    if length(ind)>1
        diff=(Path(1:2,ind)-xTrue(1:2));
        for i =1:length(ind)
            dis(i)=norm(diff(1:2,i));
        end
        [d,i]=min(dis);
        xBigGoal=Path(:,ind(i));
    else
        xBigGoal=Path(:,ind+1);
        goalWaypointId=ind+1;
    end
end
% xGoal=xBigGoal;

if xBigGoal==0
    u(1)=3;
    u(2)=0;
else
    
    alpha = AngleWrap(atan2(xBigGoal(2)-xTrue(2),xBigGoal(1)-xTrue(1)) - xTrue(3));
    p=sqrt((xTrue(1)-xBigGoal(1))^2+(xTrue(2)-xBigGoal(2))^2);

    if abs(alpha)>pi/25
        u(1)=0.9;
        u(2)=Ka*alpha;
    else
        u(1)=Kp*p;
        u(2)=Ka*alpha;
    end
    
end

% rho=0.3;
% error=Path(:,goalWaypointId)-xTrue;
% waypointDist=norm(error(1:2));
% if waypointDist<rho
%     % if reached, set xGoal to waypoint and goalWaipoint to next waypoint
%     xGoal=Path(:,goalWaypointId);
%     goalWaypointId=goalWaypointId+1;
%     goalWaypointId = min(goalWaypointId,size(Path,2)); % avoid taking point after path end?
% else
%     %if not reached, move goal along line to next waypoin while dist < rho?    
%     delta=Path(:,goalWaypointId) - Path(:,goalWaypointId-1);
%     delta=delta/norm(delta); % unit direction vector to next waypoint
%     error=xGoal-xTrue;
%     goalDist=norm(error(1:2)); % rho
%     while goalDist<rho    
%         xGoal=xGoal+0.01*delta;
%         error=xGoal-xTrue;
%         goalDist=norm(error(1:2)); % rho
%     end
% end
% xSmallGoal=xTrue(1:2)+0.5*(xBigGoal(1:2)-xTrue(1:2))/norm(xBigGoal(1:2)-xTrue(1:2));
% error=xGoal-xTrue;
% goalDist=norm(error(1:2));
% AngleToGoal = AngleWrap(atan2(error(2),error(1))-xTrue(3));
% u(1) = Kp * goalDist;
% u(2) = Ka * AngleToGoal;
end

