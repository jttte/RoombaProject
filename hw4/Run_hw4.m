% [mark_x,mark_y] is coordinates of the points on the shortest path
function Run_hw4(serPort,mark_x,mark_y)
    global total_x_dist;
    global total_y_dist;
    global move_speed;
    init();
    % mark_x = [0,1,1,1];
    % mark_y = [0,-1,-2,1];
    total_x_dist = mark_x(1);
    total_y_dist = mark_y(1);
    
    for i=2:length(mark_x)
        dist = align(serPort,[total_x_dist,total_y_dist],[mark_x(i),mark_y(i)]);
        travelDist(serPort,move_speed,dist);
        update(serPort);
    end
end


function dist = align(serPort,current,goal)
    global total_angle;
    global turn_speed;
    angle  = acos(dot(goal-current,[1,0])/norm(goal-current));
    if goal(2)>current(2)
        angle = angle - total_angle;
        % Real robot
        angle = 180*angle/pi;
        turnAngle(serPort, turn_speed ,angle);
    else
        angle = (-angle + 2*pi)-total_angle;
        % Real robot
        angle = 180*angle/pi;
        turnAngle(serPort, turn_speed ,angle);
    end
    update(serPort);  
    dist = norm(goal-current);
end

function update(serPort)
    global total_x_dist;
    global total_y_dist;
    global total_angle;
    dist = DistanceSensorRoomba(serPort);
    angle = AngleSensorRoomba(serPort);
    total_angle = total_angle + angle;
 
    % keep total_angle within [0,2*pi)
    if total_angle >= 2*pi
        total_angle = total_angle - 2*pi;
    elseif total_angle < 0
        total_angle = total_angle + 2*pi;
    end

    x = dist * cos (total_angle);
    y = dist * sin (total_angle);
    total_x_dist = total_x_dist + x;
    total_y_dist = total_y_dist + y;
end

function init()
    global total_angle;
    global total_x_dist;
    global total_y_dist;
    global turn_speed;
    global move_speed;

    move_speed = 0.2;
    turn_speed = 0.2;    
    total_x_dist = 0;
    total_y_dist = 0;
    total_angle = pi/2;
end