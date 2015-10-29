function  Scan(serPort)
    global Map;
    global cur_locat;
    global total_x_dist;
    global total_y_dist;
    global status_vacant;
    global trace_flag;
    init();
    update(serPort);
    while(1)
        trace_flag = 0;
        % Whether the Map is searched completely
        display(cur_locat);
        if ~ismember(0,Map)
            display('Search complete!');
            break;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        current = cur_locat;
        goal = next(current);
        while ~valid(goal)
            current = goal;
            goal = next(current);
        end
        distance = align(serPort,[total_x_dist,total_y_dist],transf(goal,1));
        while norm([total_x_dist,total_y_dist]-transf(cur_locat,1)) < distance
            [BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
            sensor = BumpFront||BumpLeft||BumpRight||WallSensorReadRoomba (serPort);
            SetFwdVelRadiusRoomba(serPort, 0.1, inf);
            update(serPort);
            if sensor
                trace_flag = 1;
                break;               
            end
        end
        SetFwdVelRadiusRoomba(serPort, 0, inf);         % Stop!
        if trace_flag
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %   TRACE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            current = cur_locat;
            goal = next(current);
            while ~valid(goal)
                current = goal;
                goal = next(current);
            end
            distance = align(serPort,[total_x_dist,total_y_dist],transf(goal,1)); 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %   BUG 2(serPort,distance)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        cur_locat = goal;
        Map(cur_locat(1),cur_locat(2)) = status_vacant;
        Map_plot();
        pause(0.2);
    end
end
function init()
    global Map;
    global cur_locat;
    global start_locat;
    global Map_size;
    global total_angle;
    global total_x_dist;
    global total_y_dist;
    global para;            % Scale
    global status_unexplored;   % = 0
    global status_obstacle;     % = 0.5
    global status_vacant;       % = 1
    global turn_speed;
    global trace_flag;          % whether BUG algorithm need to be implemented

    para = 0.3;
    status_unexplored = 0;
    status_obstacle = 0.5;
    status_vacant = 1;
    turn_speed = 0.2;
    trace_flag = 0;
    
    total_x_dist = 0;
    total_y_dist = 0;
    total_angle = 0;
    
    Map_size = [11,11];     % The last row/column is not used in colormap
    start_locat = [5,5];
    cur_locat = start_locat;
    Map = zeros(Map_size(1),Map_size(2));
    Map(end,:)=status_obstacle; Map(:,end) = status_obstacle;
    Map(start_locat(1),start_locat(2)) = status_vacant;
     Map(2:4,2:4)=status_obstacle; 
     Map(6:10,6:10)=status_obstacle;
    Map_plot();
end

function Map_plot()
    global Map;
    figure(1);
    color_map = [1 1 1; 0 0 0.6; 0.8 0.8 0];
    colormap(color_map);
    pcolor(Map);
    figure(2)
    imagesc(Map);
end

function goal = next(current)
    global start_locat;
    if current == start_locat;
        goal = [current(1)+1,current(2)];
    else
        % Above the start point: Move left
        if current(1) > start_locat(1) && current(1)-start_locat(1) > abs(current(2)-start_locat(2))
            goal = [current(1),current(2)-1];
        % Left to the start point: Move downwards
        elseif current(2) < start_locat(2) && start_locat(2)-current(2) > start_locat(1)-current(1)
            goal = [current(1)-1,current(2)];
        % Below the start point: Move right
        elseif current(1) < start_locat(1) && start_locat(1)-current(1) > current(2)-start_locat(2)
            goal = [current(1),current(2)+1];
        % Right to the start point: Move upwards           
        elseif current(2) > start_locat(2) && current(2)-start_locat(2) >= abs(current(1)-start_locat(1));
            goal = [current(1)+1,current(2)];
        end
    end
end

function status = valid(locat)
    global Map_size;
    global Map;
    status = 1;
    boundary = double(locat<Map_size) .* double(locat>[0,0]);
    if boundary(1)==0 || boundary(2)==0
        status = 0;
    elseif Map(locat(1),locat(2))==0.5
        status = 0;
    end
end

function dist = align(serPort,current,goal)
    global total_angle;
    global turn_speed;
    if goal(2)>=current(2)
        theta = acos(dot(goal-current,[1,0])/norm(goal-current))-total_angle;
        turnAngle (serPort, turn_speed, 180*theta/pi);
%        display(180*theta/pi)
%        total_angle = acos(dot(goal-current,[1,0])/norm(goal-current));
    else
        theta = acos(dot(goal-current,[1,0])/norm(goal-current))+total_angle;
        turnAngle (serPort, turn_speed, -180*theta/pi);
        %display(180*theta/pi)
        %total_angle = -acos(dot(goal-current,[1,0])/norm(goal-current));
    end
    update(serPort);
    dist = norm(goal-current);
end

% Transfer coordinates between physical world and matrix 
function loca2 = transf(loca1,flag)
    global start_locat;
    global para;
    % From discrete to continuous
    if flag==1
       loca2 = (loca1-start_locat)*para;
       loca2 = [loca2(1),-loca2(2)];
    % From continuous to discrete
    else
       loca2 = round(local/para);
       loca2 = [loca2(1),-loca2(2)];
       loca2 = loca2 + start_locat;
    end   
end

function update(serPort)
    global total_x_dist;
    global total_y_dist;
    global total_angle;
    dist = DistanceSensorRoomba(serPort);
    angle = AngleSensorRoomba(serPort);
    total_angle = total_angle + angle;
 
    % keep total_angle between -2*pi and 2*pi
    if total_angle >= 2*pi
        total_angle = total_angle - 2*pi;
    elseif total_angle < -2*pi
        total_angle = total_angle + 2*pi;
    end

    x = dist * cos (total_angle);
    y = dist * sin (total_angle);
    total_x_dist = total_x_dist + x;
    total_y_dist = total_y_dist + y;
end

