function  Scan(serPort)
    global Map;
    global cur_locat;
    global total_x_dist;
    global total_y_dist;
    global status_vacant;
    global trace_flag;
    global move_speed;
    init();
    init_map();
%     update(serPort);
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
        display(goal);
        distance = align(serPort,[total_x_dist,total_y_dist],transf(goal,1));
        while norm([total_x_dist,total_y_dist]-transf(cur_locat,1)) < distance
            SetFwdVelRadiusRoomba(serPort, move_speed, Inf);
            [BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
            sensor = BumpFront||BumpLeft||BumpRight||WallSensorReadRoomba (serPort);
            
            update(serPort);
            if sensor
                trace_flag = 1;
                break;               
            end
        end
%         SetFwdVelRadiusRoomba(serPort, 0, Inf);         % Stop!
        if trace_flag
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            trace_boundary(serPort);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            current = cur_locat;
            goal = next(current);
            while ~valid(goal)
                current = goal;
                goal = next(current);
            end
            display(goal);
            distance = align(serPort,[total_x_dist,total_y_dist],transf(goal,1)); 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Bug2(serPort, distance);
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
    global Map_wall;
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
    global move_speed;
    para = 0.3;
    status_unexplored = 0;
    status_obstacle = 0.5;
    status_vacant = 1;
    turn_speed = 0.2;
    move_speed = 0.1;
    trace_flag = 0;
    
    total_x_dist = 0;
    total_y_dist = 0;
    total_angle = 0;
    
    Map_size = [31,31];     % The last row/column is not used in colormap
    start_locat = [15,15];
    cur_locat = start_locat;
    Map = zeros(Map_size(1),Map_size(2));
    Map_wall = zeros(Map_size(1),Map_size(2));
    Map(end,:)=status_obstacle; Map(:,end) = status_obstacle;
    Map(start_locat(1),start_locat(2)) = status_vacant;

end

function init_map ()
    global figHandle;
    figHandle = figure; 
end

function Map_plot()
    global figHandle;
    global Map;
%     display(Map);
    figure (figHandle);
    color_map = [1 1 1; 0 0 0.6; 0.8 0.8 0];
    colormap(color_map);
    pcolor(Map);
%     figure(2)
%     imagesc(Map);
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
    global status_obstacle
    status = 1;
    boundary = double(locat<Map_size) .* double(locat>[0,0]);
    if boundary(1)==0 || boundary(2)==0
        status = 0;
    elseif Map(locat(1),locat(2))== status_obstacle
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
       loca2 = round(loca1/para);
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

%%%%%%%%%%%%      Trace Part    %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function trace_boundary(serPort)

    global Map;
    global Map_size;
    global angle_right;
    global angle_front;
    global angle_left;
    global status_obstacle;
    global total_x_dist;
    global total_y_dist;
    global move_speed;
    global loop_pause_time;
    
    % remember the point it hit the obsatcle
    x_start = total_x_dist;
    y_start = total_y_dist;
    trace_init();
    % reset tmp_boundary_map
    tmp_boundary_map = zeros(Map_size);
    % Enter main loop
    while true
        display('=================loop===================')

        %check if the robot is back to start position
        if CheckBack(x_start,y_start) == true
            display ('back to starting point - Stop!')
            break;
        end
        
        [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
        BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        bumped= BumpRight || BumpLeft || BumpFront;
        wallSensor = WallSensorReadRoomba (serPort);
        
        if bumped            
            % back off for a little bit
            travelDist(serPort, 0.025, -0.01);
            update_trace(serPort);

            if BumpRight
              display('bump right');
              turnAngle (serPort, 0.2, angle_right);
              pause(0.05)
            elseif BumpLeft
              display('bump left');
              turnAngle (serPort, 0.2, angle_left);
              pause(0.05)
            elseif BumpFront
              display('bump front');
              turnAngle (serPort, 0.2, angle_front);
              pause(0.05)
            end
            tmp_boundary_map = update_current_map(tmp_boundary_map);
        elseif ~wallSensor  %need to turn back to obstacle
            display ('differntial turn');
            SetFwdVelRadiusRoomba (serPort, move_speed, -0.2);
            
        else %move forward              
            SetFwdVelRadiusRoomba(serPort, move_speed, Inf);
            pause(0.1) 
        end
        update_trace(serPort);
        tmp_boundary_map = update_current_map(tmp_boundary_map);
%         annotate_Map(status_obstacle);
%         display(tmp_boundary_map);
        annotate_Map(status_obstacle);
        Map_plot();

        % Briefly pause to avoid continuous loop iteration
        pause(loop_pause_time)
    end % end of tracing boundary loop
    
    % Stop robot motion
    SetFwdVelAngVelCreate(serPort, 0, Inf);
    
    annotate_Map(status_obstacle);
%     tmp_boundary_map = fill_blocks(tmp_boundary_map);
%     Map = Map.*(tmp_boundary_map==0)+tmp_boundary_map;
    fill_blocks();
    Map_plot();
end

function trace_init()

    global angle_left;
    global angle_right;
    global angle_front;
    global total_dist;    
    total_dist = 0;
    angle_left  = 60;
    angle_right = 15;
    angle_front = 45;

end

function update_trace(serPort)
    global total_x_dist;
    global total_y_dist;
    global total_angle;
    global total_dist;
    dist = DistanceSensorRoomba(serPort);
    angle = AngleSensorRoomba(serPort);
    total_angle = total_angle + angle;
    total_dist = total_dist + dist;
    
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

function isDone = CheckBack(x,y)
    global total_x_dist;
    global total_y_dist;
    global total_dist;
    radius = norm([total_x_dist-x,total_y_dist-y]);
    if (total_dist > 1 && radius < 0.3)
        isDone = true;
    else
        isDone = false;
    end
end

function fill_blocks()
%     global start_locat;
%     global status_obstacle;
%  
%         
%     mapfill = imfill(map,'hole');
%     if map(start_locat(1),start_locat(2)) ~= status_obstacle
%         map = mapfill;
%     else
%         mapfill(find(map==status_obstacle)) = 1;
%         mapfill(find(map==0)) = status_obstacle;
%         mapfill(find(map==1)) = 0;
%         map = mapfill+map;
%     end   

    global tmp_boundary_map;
    global Map;
    global Map_wall;
    global status_obstacle;
    global status_vacant;
    global start_locat;
    
    test_map = (Map == status_obstacle);
    test_filled_map = imfill(test_map,'hole');
    if test_filled_map (start_locat(1), start_locat(2)) == 1
        display('we just traced the wall!');
        Map_wall = - tmp_boundary_map;
    end
    
    boundary_map = (Map + Map_wall) == status_obstacle; 
    % only obstacle points stay in boundary_map
    filled_map = imfill(boundary_map,'hole');
    
    Map = (filled_map - Map_wall)*status_obstacle + (Map == status_vacant) * status_vacant;
end

function map = update_current_map(map)

    global total_x_dist;
    global total_y_dist;
    global status_obstacle;

    block_xy = transf([total_x_dist, total_y_dist], 0);
    x_idx = block_xy(1);
    y_idx = block_xy(2);
    display('update');
    display(block_xy);
        
    map(x_idx, y_idx) = status_obstacle;   
% 	Map_plot();
end

function annotate_Map(status)
    global Map;
    global total_x_dist;
    global total_y_dist;
%     global status_obstacle;
    
    block_xy = transf([total_x_dist, total_y_dist], 0);
    x_idx = block_xy(1);
    y_idx = block_xy(2);
    
    Map(x_idx, y_idx) = status;   
	Map_plot();
    
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% bug2 algorithm utilities
% bug2 does not update map, assuming robot will arrive goal successfully
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Bug2(serPort, goal)
    % constants
    global bumped_obstacle;
    global loop_pause_time;
    global bumped_dist_x; % coordinate x when first bumping into obstacle qi
    global bumped_dist_y; % coordinate y when first bumping into obstacle qi
    global reached_goal;
    global no_solution;
    global move_speed;
    global travel_dist_after_bump;
    global status;
    
    global bug2_x;
    global bug2_y;
    
    global status_explored;
    global status_boundary;
    global status_wall;

    init_bug2(goal);
    
    % reset distance and angle
%     DistanceSensorRoomba(serPort);
%     AngleSensorRoomba(serPort);
    
    % Start moving straight forward
    % main loop
    while true
        display('=================loop===================')
        display(goal);

        if reached_goal || checkLocation()
            display('reached goal - stop!');
            break;
        end
        
        if no_solution
            display('no solution found - stop!');
            break;
        end
        
        [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
        BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        if(isnan(BumpRight) || isnan(BumpLeft) || isnan(BumpFront))
            display('bad connection (getting broken signal)');
            display('    we recommand you to abort the program');
        else
            bumped_obstacle = BumpRight || BumpLeft || BumpFront;
        end
         
        
        if bumped_obstacle
            display('bumped in bug 2 algorithm')
%             update_map(status_boundary);
            %%% TODO !!! need to keep track of blocks !!!
            %%% annotate to status_wall at the end of the tracing
           
            % back off for a little bit
            travelDist (serPort, 0.025, -0.01);
            update_bug2 (serPort);
            % reset travel dist (help determine 'back to mline')
            travel_dist_after_bump = 0.0;
            
            if(status == 0 || status == 2)
                bumped_dist_x = bug2_x;
                bumped_dist_y = bug2_y;
                display(bumped_dist_x);
            end
            
            % follow boundary
            bug2_trace_boundary(serPort);
            
        else % did not bump into any obstacle, keep moving forward
            SetFwdVelRadiusRoomba(serPort, move_speed, Inf);
%             update_map(status_explored);
            display ('moving forward');          
        end

        update_bug2(serPort);
        pause(loop_pause_time);
    end % end of main loop

    update_bug2(serPort);
    
    % Stop robot motion
    SetFwdVelAngVelCreate(serPort, 0, Inf);
end

% initialize constants
function init_bug2(dist)
    global angle_left;
    global angle_right;
    global angle_front;
    global dist_to_goal;
    global reached_goal;
    global no_solution;
    global travel_dist_after_bump;
    global status;
    global bug2_angle;
    global bug2_dist;
    global bug2_position_offset;
    global bug2_x;
    global bug2_y;
    global total_x_dist;
    global total_y_dist;
    
    dist_to_goal   = dist; % distance to the target
    bug2_dist     = 0;
    bug2_angle    = 0;
    bug2_position_offset = [total_x_dist, total_y_dist];
    bug2_x = 0;
    bug2_y = 0;
    
    angle_left  = 60;
    angle_right = 15;
    angle_front = 45;
    reached_goal   = false;
    no_solution    = false;
    travel_dist_after_bump = 0.0;
    status = 0;

end

function bug2_trace_boundary(serPort)
    global no_solution;
    global reached_goal;
    global move_speed;
    global loop_pause_time;
    global angle_right;
    global angle_front;
    global angle_left;
    global travel_dist_after_bump;
    global status;
    
    status = 1;
    
    while true
        update_bug2(serPort);

        [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
        BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        bumped= BumpRight || BumpLeft || BumpFront;
        wallSensor = WallSensorReadRoomba (serPort);

        if bumped
            if BumpRight
              display('bump right');
              turnAngle (serPort, 0.2, angle_right);
              pause(0.05)
            elseif BumpLeft
              display('bump left');
              turnAngle (serPort, 0.2, angle_left);
              pause(0.05)
            elseif BumpFront
              display('bump front');
              turnAngle (serPort, 0.2, angle_front);
              pause(0.05)
            end

        elseif ~wallSensor %need to turn back to obstacle
            SetFwdVelRadiusRoomba (serPort, move_speed, -0.2);
        else %move forward              
            SetFwdVelRadiusRoomba(serPort, move_speed, Inf);                                       
        end
        update_bug2 (serPort);

        % check break loop conditions
        if is_in_mline() && travel_dist_after_bump > 0.5
            display(travel_dist_after_bump);
            display('back to mline');
            
            if back_to_bumped_point()
                no_solution = true;
                return;
            end
            
            if checkLocation()
                reached_goal = true;
                return;
            end
            
            if is_closer_to_goal()
                travel_dist_after_bump = 0;
                display('is closer to goal - now try to leave obstacle');
                success = try_leave_obstacle(serPort); 
                
                if (success)
                    % keep going
                    status = 2;                  
                    return;
                end
            end
        end
        pause(loop_pause_time);       
    end % end of tracing boundary loop
end

% this is sketchy
function success = try_leave_obstacle(serPort)
    global bug2_x;
    global dist_to_goal;
    global bug2_angle;
    global move_speed;
    global loop_pause_time;
    display('try to leave obstable')
    
    success = true;
    
    alpha = 0.6; % the arbitrary parameter we set to compensate the 
                 % 'over-turning' by the physical machine
    if (bug2_x > dist_to_goal)
        % at the other side, need to turn back, turn to 180 degree
        % -2*pi < total_angle < 2*pi
        while (abs(abs(bug2_angle)- pi ) * 180 / pi > 2)
            if bug2_angle < -pi || (bug2_angle < pi && bug2_angle > 0)
                turnAngle (serPort, 0.2, abs(pi - abs(bug2_angle)) * 180 * alpha / pi);
            else
                turnAngle (serPort, 0.2, (-1) * abs(pi - abs(bug2_angle)) * 180 * alpha / pi);
            end
            update_bug2 (serPort);           
        end
    else
        while (abs(bug2_angle) * 180 / pi > 2) % turn to 0 degree
            turnAngle (serPort, 0.2, (-1)* bug2_angle * 180 * alpha / pi);
            update_bug2 (serPort);
        end
    end

    % try moving
    bumped_obstacle = false;
    N = 0;
    % try move foward for 10 runs
    while ~bumped_obstacle && N < 8
        % try slightly moving forward
        travelDist (serPort, move_speed, 0.01);

        [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
            BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        if(isnan(BumpRight) || isnan(BumpLeft) || isnan(BumpFront))
            display('bad connection (getting broken signal)');
            display('    we recommand you to abort the program');
        else
            bumped_obstacle = BumpRight || BumpLeft || BumpFront;
        end
        update_bug2 (serPort);
        N = N + 1;
        pause(loop_pause_time);
    end
        
    if bumped_obstacle
        display('leaving obstacle not succesful - keep tracing boundary');

        % back off for a little bit
        travelDist (serPort, 0.025, -0.01);

        update_bug2 (serPort);
        success = false;
        return;
    else
        display('leaving obstacle successfully');
        update_bug2(serPort);
    end
    
    
end

function isTrue = back_to_bumped_point()
    global bumped_dist_x;
    global bumped_dist_y;
    global bug2_x;
    global bug2_y;
    isTrue = false;
    
    dist = sqrt((bug2_x - bumped_dist_x)^2 + (bug2_y - bumped_dist_y)^2);
    
    if (dist < 0.2)
        isTrue = true;
    end
    
end

% update current position in bug2
function update_bug2(serPort)

    global total_x_dist;
    global total_y_dist;
    global total_angle;
    global total_dist;
    global travel_dist_after_bump;
    global dist_to_goal;
    
    global bug2_dist;
    global bug2_angle;
    global bug2_x;
    global bug2_y;
    
    dist = DistanceSensorRoomba(serPort);
    angle = AngleSensorRoomba(serPort);
  
    % update global variable
    total_dist = total_dist + dist;
    total_angle = total_angle + angle;
    
    % update bug2 variable
    bug2_dist = bug2_dist + dist;
    bug2_angle = bug2_angle + angle;
    
    % keep total_angle between -2*pi and 2*pi
    if bug2_angle >= 2*pi
        bug2_angle = bug2_angle - 2*pi;
    elseif bug2_angle < -2*pi
        bug2_angle = bug2_angle + 2*pi;
    end
    
    % update global variable
    total_x_dist = total_x_dist + dist * cos (total_angle);
    total_y_dist = total_y_dist + dist * sin (total_angle);
    
    % update bug2 variable
    bug2_x = bug2_x + dist * cos (bug2_angle);
    bug2_y = bug2_y + dist * sin (bug2_angle);
    display(bug2_x);
    display(dist_to_goal);
    
    
    travel_dist_after_bump = travel_dist_after_bump + abs(dist);
    
    % plotting
%     global fig_plotter;
%     
%     xlabel ('Position in X-axis (m)');
%     ylabel ('Position in Y-axis (m)');
%     title  ('Position of iRobot');
%     figure (fig_plotter);
%     plot (bug2_x, bug2_y, 'o', 'MarkerEdgeColor','b', 'MarkerSize', 9);
%     dir_x = 0.001 * cos (total_angle);
%     dir_y = 0.001 * sin (total_angle);
%     plot (bug2_x + dir_x, bug2_y + dir_y, 'o', 'MarkerEdgeColor','r', 'MarkerSize', 4);
%     axis equal
%     hold on;

end

% check if robot is back to the start point
function isDone = checkLocation()

    global bug2_x;
    global bug2_y;
    global dist_to_goal;

    % this function is called only when robot is in m-line
    % so let's not let error in y-direction distract us
    radius = abs(bug2_x - dist_to_goal); 

    if (radius < 0.15)
        isDone = true;
        display (sprintf ('current y dist = %f', bug2_y));
        display (sprintf ('current x dist = %f', bug2_x));
        display (sprintf ('current radius = %f', radius));
    else
        isDone = false;
    end
end

function isCloser = is_closer_to_goal()

    global bug2_x;
    global bumped_dist_x;
    global dist_to_goal;    

    display (sprintf ('current x position in bug2 = %f', bug2_x));
    display (sprintf ('last bumped x position = %f', bumped_dist_x));

    if ( abs (dist_to_goal - bug2_x) < abs (dist_to_goal - bumped_dist_x) )
        isCloser = true;
    else
        isCloser = false;
    end
end

function in_mline = is_in_mline()

    global bug2_y;

    display (sprintf ('current total_y_dist = %f', bug2_y));

    if (abs(bug2_y) < 0.1)
        in_mline = true;
    else
        in_mline = false;
    end
end
