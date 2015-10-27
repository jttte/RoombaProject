function original_hw1(serPort)
% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Output:
% finalRad - Double, final turning radius of the Create (m)

    %constants
    global found_obstacle;
    global sim_mode;
    global angle_left;
    global angle_right;
    global angle_front;
    global start_velocity;
    
    init();
    
    % Start robot moving
    SetFwdVelAngVelCreate(serPort, start_velocity, 0.0)
    pause(1)
    display('moving')
    
    
    % Enter main loop
    while true
        display('=================loop===================')
        
        if found_obstacle
            update_status (serPort);
            
            %check if the robot is back to start position
            if checkLocation() == true
                display ('back to starting point - Stop!')
                break;
            end
        end
        
        [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
        BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        bumped= BumpRight || BumpLeft || BumpFront;
        wallSensor = WallSensorReadRoomba (serPort);
        display(wallSensor)
        
        if bumped
            if (~found_obstacle)
                % reset moving stats
                DistanceSensorRoomba (serPort);
                AngleSensorRoomba (serPort);
                found_obstacle = true;
            end
            
            % back off for a little bit
            travelDist (serPort, 0.025, -0.01);
            update_status (serPort);
            
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
            update_status (serPort);
            
        elseif ~wallSensor && found_obstacle %need to turn back to obstacle
            display ('differntial turn');
            SetFwdVelRadiusRoomba (serPort, 0.25, -0.2);
            
        else %move forward              
            SetFwdVelRadiusRoomba(serPort, 0.25, inf);
            pause(0.1)                                       
        end
        
        % Briefly pause to avoid continuous loop iteration
        pause(0.01)
    end
    
    % Stop robot motion
    SetFwdVelAngVelCreate(serPort, 0, 0)
    
    % If you call RoombaInit inside the control program, this would be a
    % good place to clean up the serial port with...
    if ~sim_mode
        fclose(serPort)
        delete(serPort)
        clear(serPort)
    end
end

function init()
    global found_obstacle;
    global sim_mode;
    global angle_left;
    global angle_right;
    global angle_front;
    global total_x_dist;
    global total_y_dist;
    global total_angle;
    global total_dist;
    global start_velocity;
    
    sim_mode       = true;
    found_obstacle = false;
    total_dist     = 0;
    total_x_dist   = 0.0;
    total_y_dist   = 0.0;
    total_angle    = 0.0;
    
    if sim_mode %set simulation test parameters
        angle_left  = 60;
        angle_right = 15;
        angle_front = 45;
        start_velocity = 0.25;
    else %set physical test parameters
        angle_left  = 60;
        angle_right = 15;
        angle_front = 45;
        start_velocity = 0.02;
    end
end

function update_status (serPort)

    global total_x_dist;
    global total_y_dist;
    global total_angle;
    global total_dist;

    dist = DistanceSensorRoomba (serPort)
    angle = AngleSensorRoomba (serPort)

    total_dist = total_dist + dist;
    total_angle = total_angle + angle;
    total_x_dist = total_x_dist + dist * cos (total_angle);
    total_y_dist = total_y_dist + dist * sin (total_angle);
end

function isDone= checkLocation ()

    global total_x_dist;
    global total_y_dist;
    global total_dist;

    radius = sqrt (total_x_dist ^ 2 + total_y_dist ^ 2);

    display (sprintf ('current radius = %f', radius));
    display (sprintf ('current g_total_dist = %f', total_dist));

    if (total_dist > 0.5 && radius < 0.3)
        isDone = true;
    else
        isDone = false;
    end
end
