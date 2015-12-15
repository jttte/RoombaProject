% [mark_x,mark_y] is coordinates of the points on the shortest path
function Run_hw4(serPort)
    global total_x_dist;
    global total_y_dist;
    global total_angle;
    global move_speed;
    global turn_speed;
    
    % clean records
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);
    
    init();
    init_plot();
%     mark_x = [-0.0368   -0.0368    0.0344    0.0344   -0.83  -0.6800];
%     mark_y = [0.6892    1.5092    2.9291    3.7491    8.2   13.7640];
    mark_x = [-0.0368   -0.0368    0.0344    0.0344   -0.7873   -0.7873   -0.6100];
    mark_y = [0.6892    1.5092    2.9291    3.7491    7.7916    8.6116   13.7640];
%     mark_x = [0];
%     mark_y = [4];
    
    for i=1:length(mark_x)
        if i == 6
            total_angle = total_angle - 15 * pi / 180;
        end
        dist = align(serPort,[total_x_dist,total_y_dist],[mark_x(i),mark_y(i)]);
%         max_dist = dist;
%         while abs(norm([total_x_dist,total_y_dist]-[mark_x(i),mark_y(i)]) - max_dist) < 0.7
%             travelDist(serPort,move_speed,0.5);
%             update(serPort);
%             SetFwdVelAngVelCreate(serPort, 0, 0);
%             dist = align(serPort,[total_x_dist,total_y_dist],[mark_x(i),mark_y(i)]);
%             pause(1);
%         end
        if dist > 1.5
            turnAngle(serPort,turn_speed,3);
            total_angle = total_angle - pi/90;
        end
        travelDist(serPort,move_speed,dist);
        update(serPort);
        SetFwdVelAngVelCreate(serPort, 0, 0);
        
%             display([total_x_dist,total_y_dist]);
%             display([mark_x(i),mark_y(i)]);
%         dist = align(serPort,[total_x_dist,total_y_dist],[mark_x(i),mark_y(i)]);
%         travelDist(serPort,move_speed,dist);
%         update(serPort);
%         SetFwdVelAngVelCreate(serPort, 0, 0);
        display('total x y');
        display([total_x_dist,total_y_dist]);
        display('goal');
        display([mark_x(i),mark_y(i)]);
        display(total_angle);
        pause(0.2);
        % plotting
        global fig_plotter;

        xlabel ('Position in X-axis (m)');
        ylabel ('Position in Y-axis (m)');
        title  ('Position of iRobot');
        figure (fig_plotter);
        plot (total_x_dist, total_y_dist, 'o', 'MarkerEdgeColor','b', 'MarkerSize', 9);
        
        axis equal
        hold on;
        
    end
    display(total_x_dist);
    display(total_y_dist);
end


function dist = align(serPort,current,goal)
    global total_angle;
    global turn_speed;
%     display(current)
%     display(goal)
    angle  = acos(dot(goal-current,[1,0])/norm(goal-current));
%     display(angle);
    if goal(2)>current(2)
        error = angle - total_angle;
        % Real robot
        angle = 180*error/pi;
        turnAngle(serPort, turn_speed ,angle);
    else
        error = (-angle + 2*pi)-total_angle;
        % Real robot
        angle = 180*error/pi;
        turnAngle(serPort, turn_speed ,angle);
    end
    %error = angle;
%     while abs(error)>1
%         turnAngle(serPort,turn_speed,sign(error));
%         update(serPort);  
%         if goal(2)>current(2)
%             error = angle - total_angle;
%             error = 180*error/pi;
%         else
%             error = (-angle + 2*pi)-total_angle;
%             error = 180*error/pi;
%         end
%     end
    update(serPort);  
%     display(total_angle);
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
    
%     % plotting
%     global fig_plotter;
%     
%     xlabel ('Position in X-axis (m)');
%     ylabel ('Position in Y-axis (m)');
%     title  ('Position of iRobot');
%     figure (fig_plotter);
%     plot (total_x_dist, total_y_dist, 'o', 'MarkerEdgeColor','b', 'MarkerSize', 9);
%     dir_x = 0.001 * cos (total_angle);
%     dir_y = 0.001 * sin (total_angle);
%     plot (total_x_dist + dir_x, total_y_dist + dir_y, 'o', 'MarkerEdgeColor','r', 'MarkerSize', 4);
%     axis equal
%     hold on;
end

function init()
    global total_angle;
    global total_x_dist;
    global total_y_dist;
    global turn_speed;
    global move_speed;

    move_speed = 0.3;
    turn_speed = 0.05;    
    total_x_dist = 0;
    total_y_dist = 0;
    total_angle = pi/2;
end

function init_plot()
    
    global fig_plotter;
    
    fig_plotter = figure;
    axis equal;
    xlabel ('Position in X-axis (m)');
    ylabel ('Position in Y-axis (m)');
    title  ('Position of iRobot');
end