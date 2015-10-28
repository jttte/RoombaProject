function  Scan(serPort)
    global Map;
    global cur_locat;
    init();
    while(1)
        % Whether the Map is searched completely
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
        move(serPort,cur_locat,goal);
        cur_locat = goal;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        Map(cur_locat(1),cur_locat(2)) = 1;
        Map_plot();
%        pause(0.2);
    end
end
function init()
    global Map;
    global cur_locat;
    global start_locat;
    global Map_size;
    global current_angle;
    current_angle = 0;
    Map_size = [11,11];     % The last row/column is not used in colormap
    start_locat = [5,5];
    cur_locat = start_locat;
    Map = zeros(Map_size(1),Map_size(2));
    Map(end,:)=1; Map(:,end)=1;
    Map(start_locat(1),start_locat(2)) = 1;
    Map(2:4,2:4)=0.5; Map(6:10,6:10)=0.5;
    Map_plot();
end

function Map_plot()
    global Map;
    figure(1);
    color_map = [1 1 1; 0 0 0.6; 0.8 0.8 0];
    colormap(color_map);
    pcolor(Map);
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
function move(serPort,current,goal)
    global current_angle;
    if goal(2)<=current(2)
        theta = acos(dot(goal-current,[1,0])/norm(goal-current))-current_angle;
        turnAngle (serPort, 0.4, 180*theta/pi);
%        display(180*theta/pi)
        current_angle = acos(dot(goal-current,[1,0])/norm(goal-current));
    else
        theta = acos(dot(goal-current,[1,0])/norm(goal-current))+current_angle;
        turnAngle (serPort, 0.4, -180*theta/pi);
        display(180*theta/pi)
        current_angle = -acos(dot(goal-current,[1,0])/norm(goal-current));
    end
    travelDist (serPort, 0.4, norm(goal-current)*0.3);      % The parameter may need to change.
    display(current_angle);
    pause(0.2);
end