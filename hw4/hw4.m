function hw4()

    global coordinates
    global dimension
    global n_total_vertices
    global start_point
    global goal_point
    global E_obstacles
    global figHandle
    global wall
    
    E_obstacles = [];
    n_total_vertices = 0;
    fileID = fopen('hw4_world_and_obstacles_convex_copy.txt','r');
    formatSpec = '%f';
    coor_list = fscanf(fileID,formatSpec);
    N = coor_list(1)-1;
    idx = 2;
    dimension = zeros(1, N)
    coordinates = zeros(20, 2*N);
    
    % store wall vertices somewhere else
    wall = [] 
    m = coor_list(idx);
    idx = idx + 1;
    for i = 1:m
        wall = [wall; [coor_list(idx), coor_list(idx+1)]];
        idx = idx + 2;
    end
    
    for n = 1:N
        m = coor_list(idx);
        dimension(n) = m;
        for i = 1:m
            idx = idx + 1;
            coordinates(i, 1 + (n-1)*2) = coor_list(idx);
            
            idx = idx + 1;
            coordinates(i, 2 * n) = coor_list(idx);
            
            n_total_vertices = n_total_vertices + 1;
        end
        idx = idx + 1;
    end
    
    % plot
    figHandle = figure;
    
    % wall
    x = wall(:, 1);
    y = wall(:, 2);
    plot(x,y,'g');
    hold on;
    plot([wall(end, 1), wall(1, 1)], [wall(end, 2), wall(1, 2)], 'g');
    hold on;
    
    % obstacles
    for i = 1:N
        x = coordinates(1:dimension(i), 2*i-1);
        y = coordinates(1:dimension(i), 2*i);
        plot(x,y,'g');
        hold on;
        plot([x(end), x(1)], [y(end), y(1)], 'g');
        hold on;
    end
    
    % start, goal
    fileID = fopen('hw4_start_goal.txt','r');
    formatSpec = '%f';
    start_end = fscanf(fileID,formatSpec);
    start_point = [start_end(1), start_end(2)];
    goal_point = [start_end(3), start_end(4)];
    plot(start_point(1), start_point(2), '*', 'MarkerEdgeColor','r', 'MarkerSize', 4);
    hold on;
    plot(goal_point(1), goal_point(2), '*', 'MarkerEdgeColor','b', 'MarkerSize', 4);
    axis equal;
    
    build_vgraph();
    
end

function build_vgraph()
    global coordinates
    global dimension
    global complete_vertex_list
    global n_total_vertices
    global E_map
    global V_graph
    global start_point
    global goal_point
    global E_obstacles
    global figHandle
    global wall
     
    N = size(dimension);
    complete_vertex_list = [];
    
    E_map = zeros(n_total_vertices + 2 + size(wall, 1), n_total_vertices + 2, size(wall, 1));
    
    idx = 1;
    for i = 1:N(2)
        d = dimension(i);
        complete_vertex_list = [complete_vertex_list; coordinates(1:d, 2*i-1:2*i)];
            
        for j = 1:d
            
            if j > 1
                E_map(idx, idx - 1) = 1;
                E_map(idx - 1, idx) = 1;
                E_obstacles = [E_obstacles; [idx, idx - 1]];
            end
            idx = idx + 1;
        end
        E_map(idx - 1, idx - d) = 1;
        E_map(idx - d, idx - 1) = 1;
        E_obstacles = [E_obstacles; [idx - 1, idx - d]];
    end
    
    % put in start and goal points
    complete_vertex_list = [complete_vertex_list; start_point; goal_point];
    idx = idx + 2;
    
    % put back wall vertices
    d = size(wall, 1);
    for i = 1:d
        complete_vertex_list = [complete_vertex_list; wall(i, :)];
        if i > 1
            E_map(idx, idx - 1) = 1;
            E_map(idx - 1, idx) = 1;
            E_obstacles = [E_obstacles; [idx, idx - 1]];
        end
        idx = idx + 1;       
    end
    E_map(idx - 1, idx - d) = 1;
    E_map(idx - d, idx - 1) = 1;
    E_obstacles = [E_obstacles; [idx - 1, idx - d]];
    
    
    
%     figure;
%     imagesc(E_map);
    
    V_graph = zeros(n_total_vertices + 2, n_total_vertices + 2);
    
    % if edge(i, j) doesn't intersect with obstacles, add edge(i, j) to
    % v_graph
    figure(figHandle);
    t = 0;
    for i = 1 : n_total_vertices + 2
       for j = i + 1 : n_total_vertices + 2
           if E_map(i, j) == 1
               continue;
           end
           add = true;
           for k = 1:size(E_obstacles, 1)
               point1 = complete_vertex_list(E_obstacles(k, 1), :);
               point2 = complete_vertex_list(E_obstacles(k, 2), :);
               % has intersection
               if isIntersect([point1; point2], ...
                       [complete_vertex_list(i, :);...
                        complete_vertex_list(j, :)]) == true
                  add = false;
                  break;
               end              
           end
           if add
               V_graph(i, j) = 1;
               V_graph(j, i) = 1;
               t = t + 1
               plot([complete_vertex_list(i, 1), complete_vertex_list(j, 1)],...
                   [complete_vertex_list(i, 2), complete_vertex_list(j, 2)],...
                   'b');
               hold on;
           end
           
       end
    end
    
    % plot V graph
%     figure(figHandle);
%     for i = 1 : n_total_vertices + 2
%        for j = i + 1 : n_total_vertices + 2
%            if V_graph(i, j) == 1
%                plot([complete_vertex_list(i, 1), complete_vertex_list(j, 1)],...
%                    [complete_vertex_list(i, 2), complete_vertex_list(j, 2)],...
%                    'b');
%            end
%        end
%     end
    
    
end