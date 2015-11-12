function hw4()

    global coordinates
    global dimension
    
    
    fileID = fopen('hw4_world_and_obstacles_convex.txt','r');
    formatSpec = '%f';
    coor_list = fscanf(fileID,formatSpec);
    N = coor_list(1);
    idx = 2;
    dimension = zeros(1, N)
    coordinates = zeros(20, 2*N);
    point_list_x = [];
    point_list_y = [];
    total_count = 0;
    for n = 1:N
        m = coor_list(idx);
        dimension(n) = m;
        for i = 1:m
            idx = idx + 1;
            coordinates(i, 1 + (n-1)*2) = coor_list(idx);
            
            idx = idx + 1;
            coordinates(i, 2 * n) = coor_list(idx);
        end
        idx = idx + 1;
    end
    
    % plot
    figHandle = figure;
    for i = 1:N
        x = coordinates(1:dimension(i), 2*i-1);
        y = coordinates(1:dimension(i), 2*i);
        plot(x,y,'g');
        hold on;
        plot([x(end), x(1)], [y(end), y(1)], 'g');
        hold on;
    end
    axis equal;
    
    fileID = fopen('hw4_start_goal.txt','r');
    formatSpec = '%f';
    start_end = fscanf(fileID,formatSpec);
    start_point = [start_end(1), start_end(2)];
    end_point = [start_end(3), start_end(4)];
    plot(start_point(1), start_point(2), '*', 'MarkerEdgeColor','r', 'MarkerSize', 4);
    hold on;
    plot(end_point(1), end_point(2), '*', 'MarkerEdgeColor','b', 'MarkerSize', 4);
    axis equal;
    
end

function build_vgraph()
    global coordinates
    global dimension
    global complete_vertex_list
    
    for i = 2:N
        d = dimension(i);
        for j = 1:d
            complete_vertex_list = [complete_vertex_list; [coordinates(1:dimension(i), 2*i-1:2*i)];
    end
    
end