function hw4_test()
    global coordinates
    global dimension
    global n_total_vertices
    global start_point
    global goal_point
    global E_obstacles
    global figHandle
    global wall
    global xG
    global yG
    global dimG
    
    global vertex_list_x
    global vertex_list_y
    
    E_obstacles = [];
    n_total_vertices = 0;
    fileID = fopen('hw4_world_and_obstacles_convex.txt','r');
    formatSpec = '%f';
    coor_list = fscanf(fileID,formatSpec);
    N = coor_list(1)-1;
    idx = 2;
    dimension = zeros(1, N)
    coordinates = zeros(20, 2*N);
    
    % plot
    figHandle = figure;
    
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
        plot(coordinates(1:m, 1 + (n-1)*2), coordinates(1:m, 2 * n), 'c');
        hold on;
        plot([coordinates(1, 1 + (n-1)*2), coordinates(m, 1 + (n-1)*2)],...
            [coordinates(1, 2 * n), coordinates(m, 2 * n)], 'c');
        hold on;
        idx = idx + 1;
    end
    
    % wall
    x = wall(:, 1);
    y = wall(:, 2);
    plot(x,y,'c');
    hold on;
    plot([wall(end, 1), wall(1, 1)], [wall(end, 2), wall(1, 2)], 'c');
    hold on;
%     axis([-6 6 -4 12]);
    axis equal;

 
    figure(figHandle);

    % grow obstacles
    M = N;
    obs_dim = dimension(1:end);
    obs = coordinates(1:end,1:end);
    robot = [0,0;-0.35,0;-0.35,-0.35;0,-0.35];
    [Gobs,Gdim] = Obj_grow(obs,M,obs_dim,robot);
    for i = 1:M
        x = Gobs(1:Gdim(i),2*i-1);
        y = Gobs(1:Gdim(i),2*i);
        k{i} = convhull(x,y);
        coor_x{i} = x(k{i});
        coor_y{i} = y(k{i});
        [xG{i},yG{i},dimG(i)] = removeP(coor_x{i},coor_y{i});
        plot(xG{i},yG{i},'k')
        hold on;
        plot([xG{i}(end), xG{i}(1)], [yG{i}(end), yG{i}(1)], 'k');
        hold on;        
    end
    axis([-6 6 -4 12]);
    axis equal;
     

    % add in start and goal
    fileID = fopen('hw4_start_goal.txt','r');
    formatSpec = '%f';
    start_end = fscanf(fileID,formatSpec);
    start_point = [start_end(1), start_end(2)];
    goal_point = [start_end(3), start_end(4)];
    plot(start_point(1), start_point(2),'*', 'MarkerEdgeColor','r', 'MarkerSize', 4);
    txt = text(start_point(1), start_point(2),'start');
    txt.VerticalAlignment = 'top';
    hold on;
    plot(goal_point(1), goal_point(2), '*', 'MarkerEdgeColor','b', 'MarkerSize', 4);
    txt = text(goal_point(1), goal_point(2),'goal');
    txt.VerticalAlignment = 'top';
    axis equal;
%     axis([-6 6 -4 12]);
    camroll(90)
    
    % compute visibility graph
    [grown_vertex_n, V_graph] = build_vgraph();
    
    % shortest path
    path = dijkstra(V_graph, [grown_vertex_n + 1, grown_vertex_n + 2]);
    
    for p = 2:length(path)
        i = path(p-1);
        j = path(p);
        plot([vertex_list_x(i), vertex_list_x(j)],...
             [vertex_list_y(i), vertex_list_y(j)],'r');
    end
    
    
end

% N is the number of obstacle,obs_dim is array contains the dimension of
% each obstacle; obs and rob are vertices matrix
function [Gobs,Gdim] = Obj_grow(obs,N,obs_dim,rob)
% The vertice with coordinate (0,0) is the reference point
    rob_dim = length(rob);
    Gdim = obs_dim;
    Gobs = obs;
    rob = -rob;         % Flip
    for i = 1:N
        row = obs_dim(i)+1;
        for j = 1:obs_dim(i)
            for k = 2:rob_dim
                Gobs(row,2*i-1) = Gobs(j,2*i-1) + rob(k,1);
                Gobs(row,2*i) = Gobs(j,2*i) + rob(k,2);
                row = row+1;
                Gdim(i) = Gdim(i)+1;
            end
        end
    end
end

function [Rx,Ry,j] = removeP(x,y)
    dim = length(x);
    j = 1;
 %   Rx(1) = x(1); Ry(1) = y(1);
    vec_SS = [x(2)-x(1),y(2)-y(1)];
    vec_S = vec_SS;
    S = 1;
    for i = 2:dim
        vec = [x(i)-x(S),y(i)-y(S)];
        cosin = dot(vec,vec_S)/(norm(vec)*norm(vec_S));
        if(cosin == 1)
            continue;
        else
            Rx(j) = x(i-1);
            Ry(j) = y(i-1);
            j = j+1;
            S = i-1;
            vec_S = [x(i)-x(S),y(i)-y(S)];
        end
    end
    cosin = dot(vec_S,vec_SS)/(norm(vec_S)*norm(vec_SS));
    if cosin ~= 1
        Rx(j) = x(1);Ry(j) = y(1);
        j = j+1;
    end
    j = j-1;    
end

function [grown_vertex_n, V_graph] = build_vgraph()
    global dimension
    global vertex_list_x
    global vertex_list_y
    global n_total_vertices
    global E_map
    global start_point
    global goal_point
    global E_obstacles
    global figHandle
    global wall
    global xG
    global yG
    
    % pre-computations
     
    N = length(dimension);
    banned_list = [];
    vertex_list_x = [];
    vertex_list_y = [];
    
    E_map = zeros(n_total_vertices + 2 + size(wall, 1), n_total_vertices + 2 + size(wall, 1));
    
    idx = 0;
    for i = 1:N
        cx = xG{i};
        cy = yG{i};
        d = length(cx);

        vertex_list_x = [vertex_list_x, cx];
        vertex_list_y = [vertex_list_y, cy];
        for j = 1 : d - 1
            for k = j+1 : d
                E_map(idx + j, idx + k) = 1;
                E_map(idx + k, idx + j) = 1;
                E_obstacles = [E_obstacles; [idx + j, idx + k]];
            end
        end
        
        % check if grown vertex is inside the polygon of other obstacles
        for v = 1:d
            x = cx(v);
            y = cy(v);
            for ii = 1:N
                if ii == i
                    continue;
                end
                if inpolygon(x,y,xG{ii},yG{ii}) % vertex (idx+v) is not accessable
                    banned_list = [banned_list idx + v];
                    plot (x, y, 'r*');
                    hold on;
                end
            end
            
            % check if grown vertex is still inside walls
            if ~inpolygon(x,y,wall(:, 1)',wall(:, 2)')
                banned_list = [banned_list idx + v];
                plot (x, y, 'r*');
                hold on;
            end
        end
        
        idx = idx + d;
        
    end
    
    % number of vertices that belong to obstacles
    grown_vertex_n = idx;
    
    % put start and goal points right behind obstacle points
    vertex_list_x = [vertex_list_x, start_point(1), goal_point(1)];
    vertex_list_y = [vertex_list_y, start_point(2), goal_point(2)];
    idx = idx + 2;
    
    % last, put in wall vertices
    d = size(wall, 1);
    idx = idx + 1;
    for i = 1:d
        vertex_list_x = [vertex_list_x, wall(i, 1)];
        vertex_list_y = [vertex_list_y, wall(i, 2)];
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
    
    
    % initialize v graph
    V_graph = zeros(grown_vertex_n + 2, grown_vertex_n + 2);
    
    % if edge(i, j) doesn't intersect with obstacles, add edge(i, j) to
    % v_graph
    figure(figHandle);
    for i = 1 : grown_vertex_n + 2
       if any(i==banned_list)
           continue;
       end
       for j = i + 1 : grown_vertex_n + 2
           if any(j==banned_list)
            continue;
           end
           add = true;
           for k = 1:size(E_obstacles, 1)
               point1 = [vertex_list_x(E_obstacles(k, 1)), vertex_list_y(E_obstacles(k, 1))];
               point2 = [vertex_list_x(E_obstacles(k, 2)), vertex_list_y(E_obstacles(k, 2))];
               % has intersection
               if isIntersect([point1; point2], ...
                       [vertex_list_x(i), vertex_list_y(i);...
                        vertex_list_x(j), vertex_list_y(j)]) == true
                  add = false;
                  break;
               end              
           end
           if add
               V_graph(i, j) = norm([vertex_list_x(i)-vertex_list_x(j), ...
                                    vertex_list_y(i)-vertex_list_y(j)]);
               V_graph(j, i) = V_graph(i, j);
               plot([vertex_list_x(i), vertex_list_x(j)],...
                   [vertex_list_y(i), vertex_list_y(j)],...
                   'b');
               hold on;
           end
           
       end
    end
    
end
