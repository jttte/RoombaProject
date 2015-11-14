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
        m = coor_list(idx);     % Dimension of Obstacle i
        dimension(n) = m;
        for i = 1:m
            idx = idx + 1;
            coordinates(i, 1 + (n-1)*2) = coor_list(idx);
            idx = idx + 1;
            coordinates(i, 2 * n) = coor_list(idx);
        end
        idx = idx + 1;
    end
    
    % Before grow the obstacle
    figure(1);
    for i = 1:N
        x = coordinates(1:dimension(i), 2*i-1);
        y = coordinates(1:dimension(i), 2*i);
        plot(x,y,'k');
        hold on;
        plot([x(end), x(1)], [y(end), y(1)], 'k');
        hold on;
    end
     axis([-6 6 -4 12]);
     axis equal;
     

 
    figure(2);
    % Draw the surroundings
    x = coordinates(1:dimension(1), 1);
    y = coordinates(1:dimension(1), 2);
    plot(x,y,'k');
    hold on;
    plot([x(end), x(1)], [y(end), y(1)], 'k');
    hold on;

     M = N -1;
     obs_dim = dimension(2:end);
     obs = coordinates(1:end,3:end);
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
%        plot(coor_x{i},coor_y{i},'r*');
%        plot(xG{i},yG{i},'b*')
        hold on;
     end
     axis([-6 6 -4 12]);
     axis equal;
     

     
     

    fileID = fopen('hw4_start_goal.txt','r');
    formatSpec = '%f';
    start_end = fscanf(fileID,formatSpec);
    start_point = [start_end(1), start_end(2)];
    end_point = [start_end(3), start_end(4)];
    plot(start_point(1), start_point(2),'*', 'MarkerEdgeColor','r', 'MarkerSize', 4);
    txt = text(start_point(1), start_point(2),'start');
    txt.VerticalAlignment = 'top';
    hold on;
    plot(end_point(1), end_point(2), '*', 'MarkerEdgeColor','b', 'MarkerSize', 4);
    txt = text(end_point(1), end_point(2),'goal');
    txt.VerticalAlignment = 'top';
    axis equal;
    axis([-6 6 -4 12]);
    camroll(90)


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
            Rx(j) = x(i-1);Ry(j) = y(i-1)
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


