%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2015
%
% Homework 4
%
% Team number: 24
% Team leader: Chia-Jung Lin (cl3295)
% Team members: Cheng Zhang (cz2398), Ming-Ching Chu (mc4107)
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function backtrack = dijkstra(G, pair)
    % pair     : idx of start and end point (1 * 2)
    % G        : adjacency matrix (N * N)
    % backtrack: idx of path from start to end point (1 * p)

    v = size(G, 1); % number of vertices
    % keep record of where this node is reached from
    path = zeros(1, v); 
    
    % the total distance to walk from source to node i
    dist = inf(1, v);
    
    % simulate the queue in the algorithm
    % if q(i) == 0, then node i is in queue. 
    % Else, i is viewed as visited and had been taken out of queue.
    q = zeros(1, v); 

    dist(1, pair(1)) = 0;

    while any(q==0)
        [distance index] = min(dist + q);

        if distance == inf
            break;
        end
        
        % early break condition
        if index == pair(2)
            break;
        end

        q(index) = inf;

        for n = 1:v
            if q(n) == 0 && G(index, n) > 0
                alt = distance + G(index, n);
                if alt < dist(n)
                    dist(n) = alt;
                    path(n) = index;
                end
            end
        end
    end

    % backtrack path from end point
    idx = pair(2);
    backtrack = [];

    while true
        next = path(idx);
        if next == 0
            break;
        end
        backtrack = [next, backtrack];
        idx = next;
    end
    
    backtrack = [backtrack, pair(2)];

end