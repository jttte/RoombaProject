function backtrack = dijkstra(G, pair)
    % pair     : idx of start and end point (1 * 2)
    % G        : adjacency matrix (N * N)
    % backtrack: idx of path from start to end point (1 * p)

    v = size(G, 1); % Number of vertices
    path = zeros(1, v);
    dist = inf(1, v);

    vertices = 1:v;
    dist(pair(1))=0;
    
    while ~isempty(vertices)
        [distance index] = min(dist);
        node = vertices(index);
        
        % early break condition
        if node == pair(2)
            break;
        end
        
        % take out this visited node
        vertices = vertices(vertices ~= node);
        dist = dist(dist ~= distance);

        for n = 1:length(vertices)
            m = vertices(n);
            if G(node, m) > 0
                alt = distance + G(node, m);
                if alt < dist(n)
                    dist(n) = alt;
                    path(m) = node;
                end
            end
        end
    end
    
%     display(pair)
%     display(path)
%     display(dist)

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
    
    backtrack = [backtrack, pair(2)]

end