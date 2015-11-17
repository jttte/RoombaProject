function backtrack = dijkstra(G, pair)
    % pair     : idx of start and end point (1 * 2)
    % G        : adjacency matrix (N * N)
    % backtrack: idx of path from start to end point (1 * p)

    v = size(G, 1); % Number of vertices
    path = zeros(1, v);

    for i = 1:v
        for j = 1:v
            if G(i,j) <= eps
                G(i,j) = inf;
            end
        end
    end

    dist = inf(1, v);
    seen = ones(1, v);
    not_seen = v;

    dist(1, pair(1)) = 0;

    while not_seen > 0
        [distance index] = min(dist .* seen);

        if distance == inf
            break;
        end

        if index == pair(2)
            break;
        end

        seen(index) = inf;
        not_seen = not_seen - 1;

        for n = 1:v
            if seen(n) == 1
                alt = distance + G(index, n);
                if alt < dist(n)
                    dist(n) = alt;
                    path(n) = index;
                end
            end
        end
    end

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