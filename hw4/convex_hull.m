function [Sx, Sy, dim] = convex_hull(x, y)

   v = length(y);
   % Find the bottommost point
   min_y = y(1);
   min_y_idx = 1;
   for i = 2:v
     % Pick the bottom-most or chose the left most point in case of tie
     if y(i) < min_y || (min_y == y(i) && x(i) > x(min_y_idx))
        min_y = y(i);
        min_y_idx = i;
     end
 
     
   end
   
   % Place the bottom-most point at first position
   tmpx = x(1);
   tmpy = y(1);
   x(1) = x(min_y_idx);
   y(1) = min_y;
   x(min_y_idx) = tmpx;
   y(min_y_idx) = tmpy;

   % Sort n-1 points with respect to the first point.  A point p1 comes
   % before p2 in sorted ouput if p2 has larger polar angle (in 
   % counterclockwise direction) than p1
   
   % construct angle array
   angle = zeros(v, 1);
   for i = 2:v
       angle(i) = acos( (x(i) - x(1)) / norm ([x(i) - x(1), y(i) - y(1)]));
   end
   
   % sort rows(points) by angle
   P = sortrows([x, y, angle], 3);
   % in case of tie, sort by ascending distance...
   i = 2;
   while i < v
       tie_a = P(i, 3);
       n = 1;
       idx = i + 1;
       while idx <= v && P(idx, 3) == tie_a
           n = n + 1;
           idx = idx + 1;
       end
       
       if n > 1
        mx = abs (P(i:i+n-1, 1) - P(1,1)); % take abs value
        my = abs (P(i:i+n-1, 2) - P(1,2));
        new_matrix = [P(i:i+n-1, 1:2), mx + my];
        sorted_matrix = sortrows(new_matrix, 3); % sort by x + y
        P(i:i+n-1, 1:2) = sorted_matrix(:, 1:2); % put sorted values back
       end
       
       i = i + n;
   end
   
   % Create an empty stack and push first three points to it.
   S = [1 2 3];
   top = 3;
  
   % process remaining v-3 points
   for i = 4:v
       while top >= 2 && ~is_turn_left(P(S(top-1), 1:2), P(S(top), 1:2), P(i, 1:2))
           top = top - 1; % pop
           S = S(1:top);
       end
       S = [S i]; % push i
       top = top + 1;
   end
 
   % Now stack has the output points, print contents of stack
   Sx = P(S', 1);
   Sy = P(S', 2);
   dim = length(S);
%    display(S);
% %    
%    %
%    figure;
%    for i = 1:v
%        plot(P(i, 1), P(i, 2), '*');
%        text(P(i, 1), P(i, 2), num2str(i));
%        hold on;
%    end
end