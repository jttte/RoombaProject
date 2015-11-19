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
function isTrue = is_turn_left(point1, point2, point3)
    angle1 = acos( (point2(1) - point1(1)) / norm (point2 - point1));
    
    if point2(2) - point1(2) < 0
        angle1 = 2 * (pi - angle1) + angle1;
    end
    
    angle2 = acos( (point3(1) - point2(1)) / norm (point3 - point2));
    if point3(2) - point2(2) < 0
        angle2 = 2 * (pi - angle2) + angle2;
    end
    
    if angle2 > angle1
        isTrue = true;
    else
        isTrue = false;
    end
        
end