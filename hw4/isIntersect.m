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

function isTrue = isIntersect(line1, line2)
    % local functions
    isTrue = true;
    slope = @(line) (line(2,2) - line(1,2))/(line(2,1) - line(1,1));
    
    intersect = @(line,m) line(1,2) - m*line(1,1);
    
    isPointXInside = @(xint,myline) ...
    (xint >= myline(1,1) && xint <= myline(2,1)) || ...
    (xint >= myline(2,1) && xint <= myline(1,1));

    isPointYInside = @(yint,myline) ...
    (yint >= myline(1,2) && yint <= myline(2,2)) || ...
    (yint >= myline(2,2) && yint <= myline(1,2));
    
    closeTo = @(point1, point2) (abs(point1-point2)<0.001);
    
    % same line segment
    if  closeTo(line2(1, 1),line1(1, 1))  &&...
        closeTo(line2(1, 2),line1(1, 2))  &&...
        closeTo(line2(2, 1),line1(2, 1))  &&...
        closeTo(line2(2, 2),line1(2, 2))
        isTrue = false;
        return;
    end
    
    if  closeTo(line2(1, 1),line1(2, 1))  &&...
        closeTo(line2(1, 2),line1(2, 2))  &&...
        closeTo(line2(2, 1),line1(1, 1))  &&...
        closeTo(line2(2, 2),line1(1, 2))
        isTrue = false;
        return;
    end
    
    
        
    m1 = slope(line1);
    m2 = slope(line2);  
    
    % vertical line segment
    if line1(2, 1) == line1(1, 1) && line2(2, 1) ~= line2(1, 1)
        b = line2(1,2)-m2*line2(1,1);
        t = line1(1, 1)*m2 + b;

        if isPointYInside (t, line1) == false
            isTrue = false;
            return;
        end
        if isPointXInside (line1(2, 1), line2) == false
            isTrue = false;
            return;
        end

        if closeTo(line2(1,1), line1(2, 1)) || closeTo(line2(2,1), line1(2, 1))
            isTrue = false;
        end
        return;
    end
    
    if line2(2, 1) == line2(1, 1) && line1(2, 1) ~= line1(1, 1)
        b = line1(1,2)-m1*line1(1,1);
        t = line2(1, 1)*m1 + b;

        if isPointYInside (t, line2) == false
            isTrue = false;
            return;
        end
        
        if isPointXInside (line2(2, 1), line1) == false
            isTrue = false;
            return;
        end

        if closeTo(line1(1,1), line2(2, 1)) || closeTo(line1(2,1), line2(2, 1))
            isTrue = false;
        end
        return;
    end
    
    % check parallel case
    if closeTo(1/m1, 1/m2) || closeTo(m1, m2)
%         b1 = intersect(line1,m1);
%         b2 = intersect(line2,m2);
%         if closeTo(b1, b2)
%             isTrue = true;
%             return;
%         end
        isTrue = false;
        return;
    end
        

    
    b1 = intersect(line1,m1);
    b2 = intersect(line2,m2);
    xintersect = (b2-b1)/(m1-m2);
    yintersect = m1*xintersect + b1;
    
    
    
    inside = isPointXInside(xintersect,line1) && ...
         isPointXInside(xintersect,line2) && ...
         isPointYInside(yintersect,line1) && ...
         isPointYInside(yintersect,line2);
     

    if ( closeTo(xintersect,line1(1, 1)) && closeTo(yintersect,line1(1, 2)) ) ||...
       ( closeTo(xintersect,line1(2, 1)) && closeTo(yintersect,line1(2, 2)) ) ||...
       ( closeTo(xintersect,line2(1, 1)) && closeTo(yintersect,line2(1, 2)) ) ||...
       ( closeTo(xintersect,line2(2, 1)) && closeTo(yintersect,line2(2, 2)) )
        inside = false;
    end

     
    if inside == false
        
%     figure;
%     plot([line1(1,1), line1(2,1)],[line1(1,2), line1(2,2)], 'r');
%     hold on;
%     plot([line2(1,1), line2(2,1)],[line2(1,2), line2(2,2)], 'r');
        isTrue = false;
    end
   
    
end