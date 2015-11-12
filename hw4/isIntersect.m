function isTrue = isIntersect(line1, line2)
    isTrue = true;
    slope = @(line) (line(2,2) - line(1,2))/(line(2,1) - line(1,1));
    m1 = slope(line1);
    m2 = slope(line2);
    
    % check parallel case ?
    if m1 == m2
        isTrue = false;
        return;
    end
        

    intersect = @(line,m) line(1,2) - m*line(1,1);
    b1 = intersect(line1,m1);
    b2 = intersect(line2,m2);
    xintersect = (b2-b1)/(m1-m2);
    yintersect = m1*xintersect + b1;
    
    isPointInside = @(xint,myline) ...
    (xint >= myline(1,1) && xint <= myline(2,1)) || ...
    (xint >= myline(2,1) && xint <= myline(1,1));
    
    inside = isPointInside(xintersect,line1) && ...
         isPointInside(xintersect,line2) && ...
         isPointInside(yintersect,line1) && ...
         isPointInside(yintersect,line2);
     
    if inside == false
        isTrue = false;
    end
    
end