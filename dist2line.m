function distance = dist2line( x,y, line2D)
% This function will return the minimum distance between a point and a line
% considering the line in general form, and 2D point as P(x,y)
distance = abs(line2D(1)*x+line2D(2)*y+line2D(3))/sqrt((line2D(1))^2+(line2D(2))^2);
end

