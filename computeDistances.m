function [mean, stdDev] = computeDistances( lines2D, points )
% This function will return the mean of total distance for all lines with respect
% the points that should be contained by the line itself
mean = 0;
stdDev = 0;
for i=1:size(lines2D,2)
    mean = mean + dist2line(points(1,i),points(2,i),...
        lines2D(:,i));
end
mean = mean /size(lines2D,2);
for i=1:size(lines2D,2)
    stdDev = stdDev +(mean - dist2line(points(1,i),points(2,i),lines2D(:,i)))^2;
end
stdDev = sqrt(stdDev/size(lines2D,2));
end

