function [lines] = epipolarLines(points, F)
%Output: array = [u1, u2, u3]
%Input: normalized point = [x, y, 1]
%       camera representation M = A*cKw   
%The projection of a point (represented as a vector in homogeneous
%coodinates) over its epipole line is zero array*point=0
lines = zeros(3, size(points,2));
for i=1:size(points,2)
    lines(:,i)= F*(points(:,i));
end
end

