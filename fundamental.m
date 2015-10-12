function [ F ] = fundamental( points, pointsC1, pointsC2 )
%   This function will return the fundamental matrix given
%   a number of projected points in the two cameras
%   This number should be at least 8

number = size(points, 2);

% The parameter f(3,3) will be fixed to 1 avoiding a zero vector solution
f = zeros(1,8);
u = zeros(number,8);
o = ones(number,1);

% Composition of matrix u. Warning: our matrix is in the form m'F'm
for i=1:number
    u(i,1)=pointsC1(1,i)*pointsC2(1,i);
    u(i,2)=pointsC1(2,i)*pointsC2(1,i);
    u(i,3)=pointsC2(1,i);
    u(i,4)=pointsC1(1,i)*pointsC2(2,i);
    u(i,5)=pointsC1(2,i)*pointsC2(2,i);
    u(i,6)=pointsC2(2,i);
    u(i,7)=pointsC1(1,i);
    u(i,8)=pointsC1(2,i);
end
f = (-pinv(u)*o)';
f = [f, 1];
F = reshape(f, 3,3);
F = F';
end


