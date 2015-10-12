function msg = drawEpipolarLines(lines1, points1, lines2, points2)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
figure();
hold on;
subplot(1,2,1)
axis([-510 510 -100 510]); % Defining axes limited with the size of our image
axis manual;
hold on;
title('Epipolar lines camera 1');
%line([1, 256, 256, 1, 1],[1, 1, 256, 256, 1],'Color','k');
%hold on;
scatter(points1(1,:),points1(2,:),'r+');
hold on;
a1 = zeros(size(points1,2));
b1 = zeros(size(points1,2));
x1 = (linspace(-500,500))';
for i=1:size(points1,2)
    a1(i) = -lines1(1,i)/lines1(2,i);
    b1(i) = -lines1(3,i)/lines1(2,i);
end
y1 = zeros(size(x1,1));
for j=1:size(points1,2)
    for i= 1:size(x1,1)
        y1(i) = a1(j)*x1(i)+b1(j);
    end
    line(x1,y1); 
end

subplot(1,2,2)
axis([-510 510 -100 510]); % Defining axes limited with the size of our image
axis manual;
hold on;
title('Epipolar lines camera 2');
%line([1, 256, 256, 1, 1],[1, 1, 256, 256, 1],'Color','k');
%hold on;
scatter(points2(1,:),points2(2,:),'r+');
hold on;
a2 = zeros(size(points2,2));
b2 = zeros(size(points2,2));
x2 = (linspace(-500,500))';
for i=1:size(points2,2)
    a2(i) = -lines2(1,i)/lines2(2,i);
    b2(i) = -lines2(3,i)/lines2(2,i);
end
y2 = zeros(size(x1,1));
for j=1:size(points2,2)
    for i= 1:size(x2,1)
        y2(i) = a2(j)*x2(i)+b2(j);
    end
    line(x2,y2); 
end
msg = 0;
end

