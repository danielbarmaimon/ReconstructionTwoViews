clear all;
close all;
clc;
%% ------------------- Step 1 --------------------------------------------
% Parameters of the first camera (respect to the world, itself) %%%%%%%%%%
au1 = 100;  % Relation between focal length and pixel area
av1 = 120;  % Relation between focal length and pixel area
uo1 = 128;  % Principal point from upper corner in image plane
vo1 = 128;  % Principal point from upper corner in image plane
%Image size: 256 x 256
disp('STEP 1');
% Defining intrinsic matrix
A1 = [au1 0 uo1;...
      0 av1 vo1;...
      0 0 1]

%% ------------------- Step 2 --------------------------------------------
% Parameters of the second camera (resptect to camera 1)
au2 = 90;
av2 = 110; 
uo2 = 128;
vo2 = 128; 
ax = 0.1    % Rotation angle around x axis in rad; 
by = pi/4.0 % Rotation angle around y axis in rad;  
cz = 0.2    % Rotation angle around z axis in rad;  
%XYZ EULER 
tx = -1000; %Distances in mm 
ty = 190;   %Distances in mm 
tz = 230;   %Distances in mm 
%Image size: 256 x 256
% Defining intrinsic matrix
disp('--------------------------');
disp('STEP 2');
A2 = [au2 0 uo2;...
      0 av2 vo2;...
      0 0 1]

%% ------------------- Step 3 --------------------------------------------
% Defining extrinsic parameters camera 1
disp('--------------------------');
disp('STEP 3');
E1 = [1 0 0;...
       0 1 0;...  
       0 0 1]
  
% Defining extrinsic parameters camera 2
R1 = [1 0 0;0 cos(ax) -sin(ax); 0 sin(ax) cos(ax)];
R2 = [cos(by) 0 sin(by); 0 1 0; -sin(by) 0 cos(by)];
R3 = [cos(cz) -sin(cz) 0; sin(cz) cos(cz) 0; 0 0 1];

R = R1*R2*R3;         % Rotation Matrix
t = [tx ty tz]';      % Translation

T = [0 -tz ty;... 
     tz 0 -tx;...
     -ty tx 0] % Antisymmetric Matrix for translation (check)

%% ------------------- Step 4 --------------------------------------------
disp('--------------------------');
disp('STEP 4');
F = (inv(A2))'*R'*T*inv(A1);
F = F/F(3,3)

%% ------------------- Step 5 ---------------------------------------------
disp('--------------------------');
disp('STEP 5');
disp('Introducing the points in the system');
V(:,1) = [100;-400;2000;1]; 
V(:,2) = [300;-400;3000;1]; 
V(:,3) = [500;-400;4000;1]; 
V(:,4) = [700;-400;2000;1]; 
V(:,5) = [900;-400;3000;1]; 
V(:,6) = [100;-50;4000;1]; 
V(:,7) = [300;-50;2000;1]; 
V(:,8) = [500;-50;3000;1]; 
V(:,9) = [700;-50;4000;1]; 
V(:,10) = [900;-50;2000;1]; 
V(:,11) = [100;50;3000;1]; 
V(:,12) = [300;50;4000;1]; 
V(:,13) = [500;50;2000;1]; 
V(:,14) = [700;50;3000;1]; 
V(:,15) = [900;50;4000;1]; 
V(:,16) = [100;400;2000;1]; 
V(:,17) = [300;400;3000;1]; 
V(:,18) = [500;400;4000;1]; 
V(:,19) = [700;400;2000;1]; 
V(:,20) = [900;400;3000;1]; 
%% ------------------- Step 6 ---------------------------------------------
disp('--------------------------');
disp('STEP 6');
ex1 = [0 0 1]'  % Extension for comp. reason
c1Kw = [E1,ex1] % Extrinsic m. camera 1
ex2 = -(R'*t)   % Rigid transformation
c2Kw = [R',ex2] % Extrinsic m. camera 2

M1 = A1*c1Kw;   % Camera m. for camera 1
M2 = A2*c2Kw;   % Camera m. for camera 2
M1 = M1/M1(3,4);% Normalization
M2 = M2/M2(3,4);% Normalization

Vc1 = projectingPoints(V', M1); 
Vc2 = projectingPoints(V', M2);
for i=1:size(Vc1,2)
    Vc1(:,i) = Vc1(:,i)/Vc1(3,i);
    Vc2(:,i) = Vc2(:,i)/Vc2(3,i);
end
%% ------------------- Step 7 ---------------------------------------------
disp('--------------------------');
disp('STEP 7');
disp('Projecting the points into the two image planes');
disp('Figure(1)');
figure(1)
hold on;
subplot(1,2,1)
axis([1 256 1 256]); % Defining axes limited with the size of our image
axis manual;
hold on;
title('Projection of 3D points camera1');
scatter(Vc1(1,:),Vc1(2,:));
hold on;
subplot(1,2,2)
axis([1 256 1 256]); % Defining axes limited with the size of our image
axis manual;
hold on;
title('Projection of 3D points camera2');
scatter(Vc2(1,:),Vc2(2,:));

%% ------------------- Step 8 ---------------------------------------------
disp('--------------------------');
disp('STEP 8');
disp('Calculating the fundamental matrix with 8 points');
F1 = fundamental(V,Vc1,Vc2);
%% ------------------- Step 9 ---------------------------------------------
disp('--------------------------');
disp('STEP 9'); 
disp('This is fundamental matrix with 8 points method and least squares');
F1
disp('This if fundamental matrix with intrinsec and extrinsec matrices');
F
disp('The error between the two matrices is error = F - F1');
error1 = abs(F1 - F)
%% ------------------- Step 10 ---------------------------------------------
disp('--------------------------');
disp('STEP 10');
disp('Calculating epipolar lines');
disp('Figure(2)');
eplines1 = epipolarLines(Vc2, F1');
eplines2 = epipolarLines(Vc1, F1);
drawEpipolarLines(eplines1, Vc1, eplines2, Vc2);
disp('Calculating epipoles');
Oc1=[0;0;0;1];
Oc2=[-1000;190;230;1];
epipole1 = projectingPoints(Oc2', M1);
epipole1 = epipole1/epipole1(3,1)
epipole2 = projectingPoints(Oc1', M2);
epipole2 = epipole2/epipole2(3,1)
%% ------------------- Step 11 ---------------------------------------------
disp('--------------------------');
disp('STEP 11');
disp('Addng noise to image plane points');
noisyVc1 = addNoise(Vc1,0, 0.5);
noisyVc2 = addNoise(Vc2,0, 0.5);
%% ------------------- Step 12 ---------------------------------------------
disp('--------------------------');
disp('STEP 12');
disp('Calculating fundamental matrix with 10 noisy points');
F2 = fundamental(V,noisyVc1,noisyVc2);
disp('This is fundamental matrix with 8 points method and 20 noisy points');
F2
disp('This if fundamental matrix with intrinsec and extrinsec matrices');
F
disp('The error between the two matrices is error = F - F2');
error2 = abs(F2 - F)
disp('Calculating epipolar lines');
eplines1n10 = epipolarLines(noisyVc2, F2');
eplines2n10 = epipolarLines(noisyVc1, F2);
drawEpipolarLines(eplines1n10, noisyVc1, eplines2n10, noisyVc2);
disp('Figure(3)');
% It is easy to check in figure 3 that there is not a unique epipole for
% each camera. Neither all points are in over the epipolar lines.
% Let's try with SVD and removing last eigenvalue
[U, S, Ve]=svd(F2);
S(3,3)=0;
F3 =U*S*Ve'
disp('The error between the two matrices is error = F - F3');
error3 = abs(F3 - F)
eplines1n10a = epipolarLines(noisyVc2, F3');
eplines2n10a = epipolarLines(noisyVc1, F3);
drawEpipolarLines(eplines1n10a, noisyVc1, eplines2n10a, noisyVc2);
disp('Figure(4)');
% Now all epipolar lines cross a unique epipole. Let's compute the error of
% this one with the one calculated without noise.
epipole1noisy1 = Ve(:,3)/Ve(3,3)
epipole2noisy1 = U(:,3)/U(3,3)
disp('The error between the epipole with and without noise is:');
errorEp1Step12 = abs(epipole1 - epipole1noisy1)
errorEp2Step12 = abs(epipole2 - epipole2noisy1)
%% ------------------- Step 13 ---------------------------------------------
disp('--------------------------');
disp('STEP 13');
noisyVc11 = addNoise(Vc1,0, 1);
noisyVc22 = addNoise(Vc2,0, 1);
disp('Calculating fundamental matrix with 20 noisy points');
F4 = fundamental(V,noisyVc11,noisyVc22);
disp('This is fundamental matrix with 8 points method and 20 noisy points');
F4
disp('This if fundamental matrix with intrinsec and extrinsec matrices');
F
disp('The error between the two matrices is error = F - F4');
error4 = abs(F4 - F)
disp('Calculating epipolar lines');
eplines11n10 = epipolarLines(noisyVc22, F4');
eplines22n10 = epipolarLines(noisyVc11, F4);
drawEpipolarLines(eplines11n10, noisyVc11, eplines22n10, noisyVc22);
disp('Figure(5)');
% It is easy to check in figure 3 that there is not a unique epipole for
% each camera. Neither all points are in over the epipolar lines.
% Let's try with SVD and removing last eigenvalue
[U1, S1, Ve1]=svd(F4);
S1(3,3)=0;
F5 =U1*S1*Ve1'
disp('The error between the two matrices is error = F - F5');
error5 = abs(F5 - F)
eplines11n10a = epipolarLines(noisyVc22, F5');
eplines22n10a = epipolarLines(noisyVc11, F5);
drawEpipolarLines(eplines11n10a, noisyVc11, eplines22n10a, noisyVc22);
disp('Figure(6)');
% Now all epipolar lines cross a unique epipole. Let's compute the error of
% this one with the one calculated without noise.
epipole1noisy2 = Ve1(:,3)/Ve1(3,3)
epipole2noisy2 = U1(:,3)/U1(3,3)
disp('The error between the epipole with and without noise is:');
errorEp1Step13 = abs(epipole1 - epipole1noisy2)
errorEp2Step13 = abs(epipole2 - epipole2noisy2)

% PART 2
%% ------------------- Step 14 ---------------------------------------------
disp('--------------------------');
disp('STEP 14');
Fsvd=svdMethod(V,Vc1,Vc2);
disp('This is fundamental matrix with 8 points method and least squares');
F1
disp('This if fundamental matrix with 8 points method and SVD');
Fsvd
disp('Calculating epipolar lines for svd method without noise');
disp('Figure(7)');
eplines111 = epipolarLines(Vc2, Fsvd');
eplines222 = epipolarLines(Vc1, Fsvd);
drawEpipolarLines(eplines111, Vc1, eplines2, Vc2);
disp('Calculating epipoles for svd method without noise');
epipole111 = projectingPoints(Oc2', M1);
epipole111 = epipole111/epipole111(3,1)
epipole222 = projectingPoints(Oc1', M2);
epipole222 = epipole222/epipole222(3,1)
%% ------------------- Step 15 ---------------------------------------------
disp('--------------------------');
disp('STEP 15');
disp('Gaussian noise, sigma = 0.5');
disp('Using the same noisy points of step 10 (in step 15)');
disp('Calculating fundamental matrix with 10 noisy points for svd method');
FsvdN1 = svdMethod(V,noisyVc1,noisyVc2);
disp('This is fundamental matrix with 8 points and least squares method, and 10 noisy points');
F2
disp('This if fundamental matrix with 8 points and svd method, and 10 noisy points');
FsvdN1
disp('The error between the two matrices is error = FsvdN1 - F2');
error6 = abs(FsvdN1 - F2)
disp('Figure(8)');
eplines1Step15N1= epipolarLines(noisyVc2, FsvdN1');
eplines2Step15N1 = epipolarLines(noisyVc1, FsvdN1);
drawEpipolarLines(eplines1Step15N1, noisyVc1, eplines2Step15N1, noisyVc2);
[Ustep15N1, Sstep15N1, Vestep15N1]=svd(FsvdN1);
Sstep15N1(3,3)=0;
Fstep15N1corrected =Ustep15N1*Sstep15N1*Vestep15N1'
eplines1Step15N1a = epipolarLines(noisyVc2, Fstep15N1corrected');
eplines2Step15N1a = epipolarLines(noisyVc1, Fstep15N1corrected);
drawEpipolarLines(eplines1Step15N1a, noisyVc1, eplines2Step15N1a, noisyVc2);
disp('Figure(9)');
% Now all epipolar lines cross a unique epipole. Let's compute the error of
% this one with the one calculated without noise.
epipole1step15N1 = Vestep15N1(:,3)/Vestep15N1(3,3)
epipole2step15N1 = Ustep15N1(:,3)/Ustep15N1(3,3)
disp('The error between the epipole with and without noise is:');
errorEp1Step15a = abs(epipole1 - epipole1noisy2)
errorEp2Step15a = abs(epipole2 - epipole2noisy2)
disp('--------------------------');
disp('Gaussian noise, sigma = 1');
disp('Using the same noisy points of step 11 (in step 15)');
disp('Calculating fundamental matrix with 20 noisy points for svd method');
FsvdN2 = svdMethod(V,noisyVc11,noisyVc22);
disp('This is fundamental matrix with 8 points and least squares method, and 10 noisy points');
F4
disp('This if fundamental matrix with 8 points and svd method, and 10 noisy points');
FsvdN2
disp('The error between the two matrices is error = FsvdN2 - F4');
error7 = abs(FsvdN2 - F4)
disp('Figure(10)');
eplines1Step15N2= epipolarLines(noisyVc22, FsvdN1');
eplines2Step15N2 = epipolarLines(noisyVc11, FsvdN1);
drawEpipolarLines(eplines1Step15N2, noisyVc11, eplines2Step15N2, noisyVc22);
[Ustep15N2, Sstep15N2, Vestep15N2]=svd(FsvdN2);
Sstep15N2(3,3)=0;
Fstep15N2corrected =Ustep15N2*Sstep15N2*Vestep15N2'
eplines1Step15N2b = epipolarLines(noisyVc22, Fstep15N2corrected');
eplines2Step15N2b = epipolarLines(noisyVc11, Fstep15N2corrected);
drawEpipolarLines(eplines1Step15N2b, noisyVc11, eplines2Step15N2b, noisyVc22);
disp('Figure(11)');
% Now all epipolar lines cross a unique epipole. Let's compute the error of
% this one with the one calculated without noise.
epipole1step15N2 = Vestep15N2(:,3)/Vestep15N2(3,3)
epipole2step15N2 = Ustep15N2(:,3)/Ustep15N2(3,3)
disp('The error between the epipole with and without noise is:');
errorEp1Step15b = abs(epipole1 - epipole1step15N2)
errorEp2Step15b = abs(epipole2 - epipole2step15N2)


%% ------------------- Step 16 ---------------------------------------------
disp('--------------------------');
disp('STEP 16');
disp('Plotting the results');
disp('---Case 1 ---');
disp('Least squares - No noise');
[mean1_1,stdDev1_1] = computeDistances(eplines1,Vc1);
[mean1_2,stdDev1_2] = computeDistances(eplines2,Vc2);
mean1 = (mean1_1 + mean1_2)/2
stdDev1 = (stdDev1_1 + stdDev1_2)/2

disp('---Case 2 ---');
disp('Least squares - Gausian noise 1');
[mean2_1,stdDev2_1] = computeDistances(eplines1n10,noisyVc1);
[mean2_2,stdDev2_2] = computeDistances(eplines2n10,noisyVc2);
mean2 = (mean2_1 + mean2_2)/2
stdDev2 = (stdDev2_1 + stdDev2_2)/2

disp('---Case 3 ---');
disp('Least squares - Gausian noise 1 - 1 epipole');
[mean3_1,stdDev3_1] = computeDistances(eplines1n10a,noisyVc1);
[mean3_2,stdDev3_2] = computeDistances(eplines2n10a,noisyVc2);
mean3 = (mean3_1 + mean3_2)/2
stdDev3 = (stdDev3_1 + stdDev3_2)/2

disp('---Case 4 ---');
disp('Least squares - Gausian noise 2');
[mean4_1,stdDev4_1] = computeDistances(eplines11n10,noisyVc11);
[mean4_2,stdDev4_2] = computeDistances(eplines22n10,noisyVc22);
mean4 = (mean4_1 + mean4_2)/2
stdDev4 = (stdDev4_1 + stdDev4_2)/2

disp('---Case 5 ---');
disp('Least squares - Gausian noise 2 - 1 epipole');
[mean5_1,stdDev5_1] = computeDistances(eplines11n10a,noisyVc11);
[mean5_2,stdDev5_2] = computeDistances(eplines22n10a,noisyVc22);
mean5 = (mean5_1 + mean5_2)/2
stdDev5 = (stdDev5_1 + stdDev5_2)/2


disp('---Case 6 ---');
disp('SVD - No noise');
[mean6_1,stdDev6_1] = computeDistances(eplines111,Vc1);
[mean6_2,stdDev6_2] = computeDistances(eplines222,Vc2);
mean6 = (mean6_1 + mean6_2)/2
stdDev6 = (stdDev6_1 + stdDev6_2)/2


disp('---Case 7 ---');
disp('SVD - Gausian noise 1');
[mean7_1,stdDev7_1] = computeDistances(eplines1Step15N1,noisyVc1);
[mean7_2,stdDev7_2] = computeDistances(eplines2Step15N1,noisyVc2);
mean7 = (mean7_1 + mean7_2)/2
stdDev7 = (stdDev7_1 + stdDev7_2)/2

disp('---Case 8 ---');
disp('SVD - Gausian noise 1 - 1 epipole');
[mean8_1,stdDev8_1] = computeDistances(eplines1Step15N1a,noisyVc1);
[mean8_2,stdDev8_2] = computeDistances(eplines2Step15N1a,noisyVc2);
mean8 = (mean8_1 + mean8_2)/2
stdDev8 = (stdDev8_1 + stdDev8_2)/2

disp('---Case 9 ---');
disp('SVD - Gausian noise 2');
[mean9_1,stdDev9_1] = computeDistances(eplines1Step15N2,noisyVc11);
[mean9_2,stdDev9_2] = computeDistances(eplines2Step15N2,noisyVc22);
mean9 = (mean9_1 + mean9_2)/2
stdDev9 = (stdDev9_1 + stdDev9_2)/2

disp('---Case 10 ---');
disp('Least squares - Gausian noise 2 - 1 epipole');
[mean10_1,stdDev10_1] = computeDistances(eplines1Step15N2b,noisyVc11);
[mean10_2,stdDev10_2] = computeDistances(eplines2Step15N2b,noisyVc22);
mean10 = (mean10_1 + mean10_2)/2
stdDev10 = (stdDev10_1 + stdDev10_2)/2

% %% Plotting 3D system
% % Plotting coordinate systems
% figure(12);
% hold on;
% grid on;
% axis square;
% % World 
% line('XData',[0 100],'LineWidth',2,'Color', 'r','SelectionHighlight', 'on');
% line('YData',[0 100],'LineWidth',2,'Color', 'g','SelectionHighlight', 'on');
% line('ZData',[0 100],'LineWidth',2,'Color', 'b','SelectionHighlight', 'on');
% 
% % Camera 1
% originC = pinv(c1Kw)*[0 0 0]';
% endxC = pinv(c1Kw)*[100 0 0]';
% endyC = pinv(c1Kw)*[0 100 0]';
% endzC = pinv(c1Kw)*[0 0 100]'
% endxC = endxC/ endxC(4,1);
% endyC = endyC/ endyC(4,1);
% endzC = endzC/ endzC(4,1);
% endC = [endxC endyC endzC];
% %hold on;
% axis equal;
% line('XData',[originC(1,1) endC(1,1)], 'YData',[originC(2,1) endC(2,1)], ...
%      'ZData',[originC(3,1) endC(3,1)],'LineWidth',2,'Color', 'r','SelectionHighlight', 'on');
% line('XData',[originC(1,1) endC(1,2)], 'YData',[originC(2,1) endC(2,2)], ...
%      'ZData',[originC(3,1) endC(3,2)],'LineWidth',2,'Color', 'g','SelectionHighlight', 'on');
% line('XData',[originC(1,1) endC(1,3)], 'YData',[originC(2,1) endC(2,3)], ...
%      'ZData',[originC(3,1) endC(3,3)],'LineWidth',2,'Color', 'b','SelectionHighlight', 'on');
%  
% % Plotting the 3D Points
% f = 80;
% hold on;
% scatter3(V(1,:), V(2,:), V(3,:), 10, 'k', 'fill'); 
% % Frame limits for Image Plane
% P1 = [-128 -128 1]';       % Coordinates of image plane
% P2 = [128 -128 1]';
% P3 = [128 128 1]';
% P4 = [-128 128 1]';
% 
% cP1 = pinv(A1)*P1;   % Coordinates of image plane limits respect to camera
% cP2 = pinv(A1)*P2;
% cP3 = pinv(A1)*P3;
% cP4 = pinv(A1)*P4;
% 
% cP1 = cP1*f;%*2;    % Adjusting to the correct focal length (right now z = 1, h = 0)
% cP2 = cP2*f;%*2;
% cP3 = cP3*f;%*2;
% cP4 = cP4*f;%*2;
% 
% cP1(4,1) = 1;   % Adjusting scaling. Positioning the plane in front to the camera
% cP2(4,1) = 1;
% cP3(4,1) = 1;
% cP4(4,1) = 1;
% 
% iP1w = pinv(M1)*(P1); % Points of the frame in world coordinates
% iP2w = pinv(M1)*(P2);
% iP3w = pinv(M1)*(P3);
% iP4w = pinv(M1)*(P4);
% iPw = [iP1w iP2w iP3w iP4w iP1w]; % We add first one to connect it with last
% plot3(iPw(1,:), iPw(2,:), iPw(3,:));%, 'b', 'fill'); 
% hold on;
% normal = cross(iP2w(1:3,:)-iP1w(1:3,:),...
%     iP4w(1:3,:)-iP1w(1:3,:));% Plotting the plane 
% A = normal(1); B = normal(2); C = normal(3);
% D = -dot(normal,iP2w(1:3,:));
% xx = linspace(iP1w(1,1),iP2w(1,1));
% zz = linspace(iP1w(3,1),iP4w(3,1));
% hold on;
% [X,Z] = meshgrid(xx,zz);
% 
% Y = (A * X + C * Z + D)/ (-B);
% hold on;
% S1 = surf(X,Y,Z ,'FaceAlpha',0.3, 'EdgeColor','none','FaceColor',[0 0 1]);
% 
% % % Plotting projections over the image plane
% % 
% % iPc = pinv(A1)*Vc1;
% % iPc = iPc*f;         % We set real focal length (f = 80), it was 1.
% % iPc(4,:)=1;         % We set homogeneous coordinate to 1 (to normalize)
% % iPw = pinv(M1)*iPc
% % scatter3(iPw(1,:), iPw(2,:), iPw(3,:), 10, 'r', 'fill');
% % %Points = Points';
% 
% % % Plotting ray trace
% % for i=1:size(V,2)
% %     line('XData',[iPw(1,i) V(1,i)], 'YData',[iPw(2,i) V(2,i)], ...
% %      'ZData',[iPw(3,i) V(3,i)],'LineWidth',1,'Color', 'k');
% % end
% % 
% % % Plotting extension of lines
% % newPoint = zeros(3,1);
% % size(Vc);
% % for i=1:size(V,2)
% %     vec = [iPw(1,i)-V(1,i), iPw(2,i)-V(2,i), iPw(3,i)-V(3,i)]'
% %     newPoint = iPw(1:3,i) + 0.4*vec
% %     line('XData',[iPw(1,i) newPoint(1,1)], 'YData',[iPw(2,i) newPoint(2,1)], ...
% %      'ZData',[iPw(3,i) newPoint(3,1)],'LineWidth',1,'Color', 'k');
% % end