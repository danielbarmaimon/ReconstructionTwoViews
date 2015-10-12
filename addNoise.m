function [ noisyPoints ] = addNoise(points, mean, sigma)
% This function will return the points introduced with a gaussian noise
% over them 
% Inputs: points = Normalized points in images plane
%         mean   = Mean value for the gaussian function to generate the noise
%         sigma  = Sigma value for the gaussian function to generate the noise
% Outputs: noisyPoints = original image plane points with the noise added

noise = normrnd(mean, sigma,[2,size(points,2)]); % 2*sigma = 1 (we want 95% of data)
noise = [noise; zeros(1,size(points,2))];
noisyPoints = points + noise;
end

