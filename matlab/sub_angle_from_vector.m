function [theta phi] = computeAngleFromVector(vec)

% SUB ANGLE FROM VECTOR
% OUTPUT 
%       theta, phi: angles in the spherical polar coordinate system (r = 1)
%
% INPUT
%       vec: vector
% 

theta = asin(vec(3,1));

phi1 = asin(vec(2,1) / cos(theta));
phi2 = acos(vec(1,1) / cos(theta));
phi = (phi1 + phi2) / 2;
