function vec = sub_vector_from_angle(theta, phi)

% SUB VECTOR FROM ANGLE
% OUTPUT 
%       vec: vector
%
% INPUT
%       theta, phi: angles in the sphirical polar coordinate system (r = 1)
% 

vec(1,1) = cos(theta)*cos(phi);
vec(2,1) = cos(theta)*sin(phi);
vec(3,1) = sin(theta);
