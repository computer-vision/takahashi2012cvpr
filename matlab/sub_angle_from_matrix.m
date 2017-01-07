function [x_angle y_angle z_angle] = sub_angle_from_matrix(R)

% SUB ANGLE FROM MATRIX
% OUTPUT 
%       x_angle, y_angle, z_angle: rotation angle for each axis
%
% INPUT
%       R: Rotation matrix
% 

threshold = 1.0e-10;

% in the case of R(3,1) = -sin(y) = -1
% sin(y) = 1 
if abs(R(3,1) + 1.0) < threshold

  x_angle = 0;
  y_angle = pi / 2;
  z_angle = atan2(-1 * R(1,2), R(2,2));

elseif abs(R(3,1) - 1.0) < threshold % R(3,1) = -sin(y) = 1

  x_angle = 0;
  y_angle = -pi / 2;
  z_angle = atan2(-1 * R(1,2), R(2,2));
  
else

  x_angle = atan2(R(3,2), R(3,3));
  y_angle = asin(-1 * R(3,1));
  z_angle = atan2(R(2,1), R(1,1));

end

