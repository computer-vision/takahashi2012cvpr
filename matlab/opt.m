function [R_opt, T_opt, n_opt, d_opt, rep_opt] = ...
    opt(Xp, q, in_param, R, T, n, d)
% OPT
% OUTPUT 
%       R_opt: optimized Rotation matrix.
%       T_opt: optimized Translation vector.
%       n_opt: optimized mirror normal vector.
%       d_opt: optimized distance betweeen camera and mirror.
%       rep_opt: optimized reprojection error when using estimated parameters
%
% INPUT
%       Xp: 3D coordinate of reference points in the reference coordinate system
%       q: 2D coordinate of mirrored reference points on the image plane.
%       in_param: intrinsic camera parameters
% 

num_of_mirror_pose = size(q, 1);

[R_x R_y R_z] = sub_angle_from_matrix(R);
init_value = [R_x; R_y; R_z; T];

for i = 1:num_of_mirror_pose
  [n_theta n_phi] = sub_angle_from_vector(n(i,:)');
  init_value = [init_value; n_theta; n_phi];
end

init_value = [init_value; d];
opt = optimset('Display', 'off');
opt = optimset(opt, 'Algorithm', 'levenberg-marquardt');

[x, fval, exitflag, output] = ...
    fsolve(@opt_func, init_value, opt, Xp, q, in_param);

R_opt = sub_matrix_from_angle(x(1,1),x(2,1),x(3,1));
T_opt = x(4:6,1);

sum_error_opt = 0;

for i = 1:num_of_mirror_pose
  n_opt_theta = x(2*(i-1)+6+1,1);
  n_opt_phi = x(2*(i-1)+6+2,1);
  n_opt(i,:) = (sub_vector_from_angle(n_opt_theta, n_opt_phi))';
  d_opt(i,1) = x(i+6+2*num_of_mirror_pose,1);
end
  
rep_opt = sub_reproj(Xp, q, R_opt, T_opt, n_opt, d_opt, in_param);
