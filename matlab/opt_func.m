function F = opt_func(x, Xp, q, in_param)

% OPT FUNC
% OUTPUT 
%       F: objective function
%
% INPUT
%       x: parameters to optimize
%       Xp: 3D coordinate of reference points in the reference coordinate system
%       q: 2D coordinate of mirrored reference points on the image plane.
%       in_param: intrinsic camera parameters
% 

num_of_mirror_pose = size(q,1);

R_x = x(1,1);
R_y = x(2,1);
R_z = x(3,1);
R = sub_matrix_from_angle(R_x, R_y, R_z);
T = x(4:6,1);

for i = 1:num_of_mirror_pose
  n_theta = x(2*(i-1)+6+1,1);
  n_phi = x(2*(i-1)+6+2,1);
  n = sub_vector_from_angle(n_theta, n_phi);

  d = x(i+6+2*num_of_mirror_pose,1);

  temp_q = q{i,1};

  reps = (sub_reproj_core(Xp, temp_q, R, T, n, d, in_param))';

  if i == 1
    F = reps(:);
  else
    F = [F; reps(:)];
  end
end
