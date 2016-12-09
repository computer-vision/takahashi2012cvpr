function [R T n d rep] = tnm(Xp, q, in_param)
% TNM
% OUTPUT
%     R: Rotation Matrix.
%     T: Translation vector.
%     n: normal vector of mirror plane.
%     d: distance between camera and mirror.
%     rep: the average of reprojection error per point
%
% INPUT
%     Xp: 3D coordinate of reference points in the reference coordinate system.
%     q:2D projectoins of three reference points obeserved via
%       three mirrors.
%     in_param: intrinsic camera parameters.

% For further information, please visit our web page.
%   http://vision.kuee.kyoto-u.ac.jp/~nob/proj/mirror
%
% Copyright (c) 2012, Kosuke Takahashi, Shohei Nobuhara and Takashi Matsuyama
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%    * Redistributions of source code must retain the above copyright notice,
%      this list of conditions and the following disclaimer.
%    * Redistributions in binary form must reproduce the above copyright
%      notice, this list of conditions and the following disclaimer in the
%      documentation and/or other materials provided with the distribution.
%    * Neither the name of the Graduate School of Informatics, Kyoto
%      University, Japan nor the names of its contributors may be used to
%      endorse or promote products derived from this software without specific
%      prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.

q_h = q;

num_of_mirror_pose = size(q_h, 1);

for i = 1:num_of_mirror_pose
  % homogeneous coordinate system.
  q_h{i,1}(:,3) = 1;

  num_of_points = size(q_h{i,1}, 1);

  % compute the 3D coordinates of mirrored reference points from thier projections.
  if num_of_points == 3
    [temp_R temp_T Cp_candidates{i,1}] = sub_p3p(Xp, q_h{i,1}, in_param, 0);
  else
    [temp_R temp_T Cp{i,1}{1,1}] = efficient_pnp_gauss(Xp, q_h{i,1}, in_param);
  end
end
  
if num_of_points == 3
  % select one combination of solution from p3p problem.
  Cp = sub_tnm_orth(Cp_candidates);
end
  
% compute the extrinsic camera parameter with proposed method.
[R T n d] = sub_tnm_rt(Xp, Cp);

% compute the reprojection error per pixel
rep = sub_reproj(Xp, q, R, T, n, d, in_param);

