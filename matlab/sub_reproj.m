function rep = sub_reproj(Xp, q, R, T, n, d, in_param)

% SUB REPROJ
% 
% OUTPUT   
%       rep: the average of reprojection error per point
%
% INPUT
%       Xp: 3D coordinate of reference points in the reference coordinate system
%       q: 2D coordinate mirrored reference points on the image plane.
%       R: Rotation matrix.
%       T: Translation vector.
%       n: mirror normal vector.
%       d: distance betweeen camera and mirror.
%       in_param: intrinsic camera parameters.
%

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


num_of_point = size(Xp, 1);
num_of_mirror_pose = size(q,1);

sum_rep = 0;

for i = 1:num_of_mirror_pose

  reps = sub_reproj_core(Xp, q{i,1}, R, T, n(i,:)', d(i,1), in_param);

  sum_reps = 0;
  for j = 1:num_of_point
    sum_reps = sum_reps + norm(reps(j,:));
  end
  
  sum_rep = sum_rep + sum_reps / num_of_point;

end

rep = sum_rep / num_of_mirror_pose;
