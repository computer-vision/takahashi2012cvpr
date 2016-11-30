function reps = sub_reproj_core(Xp, q, R, T, n, d, in_param)

% SUB REPROJ CORE
% 
% OUTPUT   
%       reps: the matrix of repojection error for Np point.
%            (size: Np * 2, ground truth - esitmated 2D points)
%
% INPUT
%       Xp: 3D coordinate of reference points in the reference
%           coordinate system.
%       q: 2D coordinate mirrored reference points on the image plane.
%       R: Rotation matrix.(3,3)
%       T: Translation vector.(3,1)
%       n: mirror normal vector.(3,1)
%       d: distance betweeen camera and mirror.(1,1)
%       in_param: intrinsic camera parameters.(3,3)
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


num_of_point = size(Xp,1);

% householder transformation matrix
H = [eye(3) - 2 * n * n', -2 * d * n; zeros(1,3), 1];

% reference points computed with estimated parameters.
temp_Cp_h = (R * Xp' + repmat(T, 1, num_of_point));
temp_Cp_h(4,:) = 1;

Cp_h = H * temp_Cp_h;

% project the reference points to the image plane
temp_q = in_param * Cp_h(1:3,:);

for i = 1:size(temp_q,2)
  estimated_q_h(i,:) = (temp_q(:,i) / temp_q(3,i))';
end

reps = q - estimated_q_h(:,1:2);
