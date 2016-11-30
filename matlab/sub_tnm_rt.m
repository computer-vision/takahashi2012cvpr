function [R T n d] = sub_tnm_rt(Xp, Cp)

% SUB TNM RT
% 
% OUTPUT   
%       R: Rotation matrix.
%       T: Translation vector.
%       n: Normal vector of the mirror plane
%       d: Distance betweeen camera and mirror.
%
% INPUT
%       Xp: 3D coordinate of reference points in the reference coordinate system.
%       Cp: 3D coordinate of mirrored reference points in the camera
%           coordinate system.
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


num_of_mirror_pose = size(Cp, 1);

rotation_constraint = 1;

% compute the axis vector m for each pair of mirror pose.
temp_index = 1;

for i1 = 1:num_of_mirror_pose - 1
  for i2 = i1 + 1:num_of_mirror_pose

    Cp_1 = Cp{i1,1}{1,1};
    Cp_2 = Cp{i2,1}{1,1};

    % compute the Q matrix
    Q = (Cp_2 - Cp_1);

    % compute the M matrix
    M = Q' * Q;

    % compute the eigen vector of M
    [V D] = eig(M);

    % compute the m vector as a eigen vector for smallest eigen value.
    m = V(:,1);
	
    m_all(:,temp_index) = [i1; i2; m];
    temp_index = temp_index + 1;
  end
end


for i = 1:num_of_mirror_pose

  % select the column related to the ith mirror pose.
  log_index_seed = (m_all(1,:) == i | m_all(2,:) == i);
  log_index = repmat(log_index_seed, 3, 1);

  m_all_sub = m_all(3:5,:);
  
  ms = m_all_sub(log_index);

  % compute the S matrix.
  S = reshape(ms, 3, num_of_mirror_pose - 1)';

  % compute the eigen value of S
  [V D] = eig(S'*S);

  % compute the normal vector as a eigen vector for smallest eigen value.  
  normal = V(:,1);

  if normal(3,:) > 0
    normal = -1 * normal;
  end
  
  n(i,:) = normal';

end

LHS = makeLHS(Xp, n);
RHS = makeRHS(Cp, n);

X = pinv(LHS) * RHS;

T = X(1:3,1);
d = X(3+1:3+num_of_mirror_pose,1);
r1 = X(3+num_of_mirror_pose+1:3+num_of_mirror_pose+3,1);
r2 = X(3+num_of_mirror_pose+4:3+num_of_mirror_pose+6,1);

if rotation_constraint

  % normalize the each colomun of rotation matrix.
  r1 = r1 / norm(r1);
  r2 = r2 / norm(r2);
  r3 = cross(r1, r2);
  r3 = r3 /norm(r3);
  
  Q = [r1 r2 r3];
  [U S V] = svd(Q);
  R = U * V';
  
else

  r3 = cross(r1,r2);
  R = [r1 r2 r3];

end


function LHS = makeLHS(Xp, n)

num_of_mirror_pose = size(n,1);
num_of_point = size(Xp,1);

for i=1:num_of_mirror_pose
  LHS_normal = zeros(3,num_of_mirror_pose);
  for j=1:num_of_point
    
    n_sub = n(i,:)';
    LHS_normal(1:3,i) = 2*n_sub;
    
    LHS_sub = [eye(3), LHS_normal,...
	  Xp(j,1)*eye(3),...
	  Xp(j,2)*eye(3)];

    if i == 1 & j == 1
      LHS = LHS_sub;
    else
      LHS = [LHS;LHS_sub];
    end
  end
end


function RHS = makeRHS(Cp, n)

num_of_mirror_pose = size(n,1);
num_of_point = size(Cp{1,1}{1,1},1);

for i=1:num_of_mirror_pose
  for j=1:num_of_point

    n_sub = n(i,:)';
    Cp_sub = Cp{i,1}{1,1};
    
    b = -2 * n_sub * Cp_sub(j,:) * n_sub + Cp_sub(j,:)';
    
    if i == 1 & j == 1
      RHS = b;
    else
      RHS = [RHS;b];
    end
  end
end




