function Cp = sub_tnm_orth(Cp_candidates)

% SUB TNM ORTH
% OUTPUT
%       Cp: 3D coordinate of mirrored reference points.
%
% INPUT
%       Cp_candidates: candidates of 3D coordinates of mirrored
%                      reference points.
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


% the size of row represents the number of mirror poses.
num_of_mirror_pose = size(Cp_candidates, 1);

num_of_combination = 1;

for i = 1:num_of_mirror_pose

  multiplicity = size(Cp_candidates{i,1}, 2);
  multiplicities(i,1) = multiplicity;
  
  num_of_combination = num_of_combination * multiplicity;
  
end

for i = 1:num_of_combination

  temp_index = i - 1;
  
  for j1 = 1:num_of_mirror_pose - 1
    den = 1;
    for j2 = j1 + 1:num_of_mirror_pose
      den = den * multiplicities(j2,:);
    end

    index_list(i,j1) = floor(temp_index/den) + 1;
    temp_index = mod(temp_index,den);
  end
  
  index_list(i,num_of_mirror_pose) = temp_index + 1;
end

temp_index = 1;
rho_min = 1.0e+20;

for i = 1:num_of_combination

  rho = 0;
  
  for j = 1:num_of_mirror_pose
    temp_Cp_candidates{j,1}{1,1} = Cp_candidates{j,1}{1,index_list(i,j)};
  end

  for j1 = 1:num_of_mirror_pose - 1
    for j2 = j1 + 1:num_of_mirror_pose

      Cp_1 = temp_Cp_candidates{j1,1}{1,1};
      Cp_2 = temp_Cp_candidates{j2,1}{1,1};
      
      Q = Cp_2 - Cp_1;
      
      M = Q' * Q;
      
      [V D] = eig(M);
	
      rho = rho + D(1,1) / (D(1,1) + D(2,2) + D(3,3));
      
    end
  end

  if rho < rho_min

    rho_min = rho;
    Cp = temp_Cp_candidates;
  
  end
end
