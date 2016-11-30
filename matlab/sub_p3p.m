function [R T Cp] = sub_p3p(Xp, q_h, in_param, inv_axis)

% SUB P3P
% OUTPUT
%       R: Rotation matrix.
%       T: Translation vector.
%       Cp: 3D coordinate of mirrored reference points.
%
% INPUT
%       Xp: 3D coordinate of reference points in the reference coordinate system.
%       q_h: 2D coordinate mirrored reference points on the image
%            plane (homogeneous coordinates).
%       in_param: intrinsic camera parameters.
%       aixs_inv: inverse the sign of one axis(change the left-right
%                  hand coordinate system 0:not inv 1:inv)
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

size_q_h = size(q_h, 1);
size_Xp = size(Xp, 1);
size_q = size_q_h;

q_h_t = q_h';
Xp_t = Xp';

% check whether the size of matrix is 3 or not
if size_q ~= 3
  error('The number of points is not 3.');
  exit;
end

% compute the orientation vector.
ovec = inv(in_param) * q_h_t;

% normalize the orientation vector.
for i = 1:size_q
  novec(:,i) = ovec(:,i) / norm(ovec(:,i));
end

% inverse the sing of y axis.
yInv = [1 0 0; 0 -1 0; 0 0 1];
if inv_axis
  novec = yInv * novec;
end

% compute the cos between the each pair of orientation vectors.

cosa = novec(:,2)' * novec(:,3);
cosb = novec(:,3)' * novec(:,1);
cosc = novec(:,1)' * novec(:,2);

% compute the distance of each pair of 3D model points.

a = norm(Xp_t(:,2) - Xp_t(:,3));
b = norm(Xp_t(:,3) - Xp_t(:,1));
c = norm(Xp_t(:,1) - Xp_t(:,2));

% main algorithm by Fischler and Bolles

D4 = 4 * b^2 * c^2 * cosa^2 - (a^2 - b^2 - c^2)^2;

D3 = -4 * c^2 * (a^2 + b^2 - c^2) * cosa * cosb ...
    - 8 * b^2 * c^2 * cosa^2 * cosc ...
    + 4 * (a^2 - b^2 - c^2) * (a^2 - b^2)* cosc;

D2 = 4 * c^2 * (a^2 - c^2) * cosb^2 ...
    + 8 * c^2 * (a^2 + b^2) * cosa * cosb * cosc ...
    + 4 * c^2 * (b^2 - c^2) * cosa^2 ...
    - 2 * (a^2 - b^2 - c^2) * (a^2 - b^2 + c^2) ...
    - 4 * (a^2 - b^2)^2 * cosc^2;

D1 = -8 * a^2 * c^2 * cosb^2 * cosc ...
    - 4 * c^2 * (b^2 - c^2) * cosa * cosb ...
    - 4 * a^2 * c^2 * cosa * cosb ...
    + 4 * (a^2 - b^2) * (a^2 - b^2 + c^2) * cosc;

D0 = 4 * a^2 * c^2 * cosb^2 - (a^2 - b^2 + c^2)^2;

u = roots([D4 D3 D2 D1 D0]);

% eliminate complex numbers.
ru = u(imag(u)==0);

% size of ru represents the mulitiplicity of solution.
size_ru = size(ru,1);

v = zeros(size_ru,1);
s1 = zeros(size_ru,1);
s2 = zeros(size_ru,1);
s3 = zeros(size_ru,1);

for i = 1:size_ru

  temp1 = - 1 * ((a^2 - b^2 - c^2) * ru(i)^2 ...
		 + 2 * (b^2 - a^2) * cosc * ru(i) ...
		 + a^2 - b^2 + c^2);

  temp2 = 2 * c^2 * (cosa * ru(i) - cosb);

  v(i) = temp1/temp2;

end

for i = 1:size_ru

    temp = a^2 / (ru(i)^2 + v(i)^2 - 2 * ru(i) * v(i) * cosa);
    s1(i) = sqrt(temp);

end

s2 = s1 .* ru;
s3 = s1 .* v;

for i = 1:size_ru

  p1 = s1(i) .* novec(:,1);
  p2 = s2(i) .* novec(:,2);
  p3 = s3(i) .* novec(:,3);

  temp_Cp = [p1'; p2'; p3'];
    
  [temp_R temp_T] = absolute_orientation(Xp, temp_Cp);
    
  R{1,i} = temp_R;
  T{1,i} = temp_T;
  Cp{1,i} = temp_Cp;

end

function [R T] = absolute_orientation(P1, P2)
% compute the extrinsic parameters, R and T, which satisfy following equation.
% P2 = R * P1 + T; 
 
aveA = mean(P1)';
aveB = mean(P2)';

covA = P1' - repmat(aveA, 1, size(P1', 2));
covB = P2' - repmat(aveB, 1, size(P2', 2));

M = covA * covB';

[U S V] = svd(M);
R = V * U';

% assume that R is the right hand coordinate system.
if det(R) < 0
  R(:,3) = -1 * R(:,3);
end

T = aveB - R * aveA;

% for recalculation: this e value should be almost zero.
e = norm(P2' - R * P1' - repmat(T, 1, size(P1',2)));

