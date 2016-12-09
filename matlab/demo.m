% DEMO Illustrates how to use the mirror calibration algorithm.

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
%

clear all;
close all;

model = load('../data/model.txt');  
%model = load('../data/model_3p.txt');  % in case of solving p3p

input{1,1} = load('../data/input1.txt'); % or input1_3p.txt (p3p)
input{2,1} = load('../data/input2.txt'); % or input2_3p.txt (p3p)
input{3,1} = load('../data/input3.txt'); % or input3_3p.txt (p3p)
%input{4,1} = load('../data/input4.txt'); % or input4_3p.txt (p3p)
%input{5,1} = load('../data/input5.txt'); % or input5_3p.txt (p3p)

in_param = load('../data/camera.txt');

num_of_mirror_pose = size(input, 1);

% calibration step．

% linear solution with proposed method
[R_t, T_t, n_t, d_t, rep_t] = tnm(model, input, in_param);

% result

fprintf('\n');
fprintf('Average reprojection error by TNM : %f pixel.\n', rep_t);
fprintf('\n');

fprintf('==== Parameters by TNM ====\n');
R = R_t
T = T_t
n1 = n_t(1,:)'
n2 = n_t(2,:)'
n3 = n_t(3,:)'
d1 = d_t(1,:)
d2 = d_t(2,:)
d3 = d_t(3,:)

% display the result

az = 10;
el = -56;

hold on;

grid on;
view(az, el);

daspect([1 1 1]);

offset = 10;

% plot camera position
R_c = [1 0 0; 0 1 0; 0 0 1];
T_c = [0; 0; 0];  

sub_plot_axis(R_c, T_c);
text(T_c(1,1) + offset, T_c(2,1) + offset, T_c(3,1) + offset, 'Camera');

% plot parameters by TNM
sub_plot_axis(R_t, T_t);

% plot reference points estimated by TNM
Cp_t = (R_t * model' + repmat(T_t, 1, size(model,1)))';
sub_plot_plane(Cp_t, 'r', 3, 1);
text(Cp_t(1,1) + offset, Cp_t(1,2) + offset, Cp_t(1,3) + offset, ...
    'Reference Points Estimated by TNM');

% plot mirrored reference points
Cp_t_h = Cp_t;
Cp_t_h(:,4) = 1;
for i = 1:num_of_mirror_pose
  n = n_t(i,:)';
  d = d_t(i,:);
  % Householder transformation
  H = [ eye(3) - 2 * n * n', -2 * d * n; zeros(1,3), 1];
  mirrored_Cp{i,1} = (H * Cp_t_h')';

  temp_text = sprintf('mirrored referece points %d (by TNM)', i);

  text(mirrored_Cp{i,1}(1,1) + offset, ...
      mirrored_Cp{i,1}(1,2) + offset,...
      mirrored_Cp{i,1}(1,3) + offset, ...
      temp_text);

  sub_plot_plane(mirrored_Cp{i,1}, 'g', 3, 1);
end

% plot mirror position estimated by TNM
for i = 1:num_of_mirror_pose
  m_pose = (mirrored_Cp{i,1}(:,1:3) + Cp_t) / 2;
  temp = m_pose(3,:);
  vec1 = m_pose(2,:) - m_pose(1,:);
  vec2 = m_pose(3,:) - m_pose(1,:);
  m_pose(3,:) = m_pose(1,:) + vec1 + vec2;
  m_pose(4,:) = temp;

  temp_text = sprintf('mirror %d (by TNM)', i);
  
  text(m_pose(1,1) + offset, m_pose(1,2) + offset, m_pose(1,3) + offset, ...
    temp_text);
  
  sub_plot_plane(m_pose, 'c', 3, 1);
end

hold off;
