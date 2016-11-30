function y = sub_plot_axis(R, T)

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


%config
msize  = 5;
lwidth = 3;

cmap = zeros(3,3);
cmap(1,:) = [1,0,0];
cmap(2,:) = [0,1,0];
cmap(3,:) = [0,0,1];

scale = 30; 
seed = [scale,0,0;0,scale,0;0,0,scale];

Axis = R * seed + repmat(T,1,3);

AxisX = Axis(:,1);
AxisY = Axis(:,2);
AxisZ = Axis(:,3);

pLocusX = [T,AxisX];
pLocusY = [T,AxisY];
pLocusZ = [T,AxisZ];

hold on

plot3(pLocusX(1,1:2),pLocusX(2,1:2),pLocusX(3,1:2),'-*',...
    'MarkerSize',msize,...
    'Color',cmap(1,:),...
    'LineWidth',lwidth);

plot3(pLocusY(1,1:2),pLocusY(2,1:2),pLocusY(3,1:2),'-*',...
    'MarkerSize',msize,...
    'Color',cmap(2,:),...
    'LineWidth',lwidth);

plot3(pLocusZ(1,1:2),pLocusZ(2,1:2),pLocusZ(3,1:2),'-*',...
    'MarkerSize',msize,...
    'Color',cmap(3,:),...
    'LineWidth',lwidth);

hold off
  
