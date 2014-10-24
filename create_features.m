%createFeatures.m creates featrues in the world to be used by
%the cameras on the quadrotor
%Copyright (C) 2014  Ke Sun
%
%This program is free software; you can redistribute it and/or
%modify it under the terms of the GNU General Public License
%as published by the Free Software Foundation; either version 2
%of the License, or (at your option) any later version.
%
%This program is distributed in the hope that it will be useful,
%but WITHOUT ANY WARRANTY; without even the implied warranty of
%MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%GNU General Public License for more details.
%
%You should have received a copy of the GNU General Public License
%along with this program; if not, write to the Free Software
%Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

clc;
clear;
close all;

% Number of features
fnum = 5000;

% Range of the world
dmax = 10;

% Create features in a 3d box
features = bsxfun(@minus, 2*dmax * rand(3, fnum), dmax*ones(3, 1));

% Delete the features near the trajectory of the quadrotor
badFeatureMarker = abs(features(1, :)) <= 2 & abs(features(2, :)) <= 2 & abs(features(3, :)) <= 2;
features(:, badFeatureMarker) = [];


%% Assign colors to the features
cmap = colormap;
maxZ = max(features(3, :));
minZ = min(features(3, :));
mapping = [minZ 1; maxZ 1] \ [1 size(cmap, 1)]';
colorIdx = uint8(mapping(1) * features(3, :) + mapping(2));
colors = cmap(colorIdx, :)';

%% Save the random feature locations to a file
save('./data/features.mat', 'features', 'colors');


%% Show the features
figure;
scatter3(features(1, :), features(2, :), features(3, :), 25, colors', 'filled');
xlabel('x'); ylabel('y'); zlabel('z');
grid on; axis equal;
































