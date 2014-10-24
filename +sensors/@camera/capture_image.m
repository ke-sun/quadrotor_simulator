%camera.m defines the camera class
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


function [ feature_in_img, tags ] = capture_image( obj, curr_state, features )
%CAPTURE_IMAGE simulates the behavior of a camera
%
%----------------------inputs------------------------
% features          : 3-by-m matrix, 3d coordinates of all the features in
%                     the world
% curr_sate         : current state of the quadrotor
%
%---------------------outputs------------------------
% feature_in_img      : 2-by-n matrix, image coordinates of the captured
%                       features with noise
% tags                : 1-by-n vector, tags of the captured features in the
%                       world
%
% Author: Ke Sun
% Date  : 10/23/2014


% Convert the coordinates of the features to homogeneous coordinates
features_h = [features; ones(1, size(features, 2))];

% Compute the extrinsic matrix
imu_R = curr_state.R;
imu_t = curr_state.position;
cam_R = imu_R * obj.R;
cam_t = imu_t + imu_R*obj.t;
extrinsic_mat = [cam_R' -cam_R'*cam_t];


% Project the features onto the image plane
image_coordinates = extrinsic_mat * features_h;
feature_depth = image_coordinates(3, :);
image_coordinates = bsxfun(@times, image_coordinates, 1./feature_depth);


% Add some radical distortion
% r = sqrt(imgCoordinate(1, :).^2 + imgCoordinate(2, :).^2);
% r4 = [r; r.^2; r.^3; r.^4];
% Lr = (r4'*obj.distortionK)' + 1;
% invLr = 1./Lr;
% invLr(invLr<0.8) = inf;
% imgCoordinate(1, :) = invLr .* imgCoordinate(1, :);
% imgCoordinate(2, :) = invLr .* imgCoordinate(2, :);

% Transform the image coordinates to pixel coordinates
pixel_coordinate = obj.K * image_coordinates;
pixel_coordinate = pixel_coordinate(1:2, :);


% Find the features that can be actually captured by the camera
on_img_feature_flag = pixel_coordinate(1, :) >= 1 & pixel_coordinate(1, :) <= obj.img_size(2) & ...
                      pixel_coordinate(2, :) >= 1 & pixel_coordinate(2, :) <= obj.img_size(1) & ...
                      feature_depth > 0.1;
pixel_coordinate = pixel_coordinate(:, on_img_feature_flag);
tags = find(on_img_feature_flag);

% Add pixel noise
[U_pixel, S_pixel, V_pixel] = svd(obj.P);
pixel_coordinate  = pixel_coordinate + U_pixel*sqrt(S_pixel)*V_pixel'*randn(size(pixel_coordinate));

% Remove the features that are out of the image after adding noise
on_img_feature_flag = pixel_coordinate(1, :) >= 1 & pixel_coordinate(1, :) <= obj.img_size(2) & ...
                      pixel_coordinate(2, :) >= 1 & pixel_coordinate(2, :) <= obj.img_size(1);

pixel_coordinate = pixel_coordinate(:, on_img_feature_flag);
tags = tags(on_img_feature_flag);
feature_in_img = pixel_coordinate;

end

