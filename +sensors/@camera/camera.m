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

classdef camera < handle

    properties

        R         % Rotation and translation relative to the IMU frame
        t         %(the IMU frame is always regarded as the body frame)

        K         % intrinsic matrix of the camera

        k         % distortion coefficients of the camera
        p

        img_size  % size of the images captured by the camera
        P         % Noise covariance matrix

    end

    methods

        function obj = camera( camSettings )

            obj.R        = camSettings.R;
            obj.t        = camSettings.t;
            obj.K        = camSettings.K;
            obj.k        = camSettings.k;
            obj.p        = camSettings.p;
            obj.img_size = camSettings.img_size;
            obj.P        = camSettings.P;

        end


        function [ ] = delete( obj )
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % @brief: captures images
        %
        %       Captures features in the world with the associated tags
        %
        % @param  curr_state          : current state of the quadrotor
        % @return measured_linear_acc : linear acceleration with Gaussian noise
        % @return measured_angular_vel: angular velocity with Gaussian noise
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [ features_in_img, tags ] = capture_image( obj, curr_state, features )

    end

end

