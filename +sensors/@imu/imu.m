%imu.m defines the imu class
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

classdef imu < handle

    properties (SetAccess = private)

        R         % Configuration of the IMU (body frame)
        t         % It is constricted to be set to eye(3) and zeros(3, 1)

    end

    properties

        linear_acc_bias    % Bias for acceleration
        angular_vel_bias   % Bias for the angular velocity

        linear_acc_P       % Noise covariance matrix for acceleration measurement
        angular_vel_P      % Noise covariance matrix for angular velocity measurement

    end

    methods

        function obj = imu( imu_settings )

            obj.R = eye(3);
            obj.t = zeros(3, 1);

            obj.linear_acc_bias  = imu_settings.linear_acc_bias;
            obj.angular_vel_bias = imu_settings.angular_vel_bias;
            obj.linear_acc_P     = imu_settings.linear_acc_P;
            obj.angular_vel_P    = imu_settings.angular_vel_P;

        end


        function [ ] = delete( obj )
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % @brief: add noise to the data
        %
        %       Add some noise and bias to the real linear acceleration
        %       and angular velocity.
        %
        % @param  curr_state          : current state of the quadrotor
        % @return measured_linear_acc : linear acceleration with Gaussian noise
        % @return measured_angular_vel: angular velocity with Gaussian noise
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [ measured_linear_acc, measured_angular_vel ] = imuMeasure( obj, curr_state )

    end

end

