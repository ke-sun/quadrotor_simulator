%imuMeasure.m adds Gaussian noise and bias to the actual
%linear acceleration and angular velocity.
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


function [ measured_linear_acc, measured_angular_vel ] = imuMeasure( obj, curr_state )
%IMUMEASRUE simulates the behavior of an IMU by adding noise and bias to the
%linear acceleration and angular velocity
%
% Author: Ke Sun
% Date  : 10/23/2014


% Gravity acceleration
g = 9.80665;

imu_R = curr_state.R;
imu_t = curr_state.position;

linear_acc  = curr_state.linear_acc;
angular_vel = curr_state.angular_vel;

[U_linear_acc,  S_linear_acc,  V_linear_acc]  = svd(obj.linear_acc_P);
[U_angular_vel, S_angular_vel, V_angular_vel] = svd(obj.angular_vel_P);

% Add bias and noise to the linear acceleration
measured_linear_acc = linear_acc + imu_R'*[0 0 g]' + obj.linear_acc_bias + U_linear_acc*sqrt(S_linear_acc)*V_linear_acc'*randn(3, 1);

% Add bias and noise to the angular velocity
measured_angular_vel = angular_vel + obj.angular_vel_bias + U_angular_vel*sqrt(S_angular_vel)*V_angular_vel*randn(3, 1);


end

