%control.m implements the position control algorithm
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


function [ ] = control( obj, src, event_data )
%CONTROL is a position controller
%
% Author: Ke Sun
% Date  : 10/23/2014 (MM/DD/YYYY)


g = 9.80665;

% Desired position, linear velocity and acceleration, and yaw angle
desired_position   = event_data.desired_position;
desired_linear_vel = event_data.desired_linear_vel;
desired_linear_acc = event_data.desired_linear_acc;
desired_yaw        = event_data.desired_yaw;


% Current postion, linear velocity and acceleration
position  = event_data.position;
linear_vel = event_data.linear_vel;
linear_acc = event_data.linear_acc;


% Compute the command acceleration vector
linear_acc_command = desired_linear_acc + obj.Kd*(desired_linear_vel-linear_vel) + obj.Kp*(desired_position-position);


% Compute the thrust
thrust    = obj.mass * (g+linear_acc_command(3));
% absThrust = min([obj.maxAbsThrust, abs(thrust)], [], 2);
% thrust    = sign(thrust)*absThrust;


% Compute the desired roll and pitch
desired_roll  = 1/g * (linear_acc_command(1)*sin(desired_yaw) - linear_acc_command(2)*cos(desired_yaw));
desired_pitch = 1/g * (linear_acc_command(1)*cos(desired_yaw) + linear_acc_command(2)*sin(desired_yaw));


% Filling the output
event_data.desired_roll  = desired_roll;
event_data.desired_pitch = desired_pitch;
event_data.thrust        = thrust;


end

