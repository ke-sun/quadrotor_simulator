%plan.m compute the desired state at a given time
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


function [  ] = planning( obj, src, event_data )
%PLANNING provides the desired postion, linear velocity, and linear
%  acceleration for the provided time instance
%
%
% Author:  Ke Sun
% Date  :  10/23/2014 (MM/DD/YYYY)


% Current time
t = event_data.time;


% Desired postion at the current time
x = cos(1*t);
y = cos(1*t - pi/2);
z = cos(1*t - pi);
desired_position = [x y z]';


% Desired linear velocity at the current time
x_dot = -1 * sin(1*t);
y_dot = -1 * sin(1*t - pi/2);
z_dot = -1 * sin(1*t - pi);
desired_linear_vel = [x_dot y_dot z_dot]';


% Desired linear acceleration at the current time
x_ddot = -1 * cos(1*t);
y_ddot = -1 * cos(1*t - pi/2);
z_ddot = -1 * cos(1*t - pi);
desired_linear_acc = [x_ddot y_ddot z_ddot]';


% Desired yaw angle
% The range of the desired angles should be within [-pi pi]
desired_yaw = 0;


% Filling the output
event_data.time_stamp         = t;
event_data.desired_position   = desired_position;
event_data.desired_linear_vel = desired_linear_vel;
event_data.desired_linear_acc = desired_linear_acc;
event_data.desired_yaw        = desired_yaw;


end

