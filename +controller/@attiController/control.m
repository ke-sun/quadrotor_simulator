%control.m implements the attitude control algorithm
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


function [  ] = control( obj, src, event_data )
%CONTROL is a attitude controller
%
%
% Author: Ke Sun
% Date  : 10/23/2014 (MM/DD/YYYY)


% Desired attitude
desired_yaw   = event_data.desired_yaw;
desired_pitch = event_data.desired_pitch;
desired_roll  = event_data.desired_roll;


% Current attitude
R     = event_data.R;
pitch = asin(-R(3, 1));
yaw   = atan2(R(2, 1), R(1, 1));
roll  = atan2(R(3, 2), R(3, 3));


% Current angular velocity
angular_vel = event_data.angular_vel;


% Compute the error of attitude and anguler velocity
roll_err  = obj.shortestAngle(roll,  desired_roll);
pitch_err = obj.shortestAngle(pitch, desired_pitch);
yaw_err   = obj.shortestAngle(yaw,   desired_yaw);
er        = [roll_err, pitch_err, yaw_err]';
ew        = angular_vel;


% Compute the torque
torque    = -obj.Kr*er - obj.Kw*ew;
% absTorque = min([obj.maxAbsTorque abs(torque)], [], 2);
% torque    = sign(torque).*absTorque;

% Filling the output
event_data.torque = torque;


end

