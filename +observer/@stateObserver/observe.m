%observe.m estimated the current state of the quadrotor
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


function [  ] = observe( obj, src, event_data )
%OBSERVE is the interface of the state estimation algorithm
%
% Author: Ke Sun
% Date  : 10/23/2014 (MM/DD/YYYY)


event_data.position   = zeros(3, 1);
event_data.linear_vel = zeros(3, 1);
event_data.linear_acc = zeros(3, 1);


event_data.angular_vel  = zeros(3, 1);
event_data.angular_acc  = zeros(3, 1);

event_data.R     = eye(3);
event_data.yaw   = 0;
event_data.pitch = 0;
event_data.roll  = 0;


end

