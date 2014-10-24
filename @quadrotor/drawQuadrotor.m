%drawQuadrotor.m draws the quadrotor in the world
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


function [ ] = drawQuadrotor( obj )
%DRAWQUADROTOR draws the quadrotor in the figure
%
%
% Author: Ke Sun
% Date  : 10/23/2014


% The orientation and translation of the body frame
R = obj.state.R;
t = obj.state.position;

% Find the cooridnates of the end points of the 4 links in the inertial
% frame
end_point1 = R * obj.link1_body + t;
end_point2 = R * obj.link2_body + t;
end_point3 = R * obj.link3_body + t;
end_point4 = R * obj.link4_body + t;

% Udpate the links
set(obj.link1,'Parent',obj.world_axes_handle,'XData',[t(1) end_point1(1)], 'YData', [t(2) end_point1(2)], 'ZData', [t(3) end_point1(3)], 'visible','on');
set(obj.link2,'Parent',obj.world_axes_handle,'XData',[t(1) end_point2(1)], 'YData', [t(2) end_point2(2)], 'ZData', [t(3) end_point2(3)], 'visible','on');
set(obj.link3,'Parent',obj.world_axes_handle,'XData',[t(1) end_point3(1)], 'YData', [t(2) end_point3(2)], 'ZData', [t(3) end_point3(3)], 'visible','on');
set(obj.link4,'Parent',obj.world_axes_handle,'XData',[t(1) end_point4(1)], 'YData', [t(2) end_point4(2)], 'ZData', [t(3) end_point4(3)], 'visible','on');


% Update the body frame
end_point1 = R * obj.x_axis_body + t;
end_point2 = R * obj.y_axis_body + t;
end_point3 = R * obj.z_axis_body + t;

set(obj.x_axis,'Parent',obj.world_axes_handle,'XData',[t(1) end_point1(1)], 'YData', [t(2) end_point1(2)], 'ZData', [t(3) end_point1(3)], 'visible','on');
set(obj.y_axis,'Parent',obj.world_axes_handle,'XData',[t(1) end_point2(1)], 'YData', [t(2) end_point2(2)], 'ZData', [t(3) end_point2(3)], 'visible','on');
set(obj.z_axis,'Parent',obj.world_axes_handle,'XData',[t(1) end_point3(1)], 'YData', [t(2) end_point3(2)], 'ZData', [t(3) end_point3(3)], 'visible','on');

end

