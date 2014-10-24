%evntData.m defines the event data for the planning algorithm
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


classdef (ConstructOnLoad) evntData < event.EventData

    properties

        %-------------- Inputs--------------------
        % scalar, current time
        time

        %--------------Outputs--------------------
        % scalar, time at which the desired references are created
        time_stamp

        % Desired properties
        desired_position
        desired_linear_vel
        desired_linear_acc
        desired_yaw

    end

    methods

        function obj = evntData( curr_time, curr_state )

            obj.time = curr_time;

            obj.desired_position   = [];
            obj.desired_linear_vel = [];
            obj.desired_linear_acc = [];
            obj.desired_yaw        = [];

        end

    end

end

