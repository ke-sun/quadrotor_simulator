%posControlEvntData.m provides the I/O for position controller
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


classdef (ConstructOnLoad) posControlEvntData < event.EventData

    properties

        %-----------inputs------------------
        % Desired attitude
        desired_position
        desired_linear_vel
        desired_linear_acc

        % Desired yaw angle
        desired_yaw

        % Current attitude and angular velocity
        position
        linear_vel
        linear_acc

        %-----------outputs-----------------
        % Output
        desired_roll
        desired_pitch
        thrust

    end

    methods


        function obj = posControlEvntData(desired_state, curr_state)

            obj.desired_position   = desired_state.desired_position;
            obj.desired_linear_vel = desired_state.desired_linear_vel;
            obj.desired_linear_acc = desired_state.desired_linear_acc;

            obj.desired_yaw = desired_state.desired_yaw;

            obj.position   = curr_state.position;
            obj.linear_vel = curr_state.linear_vel;
            obj.linear_acc = curr_state.linear_acc;

        end


    end

end


