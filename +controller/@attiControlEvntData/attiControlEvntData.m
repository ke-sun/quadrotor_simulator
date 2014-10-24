%attiControlEvntData.m provides the I/O for attitude controller
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


classdef (ConstructOnLoad) attiControlEvntData < event.EventData

    properties

        %-------------inputs------------------
        % Desired attitude
        desired_yaw
        desired_pitch
        desired_roll

        % Current attitude and angular velocity
        R
        angular_vel


        %-------------outputs-----------------
        % Control input torque
        torque

    end

    methods

        function obj = attiControlEvntData( desired_state, curr_state )

            obj.desired_yaw   = desired_state.desired_yaw;
            obj.desired_pitch = desired_state.desired_pitch;
            obj.desired_roll  = desired_state.desired_roll;

            obj.R           = curr_state.R;
            obj.angular_vel = curr_state.angular_vel;

        end


    end

end

