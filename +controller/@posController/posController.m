%posController.m defines the class for position control algorithm
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


classdef posController < handle

    properties

        Kp           % 3x3 matrix, gain for the error of position
        Kd           % 3x3 matrix, gain for the error of linear velocity

        mass         % scalar, mass of the quadrotor

        freq         % scalar, the frequency of the controller
        max_thrust % scalar, the maximum thrust that can be provided

    end

    methods

        % Class Constructor
        function obj = posController( contrl_settings )

            obj.Kp = contrl_settings.Kp;
            obj.Kd = contrl_settings.Kd;

            obj.mass = contrl_settings.mass;

            obj.freq = contrl_settings.freq;
            obj.max_thrust = contrl_settings.max_thrust;


        end

        % Interface of the algorithm
        [] = control(obj, src, event_data)

        % Class destructor
        function [] = delete(obj)

        end

    end

end



