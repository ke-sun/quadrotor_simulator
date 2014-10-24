%trajPlanner.m defines the class for the planning algorithm
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


classdef trajPlanner < handle
    %class TRAJPLANNER sets the desired postion, linear velocity and
    %acceleration and yaw angle for the next time step
    %
    %
    % Author: Ke Sun
    % Date  : 10/23/2014 (MM/DD/YYYY)

    properties

        % The frequency at which the algorithm runs
        freq

    end

    methods

        function obj = trajPlanner( planner_settings )

            obj.freq = planner_settings.freq;

        end


        [ ] = plan(obj, src, event_data)

        function delete(obj)

        end

    end

end

