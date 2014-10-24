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

classdef stateObserver < handle
    %class STATEOBSERVER estimates the current state of the quadrotor
    %
    %
    % Author: Ke Sun
    % Date  : 06/22/2014 (MM/DD/YYYY)

    properties

        freq         % scalar, frequency at which the algorithm runs

    end

    methods

        % Class constructor
        function obj = stateObserver(observer_settings)

            obj.freq = observer_settings.freq;

        end

        % Interface of the class
        [ ] = observe(obj, src, event_data)


        % Class destructor
        function delete(obj)

        end

    end

end



