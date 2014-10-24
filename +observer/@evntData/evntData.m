%evntData.m defines the I/O for state estimation algorithms
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

        %---------------Inputs-----------------
        % Measurement from different sensors
        imu_measurement
        cam_measurement

        %---------------Outputs----------------
        % Estimated state for output
        position
        linear_vel
        linear_acc

        R
        angular_vel
        angular_acc

        yaw
        pitch
        roll

    end

    methods

        function obj = evntData( imu_measurement, cam_measurement )

            % IMU data
            obj.imu_measurement = imu_measurement;

            % Camera data
            obj.cam_measurement = cam_measurement;

        end


    end

end

