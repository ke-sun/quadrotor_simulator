%quadrotor.m defines the quadrotor class
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


classdef quadrotor < handle
    %class QUADROTOR simulates the behavior of a quadrotor
    %
    %
    % Author: Ke Sun
    % Date  : 10/23/2014 (MM/DD/YYYY)

    properties

        % axes handle, contains the plot of the quadrotor
        world_axes_handle

        % line handles, contains the for links of the quadrotor
        link1
        link2
        link3
        link4
        % 3-by-1 vectors, positions of the end points of the links in the body frame of the quad rotor
        link1_body
        link2_body
        link3_body
        link4_body

        % line handles, represent the body frame
        x_axis
        y_axis
        z_axis
        % 3-by-1 vectors, basis of the body frame
        x_axis_body
        y_axis_body
        z_axis_body

        % Mass and moment of inertia of the quad rotor
        mass
        inertia_tensor

        % Sensors on the quadrotor
        imu
        left_cam
        right_cam

        % struct, current state of the quadrotor
        %         including yaw pitch roll, angular
        %         velocity, angular acceleration,
        %         position, linear velocity and linear
        %         acceleration
        state

    end

    methods

        function obj = quadrotor(world_axes_handle, quad_settings)

            % The axes where the quadrotor is drawn
            obj.world_axes_handle = world_axes_handle;

            % Colors for the front and back links
            color2 = [0 0 1];
            color1 = [1 0 0];

            % Create the links for the quadrotor
            obj.link1 = line('Parent',obj.world_axes_handle,'Color',color1,'Visible','off','LineWidth',3);
            obj.link2 = line('Parent',obj.world_axes_handle,'Color',color1,'Visible','off','LineWidth',3);
            obj.link3 = line('Parent',obj.world_axes_handle,'Color',color2,'Visible','off','LineWidth',3);
            obj.link4 = line('Parent',obj.world_axes_handle,'Color',color2,'Visible','off','LineWidth',3);

            % The vector for the links in the body frame
            link_len = quad_settings.link_len;
            obj.link1_body = [ link_len,  link_len, 0]';
            obj.link2_body = [-link_len,  link_len, 0]';
            obj.link3_body = [-link_len, -link_len, 0]';
            obj.link4_body = [ link_len, -link_len, 0]';

            % Create the body frame, which is to be shown in the figure
            obj.x_axis = line('Parent',obj.world_axes_handle,'Color',[1 0 0],'Visible','off','LineWidth',2);
            obj.y_axis = line('Parent',obj.world_axes_handle,'Color',[0 1 0],'Visible','off','LineWidth',2);
            obj.z_axis = line('Parent',obj.world_axes_handle,'Color',[0 0 1],'Visible','off','LineWidth',2);

            obj.x_axis_body = [0.08 0 0]';
            obj.y_axis_body = [0 0.08 0]';
            obj.z_axis_body = [0 0 0.08]';

            % Set the sensors on the quadrotor
            obj.imu       = quad_settings.imu;
            obj.left_cam  = quad_settings.left_cam;
            obj.right_cam = quad_settings.right_cam;


            % Set the mass the inertia tensor of the quadrotor
            obj.mass           = quad_settings.mass;
            obj.inertia_tensor = quad_settings.inertia_tensor;


            % Initialize the orientation and position of the quadrotor in
            % the world
            obj.state = quad_settings.initial_state;

        end

        % Draw the quadrobot on the WORLD axes
        [ ] = drawQuadrotor( obj )

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % @brief: Compute the state of the quadrotor at the new time instance
        % @param dt   : time interval
        % @param input: contains torque and thrust
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [ ] = dynamics( obj, dt, input )


        function [ ] = delete( obj )


        end



    end

end

