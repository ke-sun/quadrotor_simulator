%quadSimulator.m defines the simulator for the quadrotor
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


classdef quadSimulator < handle
    %class QUADSIMULATOR simulates the interaction between the quadrotor
    %and the world. The class also triggers differernt events in order to
    %perform state estimation, controlling and trajectory planning.
    %
    % Author: Ke Sun
    % Date  : 10/23/2014

    properties

        features               % 3-by-m matrix, 3d coordinates for the features
        feature_colors         % 3-by-m matrix, colors for the features

        robot                  % object of the class QUADROTOR

        world_fig_handle       % figure handle, contains world_axe_handle
        world_axe_handle       % axes handle,   contains the plot of features, the quadrobtor
        image_fig_handle       % figure handle, contains image_axe_handle
        image_axe_handle       % axes handle,   contains the captured image of the camera installed on the quadrotor


        time                   % scalar, current time
        time_step              % scalar, time step of the simulation


        imu_freq               % scalar, frequency of imu data measurement
        cam_freq               % scalar, frequency of image capturing
        state_est_freq         % scalar, frequency of state estimation
        atti_control_freq      % scalar, frequency of attitude control
        pos_control_freq       % scalar, frequency of position control
        traj_plan_freq         % scalar, frequency of trajectory planning


        imu_timer              % scalar, timer for IMU data measurement, recording how long has past since last IMU measurement
        cam_timer              % scalar, timer for camera
        state_est_timer        % scalar, timer for state estimation event
        atti_control_timer     % scalar, timer for attitude control event
        pos_control_timer      % scalar, timer for position control event
        traj_plan_timer        % scalar, timer for trajectory planning


        imu_measurement        % struct, record the imu data
        cam_measurement        % struct, record the camera data
        curr_state             % struct, record the estimated state from the state estimation algorithm
        desired_state          % struct, record the planned state of trajectory planner
        control_input          % struct, record the designed input from the controller

    end

    methods

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % @brief: contructor of the class
        % @param sim_settings : settings for the simulator
        % @param quad_settings: settings for the quadrotor
        % @param algo_settings: settings for the algorithms
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = quadSimulator( sim_settings, quad_settings, algo_settings)

            % Set the features in the world
            obj.features       = sim_settings.features;
            obj.feature_colors = sim_settings.colors;

            % Create figure and axes handle for the world and the captured image
            obj.world_fig_handle = figure('Name', 'Quadrotor Simulator');
            obj.world_axe_handle = axes('Parent', obj.world_fig_handle);

            % The figure for images are currently disabled
            obj.image_fig_handle = figure('Name', 'Captured Image');
            obj.image_axe_handle = axes('Parent', obj.image_fig_handle);

            % Scatter the features in the world
            % scatter3(obj.world_axe_handle, obj.features(1, :), obj.features(2, :), obj.features(3, :), 25, obj.feature_colors', 'filled');
            scatter3(obj.world_axe_handle, 0, 0, 0, 25, 'filled');
            set(obj.world_axe_handle, 'DataAspectRatio', [1 1 1], 'DataAspectRatioMode', 'manual');
            set(obj.world_axe_handle, 'XLimMode', 'manual', 'YLimMode', 'manual', 'ZLimMode', 'manual');
            set(obj.world_axe_handle, 'XLim', [-1.5 1.5], 'YLim', [-1.5 1.5], 'ZLim', [-1.5 1.5]);
            set(obj.world_axe_handle, 'NextPlot', 'add');

            % Create a quadrotor
            obj.robot = quadrotor(obj.world_axe_handle, quad_settings);

            % Set intial time to 0
            % Each time the simulaton runs, the time increases by time_step
            obj.time = 0;
            obj.time_step = sim_settings.time_step;


            % Initialize the frequencies for different events
            obj.imu_freq          = algo_settings.imu_freq;
            obj.cam_freq          = algo_settings.cam_freq;
            obj.state_est_freq    = algo_settings.state_est_freq;
            obj.atti_control_freq = algo_settings.atti_control_freq;
            obj.pos_control_freq  = algo_settings.pos_control_freq;
            obj.traj_plan_freq    = algo_settings.traj_plan_freq;

            % Initialize the timer for different events.
            % The timer is set to the ceiling at the beginning so that all
            % the events are triggered at the beginning of the simulation.
            obj.imu_timer          = 1 / obj.imu_freq;
            obj.cam_timer          = 1 / obj.cam_freq;
            obj.state_est_timer    = 1 / obj.state_est_freq;
            obj.atti_control_timer = 1 / obj.atti_control_freq;
            obj.pos_control_timer  = 1 / obj.pos_control_freq;
            obj.traj_plan_timer    = 1 / obj.traj_plan_freq;

            % Set the initial state for the quadrotor
            obj.curr_state = quad_settings.initial_state;


            % Add function handles to the different events
            addlistener(obj, 'stateEstEvnt',    algo_settings.state_est_event_handler);
            addlistener(obj, 'attiControlEvnt', algo_settings.atti_control_event_handler);
            addlistener(obj, 'posControlEvnt',  algo_settings.pos_control_event_handler);
            addlistener(obj, 'trajPlanEvnt',    algo_settings.traj_plan_event_handler);

        end

        % The interface of the simulator.
        % Forward the simulation by one time step
        [ ] = onetime_stepForward( obj )


        % Class destructor
        function delete( obj )

        end


    end

    events

        % State estimation event
        stateEstEvnt

        % Attitude control event
        attiControlEvnt

        % Position control event
        posControlEvnt

        % Trajectory planning event
        trajPlanEvnt

    end

end

