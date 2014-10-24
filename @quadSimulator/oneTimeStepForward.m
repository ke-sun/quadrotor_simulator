%oneTimestepForward.m forward the simulation by one time step
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


function [ ] = oneTimeStepForward( obj )
%ONETIMESTEPFORWARD forwards the simulation by one time step
%
% Author: Ke Sun
% Date  : 10/24/2014



%===================================================================
%                        Measurement
%===================================================================

% IMU measurement
obj.imu_timer = obj.imu_timer + obj.time_step;
if obj.imu_timer >= 1/obj.imu_freq - 1e-10

    [measured_linear_acc, measured_angular_vel] = obj.robot.imu.imuMeasure(obj.curr_state);
    obj.imu_measurement.linear_acc  = measured_linear_acc;
    obj.imu_measurement.angular_vel = measured_angular_vel;
    obj.imu_measurement.time_stamp  = obj.time;
    obj.imu_timer                   = 0;

else

    obj.imu_measurement.linear_acc  = [];
    obj.imu_measurement.angular_vel = [];
    obj.imu_measurement.time_stamp  = [];

end

% Camera measurement
obj.cam_timer = obj.cam_timer + obj.time_step;
if obj.cam_timer >= 1/obj.cam_freq - 1e-10

    [ left_features,  left_tags] = obj.robot.left_cam.capture_image(obj.curr_state, obj.features);
    [right_features, right_tags] = obj.robot.right_cam.capture_image(obj.curr_state, obj.features);

    obj.cam_measurement.left_cam.time_stamp  = obj.time;
    obj.cam_measurement.right_cam.time_stamp = obj.time;
    obj.cam_measurement.left_cam.features    = left_features;
    obj.cam_measurement.right_cam.features   = right_features;
    obj.cam_measurement.left_cam.tags        = left_tags;
    obj.cam_measurement.right_cam.tags       = right_tags;

    obj.cam_measurement.time_stamp = obj.time;
    obj.cam_timer                  = 0;

else

    obj.cam_measurement.left_features  = [];
    obj.cam_measurement.right_features = [];
    obj.cam_measurement.left_tags      = [];
    obj.cam_measurement.right_tags     = [];
    obj.cam_measurement.time_stamp     = [];

end


%===================================================================
%                        State Estimation
%===================================================================


obj.state_est_timer = obj.state_est_timer + obj.time_step;
if obj.state_est_timer > 1/obj.state_est_freq - 1e-10

    state_est_data = observer.evntData(obj.imu_measurement, obj.cam_measurement);
    notify(obj, 'stateEstEvnt', state_est_data);
    obj.state_est_timer = 0;

    % Set the current state
%     obj.currentState.time_stamp         = obj.time;
%     obj.curr_state.position             = state_est_data.position;
%     obj.curr_state.linear_velocity      = state_est_data.linear_velocity;
%     obj.curr_state.linear_acceleration  = state_est_data.linear_acceleration;
%     obj.curr_state.R                    = state_est_data.R;
%     obj.curr_state.angular_velocity     = state_est_data.angular_velocity;
%     obj.curr_state.angular_acceleration = state_est_data.angular_acceleration;
%     obj.curr_state.yaw                  = state_est_data.yaw;
%     obj.curr_state.pitch                = state_est_data.pitch;
%     obj.curr_state.roll                 = state_est_data.roll;

end
% This is only a dummy observer currently. So the current state is set to
% be the true state read from the robot.
obj.curr_state.time_stamp  = obj.time;
obj.curr_state.position    = obj.robot.state.position;
obj.curr_state.linear_vel  = obj.robot.state.linear_vel;
obj.curr_state.linear_acc  = obj.robot.state.linear_acc;
obj.curr_state.R           = obj.robot.state.R;
obj.curr_state.angular_vel = obj.robot.state.angular_vel;
obj.curr_state.angular_acc = obj.robot.state.angular_acc;
obj.curr_state.yaw         = obj.robot.state.yaw;
obj.curr_state.pitch       = obj.robot.state.pitch;
obj.curr_state.roll        = obj.robot.state.roll;


%===================================================================
%                       Trajectory Planning
%===================================================================

obj.traj_plan_timer = obj.traj_plan_timer + obj.time_step;
if obj.traj_plan_timer > 1/obj.traj_plan_freq - 1e-10

    traj_plan_data = planner.evntData(obj.time, obj.curr_state);
    notify(obj, 'trajPlanEvnt', traj_plan_data);
    obj.traj_plan_timer = 0;

    % Set the entries of the desired state
    obj.desired_state.time_stamp         = obj.time;
    obj.desired_state.desired_yaw        = traj_plan_data.desired_yaw;
    obj.desired_state.desired_position   = traj_plan_data.desired_position;
    obj.desired_state.desired_linear_vel = traj_plan_data.desired_linear_vel;
    obj.desired_state.desired_linear_acc = traj_plan_data.desired_linear_acc;

end



%===================================================================
%                           Control
%===================================================================

%-------------------- positionition controller---------------------------
obj.pos_control_timer = obj.pos_control_timer + obj.time_step;
if obj.pos_control_timer > 1/obj.pos_control_freq -1e-10

    pos_control_data = controller.posControlEvntData(obj.desired_state, obj.curr_state);
    notify(obj, 'posControlEvnt', pos_control_data);
    obj.pos_control_timer = 0;

    % Set the entries of the disired state
    obj.desired_state.time_stamp_from_pos_Controller = obj.time;
    obj.desired_state.desired_roll                   = pos_control_data.desired_roll;
    obj.desired_state.desired_pitch                  = pos_control_data.desired_pitch;

    % Set the entries of the control input
    obj.control_input.thrust_time_stamp = obj.time;
    obj.control_input.thrust            = pos_control_data.thrust;

end


%-------------------- Attitude controller---------------------------
obj.atti_control_timer = obj.atti_control_timer + obj.time_step;
if obj.atti_control_timer > 1/obj.atti_control_freq - 1e-10

    atti_control_data = controller.attiControlEvntData(obj.desired_state, obj.curr_state);
    notify(obj, 'attiControlEvnt', atti_control_data);
    obj.atti_control_timer = 0;

    % Set the entries of the control input
    obj.control_input.torque_time_stamp = obj.time;
    obj.control_input.torque            = atti_control_data.torque;

end



%===================================================================
%                          Dynamics
%===================================================================


% Compute the state of the quadrotor at the next step
obj.robot.dynamics(obj.time_step, obj.control_input);



%===================================================================
%                        Visualization
%===================================================================

% Draw the quadrotor
obj.robot.drawQuadrotor();

% Show the captured image
if ~isempty(obj.cam_measurement.time_stamp)
    img = zeros(obj.robot.left_cam.img_size);
    imshow(img, 'Parent', obj.image_axe_handle);
    set(obj.image_axe_handle, 'NextPlot', 'add');
    scatter(obj.image_axe_handle, obj.cam_measurement.left_cam.features(1, :), obj.cam_measurement.left_cam.features(2, :), 20, obj.feature_colors(:, obj.cam_measurement.left_cam.tags)', 'filled');
    set(obj.image_axe_handle, 'NextPlot', 'replace');
end


% Advance the time by time_step
obj.time = obj.time + obj.time_step;

end

