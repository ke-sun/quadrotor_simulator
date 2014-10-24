%quad_sim_init.m intializes everything for the simulator
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




%% Initialize the sensor on the quadrotor

% IMU
imu_settings.linear_acc_bias  = zeros(3, 1);
imu_settings.angular_vel_bias = zeros(3, 1);
imu_settings.linear_acc_P     = zeros(3, 3);
imu_settings.angular_vel_P    = zeros(3, 3);
sensor_imu                    = sensors.imu(imu_settings);

% Size of the image
img_width = 752;
img_height = 480;

% Intrinsic matrix of the camera
mx = 1 / (6e-6);
my = 1 / (6e-6);
f  = 3e-3;
intrinsic_mat = diag([mx my 1]) * diag([f f 1]);
intrinsic_mat(1, 3) = img_width/2+1;
intrinsic_mat(2, 3) = img_height/2+1;

% Stereo camera
left_cam_settings.R         = [1 0 0; 0 0 1; 0 -1 0];
left_cam_settings.t         = [-0.1 0 0]';
left_cam_settings.K         = intrinsic_mat;
left_cam_settings.k         = [0.0001 0.0001 0.001 0.0008]';
left_cam_settings.p         = zeros(2, 1);
left_cam_settings.img_size  = [img_height, img_width];
left_cam_settings.P         = 1*eye(2);

right_cam_settings.R        = [1 0 0; 0 0 1; 0 -1 0];
right_cam_settings.t        = [0.1 0 0]';
right_cam_settings.K        = intrinsic_mat;
right_cam_settings.k        = [0.0001 0.0001 0.001 0.0008]';
right_cam_settings.p        = zeros(2, 1);
right_cam_settings.img_size = [img_height, img_width];
right_cam_settings.P        = 1*eye(2);

sensor_left_cam  = sensors.camera(left_cam_settings);
sensor_right_cam = sensors.camera(right_cam_settings);

%% Initialize quadrotor

% Sensors on the quadrotor
quad_settings.imu       = sensor_imu;
quad_settings.left_cam  = sensor_left_cam;
quad_settings.right_cam = sensor_right_cam;

% Properties of the quadrotor
quad_settings.link_len       = 0.15;
quad_settings.mass           = 1;
quad_settings.inertia_tensor = zeros(3, 3);
quad_settings.inertia_tensor(1, 1) = quad_settings.mass/2 * quad_settings.link_len^2;
quad_settings.inertia_tensor(2, 2) = quad_settings.mass/2 * quad_settings.link_len^2;
quad_settings.inertia_tensor(3, 3) = quad_settings.mass   * quad_settings.link_len^2;


% Inital state of the quadrotor
quad_settings.initial_state.R           = eye(3);
quad_settings.initial_state.yaw         = 0;
quad_settings.initial_state.pitch       = 0;
quad_settings.initial_state.roll        = 0;
quad_settings.initial_state.angular_vel = [0 0 0]';
quad_settings.initial_state.angular_acc = zeros(3, 1);
quad_settings.initial_state.position    = [1 0 -1]';
quad_settings.initial_state.linear_vel  = [0 1 0]';
quad_settings.initial_state.linear_acc  = [-1 0 1]';


%% Initialize the oberver

observer_settings.freq = 20;
state_observer = observer.stateObserver(observer_settings);


%% Initalize the trajectory planner
plan_settings.freq = 15;
traj_planner = planner.trajPlanner(plan_settings);


%% Intialize the controller

% Attitude controller
atti_control_settings.Kr         = 240*eye(3);
atti_critical_damp               = 4.5*atti_control_settings.Kr*quad_settings.inertia_tensor;
atti_control_settings.Kw         = diag(sqrt(diag(atti_critical_damp)));
atti_control_settings.freq       = 200;
atti_control_settings.max_torque = 120*ones(3, 1);
atti_controller                  = controller.attiController(atti_control_settings);


% Position controller
pos_control_settings.Kp         = 14*eye(3);
pos_critical_damp               = 4.5*pos_control_settings.Kp*quad_settings.mass;
pos_control_settings.Kd         = diag(sqrt(diag(pos_critical_damp)));
pos_control_settings.mass       = quad_settings.mass;
pos_control_settings.freq       = 10;
pos_control_settings.max_thrust = 2*9.8*quad_settings.mass;
pos_controller                  = controller.posController(pos_control_settings);



%% Initialize the simulator

% Total simulation time
sim_duration = 20;

% Settings for different algorithms running in the simulator
algo_settings.imu_freq          = 200;
algo_settings.cam_freq          = 20;
algo_settings.state_est_freq    = observer_settings.freq;
algo_settings.atti_control_freq = atti_control_settings.freq;
algo_settings.pos_control_freq  = pos_control_settings.freq;
algo_settings.traj_plan_freq    = plan_settings.freq;

algo_settings.state_est_event_handler    = @state_observer.observe;
algo_settings.traj_plan_event_handler    = @traj_planner.plan;
algo_settings.atti_control_event_handler = @atti_controller.control;
algo_settings.pos_control_event_handler  = @pos_controller.control;

% Some properties of the world
sim_settings.features  = features;
sim_settings.colors    = colors;
sim_settings.time_step = 0.001;

% Create an object of the simulator
my_simulator = quadSimulator(sim_settings, quad_settings, algo_settings);


