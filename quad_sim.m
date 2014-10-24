%quad_sim.m starts the simulator
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

clc;
clear;
close all;
rng('default');


%% Settings for the simulator
load ./data/features.mat;


%% Initialization
quad_sim_init;


%% Start simulation

while my_simulator.time < sim_duration + 1e-10

    my_simulator.oneTimeStepForward();
    drawnow;

end











