%% TVC Rocket 6DOF Simulation Initialization Script
%
% Frames:
%   N - Local navigation frame (assumed inertial, origin at intended landing location)
%   B - Rocket body frame (origin at body center of mass)
%
% Notation:
%   Position Vector: x_A_AB (vector from A to B expressed in A frame)
%   Velocity Vector: xDot_A_AB (translational velocity of A with respect to B
%                            expressed in A frame)
%   Angular Velocity Vector: w_A_BA (angular velocity of B with respect to A
%                                   expressed in A frame)
%   q: q_A_B (quaternion from B to A)
%   DCM: R_A_B (SO(3) Rotation Matrix from B to A)
%   
% 
% Notes:
%
%%%%%%%%%%%%%%%%%%%
clc; clearvars; close all;
rng(100); % set seed value for random number generator

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% OPTIONS: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

CTRL_ON =           1; %Use control (0 - off, 1 - on)
GUIDANCE_ON =       1; %Use guidance method igniting landing motor (0 - off, 1 - on)

MAX_SIM_TIME =      100; %maximum simulation time (sec)

PLOTS_ON =          1; %Generate post-run plots (0 - off, 1 - on)

%Rates:
dtOut =             0.01; %Sim output time step for plotting (sec)
dtSensor =          0.01; %Sensor (measurement) time step (sec)
dtCtrl =            0.01; %Estimator/Controller time step (sec)


%% CONSTANTS: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

gravity = 9.81; %m/sec^2
%...

%% ROCKET PARAMETERS: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%mass 
%moment of inertia
load('Aerotech_G12_thrustCurve.mat')
%...

%% INITIAL CONDITIONS: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%initial position
%initial velocity
%initial attitude
%initial angular velocity
%...

%% GUIDANCE SETUP: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% NAVIGATION SETUP: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% CONTROLLER SETUP: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% AUX FUNCTIONS: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

