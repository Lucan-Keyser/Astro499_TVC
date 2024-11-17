%% TVC Rocket Simulation Post-Run Script
%
% Frames:
%   N - Local navigation frame (assumed inertial, origin at intended landing location)
%   B - Rocket body frame (origin at body center of mass)
%
% Notation:
%   Position Vector: x_A_AB (vector from A to B expressed in A frame)
%   Velocity Vector: xDot_A_AB (translational velocity of A with respect to B
%                            expressed in A frame)
%   Angular Velocity Vector: w_A_AB (angular velocity of A with respect to B
%                                   expressed in A frame)
%   DCM: R_A_B (SO(3) Rotation Matrix from B to A)
%   
% 
% Notes:
%   
%
%%%%%%%%%%%%%%%%%%%
clc; close all;
set(0,'defaultAxesFontSize',20)

%% PLOTS

if PLOTS_ON

    %...

end