clc; clearvars; close all;

%diagonal matrix because we're working in the body axis?
I = diag([.05, .08, .8]);
%full state space model, A and B
A = [zeros(3,3), 0.5*I;
    zeros(3,3), zeros(3,3)];
B = [zeros(3,3); I^-1];

%we care about position a little more that velocity
Q = diag([100,100,100,1,1,1]); %[qe1, qe2, qe3, w1, w2, w3]
R = diag([1,1,1]);%equal control cost
K = lqr(A,B, Q, R); %find K

%configuration variables
dt = 1/400;
euler_init = [0, .3, .3]; %orientation RPY init (rad)
q_init = angle2quat(euler_init(3),euler_init(2),euler_init(1),'ZYX'); %controller estimation
w0 = [0, 0, 0]; %angular velocity
mass = .89; %kg
momentArm = 0.49; %in m
T = 15;  % thrust, N, for control calculations
servoHorn = 1; %
gimbalArm = 3.3;
servoRatio = 3.3;
maxGimbal = deg2rad(8); %max gimbal angle

filename = ['Mike Haberer F15-0 Thrust Curve.xlsx']; % Replace with actual file name
data = readtable(filename, VariableNamingRule="preserve"); % Replace with your actual file name

% Extract columns into arrays
time = data{:,1};    % First column: Time
thrust = data{:,2};  % Second column: Thrust

%lqr referenced: 
%https://www.sciencedirect.com/science/article/pii/S1367578812000387?ref=pdf_download&fr=RR-2&rr=92dbcffa994d1f34

% run the simulink model joined to this file
out = sim("TVC_6DOF_Model_WJHedits_2024a");