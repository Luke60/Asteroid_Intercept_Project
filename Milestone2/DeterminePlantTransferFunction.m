%% Author: _Luke Baatjes_
%% Date: _05/03/2023_
%% _EEE4119F Mechatronics II Project Milestone 2_
% 
% 
%% _DETERMINE TRANSFER FUNCTION FOR INPUT ALPHA TO OUTPUT THETA_
% Using the acceleration equation determined in milestone 1, specifically
% ddth, we can determine the transfer function of of the plant for input
% alpha to output theta using a laplace transform. In this case force is
% held constant while we attempt to control theta

% Constants and symbolic variables
syms m
F = 30550; % Constant force value used to design controller

% Determine mass moment of inertia of the rocket
Iz = (1/12)*m*(15^2 + 5^2) + m*4^2;
Iz = double(subs(Iz,m,1000));

% After laplace transforming ddth we end up with the plant for input alpha
% to output theta below
s = tf('s');
alph2th = -3.5*(F)/Iz * (1/s^2);

% Now that we have a transfer function we can use MATLAB's sisotool() to
% tune a lead-lag controller for the plant
% sisotool(-alph2th)
load('Controller.mat')

%step(-alph2th*Cc) % where Cc is the transfer function of the lead-lag compensator
