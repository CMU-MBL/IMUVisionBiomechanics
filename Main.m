%% IMUVisionBiomechanics Main Script
% Author: Owen Douglas Pearl | Carnegie Mellon University
% Musculoskeletal Biomechanics Lab & Micro Robotics Lab

% This script allows you to choose settings from which to run each class of
% simulation contained in the following work (see ReadMe for info):
% Pearl, O., Shin, S., Godura, A., Bergbreiter, S., & Halilaj, E. (2023). 
% Fusion of video and inertial sensing data via dynamic optimization of a biomechanical model. 
% Journal of Biomechanics, 155, 111617. https://doi.org/10.1016/j.jbiomech.2023.111617
clear all; close all; clc

% Settings Flags
doExperimentalData = 1; % if 0, uses synthetic data; if 1, uses sample experimental data
doDirectCollocationVSUnconstrained = 0; % if 0, performs unconstrained simulations; if 1, performs direct collocation simulations
chooseDirectCollocationTracking = 2; % if 0, tracks kinematics for validation; if 1, tracks IMUs only; if 2, tracks IMUs and computer vision keypoints (constrained fusion)
chooseUnconstrainedTracking = 1; % if 0, tracks only computer vision keypoints; if 1, tracks both IMUs and computer vision keypoints (unconstrained fusion)

% Runs the Selected Simulation
if(doDirectCollocationVSUnconstrained == 1)
    RunDirectCollocationSimulations
elseif(doDirectCollocationVSUnconstrained == 0)
    RunUnconstrainedSimulations
end