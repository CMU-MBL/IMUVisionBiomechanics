IMUVisionBiomechanics

Owen Douglas Pearl | Carnegie Mellon University

Micro Robotics Lab | Musculoskeletal Biomechanics Lab 

----------------------------------------------------------------------------------------------------------

Introduction and Proper Usage:

This repository contains code for running 4 classes of simulations in MATLAB with a nine DOF biomechanical
model for estimating full body kinematics (and dynamics and contact forces if using direct collocation):

(1) IMU and vision data fusion (tracking) via direct collocation

(2) IMU only data tracking (and denoising) via direct collocation

(3) Unconstrained IMU and vision data fusion via inverse kinematics 

(4) Unconstrained kinematics calculations using computer vision keypoints only

The code is set up to run on sample experimental data from the article below as well as synthetic created 
from sample walking mocap data contained in open source software OpenSim's installation documents. Users 
can validate findings from the referenced article with the sample experimental data as well as explore
methodological trade-offs using synthetically generated data with sample mocap data. However,
it should be noted that the findings of the study cannot be validated using the synthetically generated 
data from mocap data since they incorporate simplified assumptions and noise models that are not as 
representative of the true noise backgrounds of experimental IMU and computer vision data used to generated
the findings of the referenced article. The synthetic approach is provided for convenience and learning.
This synthetic approach was NOT utilized to generate the findings of the referenced article; it also 
overestimates the accuracy of each approach. However, it does still illustrate the trade-offs accurately.

Be sure to add all files to the MATLAB path in order to run all code from the "Main" script.

Please see the attached License.txt file for rules on use and redistribution 
(Copyright (c) 2022 Owen Douglas Pearl via MIT License)

When using this work, we ask that you please cite the relevant literature to properly credit the authors:
Pearl, O., Shin, S., Godura, A., Bergbreiter, S., & Halilaj, E. (2023). 
Fusion of video and inertial sensing data via dynamic optimization of a biomechanical model. 
Journal of Biomechanics, 155, 111617. https://doi.org/10.1016/j.jbiomech.2023.111617

The sample experimental data is adapted from the Total Capture Dataset:
Total Capture: 3D Human Pose Estimation Fusing Video and Inertial Sensors
cvssp.org/data/totalcapture/data/

The direct collocation optimal control problem is transcribed using the OptimTraj toolbox:
OptimTraj. Matthew P. Kelly. May 27, 2016. https://github.com/MatthewPeterKelly/OptimTraj
