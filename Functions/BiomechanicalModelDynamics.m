function [dz] = BiomechanicalModelDynamics(t, z, u, toggleExpVsSynth)
% Load Model Parameters
if(toggleExpVsSynth == 0)
    SetModelParams_Synthetic
elseif(toggleExpVsSynth == 1)
    SetModelParams_Experimental
end
      
% Unpack State
X = z(1,:);
Y = z(2,:);
QT = z(3,:);
QLH = z(4,:);
QRH = z(5,:);
QLK = z(6,:);
QRK = z(7,:);
QLA = z(8,:);
QRA = z(9,:);
XP = z(10,:);
YP = z(11,:);
QTP = z(12,:);
QLHP = z(13,:);
QRHP = z(14,:);
QLKP = z(15,:);
QRKP = z(16,:);
QLAP = z(17,:);
QRAP = z(18,:);

% Unpack Controls
RESX = u(1,:);
RESY = u(2,:);
T_T  = u(3,:);
T_LH = u(4,:);
T_RH = u(5,:);
T_LK = u(6,:);
T_RK = u(7,:);
T_LA = u(8,:);
T_RA = u(9,:);

% Input GRF
LGRFX = u(10,:); 
LGRFY = u(11,:); 
RGRFX = u(12,:); 
RGRFY = u(13,:);

% Calculate Musculoskeletal Model Equations of Motion Using Kane's Dynamics
CalculateModelEOM

% Output State Derivative dz
dz = [XP; YP; QTP; QLHP; QRHP; QLKP; QRKP; QLAP; QRAP; XPP; YPP; QTPP; QLHPP; QRHPP; QLKPP; QRKPP; QLAPP; QRAPP];
end
