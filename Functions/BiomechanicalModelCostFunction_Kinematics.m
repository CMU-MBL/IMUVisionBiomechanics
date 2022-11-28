function [Cost] = BiomechanicalModelCostFunction_Kinematics(t, z, u, kinematics_table, toggleExpVsSynth)
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

x_truth = interp1( kinematics_table(1,:), kinematics_table(2,:), t, 'spline', 'extrap');
y_truth = interp1( kinematics_table(1,:), kinematics_table(3,:), t, 'spline', 'extrap');
qt_truth = interp1( kinematics_table(1,:), kinematics_table(4,:), t, 'spline', 'extrap');
qlh_truth = interp1( kinematics_table(1,:), kinematics_table(5,:), t, 'spline', 'extrap');
qrh_truth = interp1( kinematics_table(1,:), kinematics_table(6,:), t, 'spline', 'extrap');
qlk_truth = interp1( kinematics_table(1,:), kinematics_table(7,:), t, 'spline', 'extrap');
qrk_truth = interp1( kinematics_table(1,:), kinematics_table(8,:), t, 'spline', 'extrap');
qla_truth = interp1( kinematics_table(1,:), kinematics_table(9,:), t, 'spline', 'extrap');
qra_truth = interp1( kinematics_table(1,:), kinematics_table(10,:), t, 'spline', 'extrap');

%  Weights for Cost Function
w = [4.1442e+05 6133.3 1.0000e-03 1.0000e-04];

Cost = w(1).*( (X - x_truth).^2 + (Y - y_truth).^2 ) ...
     + w(2).*( (QT - qt_truth).^2 + (QLH - qlh_truth).^2 + (QRH - qrh_truth).^2 + (QLK - qlk_truth).^2 + (QRK - qrk_truth).^2 + (QLA - qla_truth).^2 + (QRA - qra_truth).^2) ...
     + w(3).*(RESX.^2 + RESY.^2) ...
     + w(4).*(T_T.^2 + T_LH.^2 + T_RH.^2 + T_LK.^2 + T_RK.^2 + T_LA.^2 + T_RA.^2);
end
