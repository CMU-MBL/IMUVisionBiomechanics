function [Cost] = BiomechanicalModelCostFunction_IMUs(t, z, u, imu_table, toggleExpVsSynth)
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

% Unpack IMU Table for Tracking
torso_ax_imu = interp1( imu_table(1,:), imu_table(2,:), t, 'spline', 'extrap');
torso_ay_imu = interp1( imu_table(1,:), imu_table(3,:), t, 'spline', 'extrap');
torso_wz_imu = interp1( imu_table(1,:), imu_table(4,:), t, 'spline', 'extrap');
lthigh_ax_imu = interp1( imu_table(1,:), imu_table(5,:), t, 'spline', 'extrap');
lthigh_ay_imu = interp1( imu_table(1,:), imu_table(6,:), t, 'spline', 'extrap');
lthigh_wz_imu = interp1( imu_table(1,:), imu_table(7,:), t, 'spline', 'extrap');
rthigh_ax_imu = interp1( imu_table(1,:), imu_table(8,:), t, 'spline', 'extrap');
rthigh_ay_imu = interp1( imu_table(1,:), imu_table(9,:), t, 'spline', 'extrap');
rthigh_wz_imu = interp1( imu_table(1,:), imu_table(10,:), t, 'spline', 'extrap');
lshank_ax_imu = interp1( imu_table(1,:), imu_table(11,:), t, 'spline', 'extrap');
lshank_ay_imu = interp1( imu_table(1,:), imu_table(12,:), t, 'spline', 'extrap');
lshank_wz_imu = interp1( imu_table(1,:), imu_table(13,:), t, 'spline', 'extrap');
rshank_ax_imu = interp1( imu_table(1,:), imu_table(14,:), t, 'spline', 'extrap');
rshank_ay_imu = interp1( imu_table(1,:), imu_table(15,:), t, 'spline', 'extrap');
rshank_wz_imu = interp1( imu_table(1,:), imu_table(16,:), t, 'spline', 'extrap');
lfoot_ax_imu = interp1( imu_table(1,:), imu_table(17,:), t, 'spline', 'extrap');
lfoot_ay_imu = interp1( imu_table(1,:), imu_table(18,:), t, 'spline', 'extrap');
lfoot_wz_imu = interp1( imu_table(1,:), imu_table(19,:), t, 'spline', 'extrap');
rfoot_ax_imu = interp1( imu_table(1,:), imu_table(20,:), t, 'spline', 'extrap');
rfoot_ay_imu = interp1( imu_table(1,:), imu_table(21,:), t, 'spline', 'extrap');
rfoot_wz_imu = interp1( imu_table(1,:), imu_table(22,:), t, 'spline', 'extrap');

% Compute Model State IMU Values
torso_ax_state = XPP + LEN_HIP_TORSOIMU.*(sin(QT).*QTP.^2-cos(QT).*QTPP);
torso_ay_state = YPP - LEN_HIP_TORSOIMU.*(cos(QT).*QTP.^2+sin(QT).*QTPP);
torso_wz_state = QTP;
lthigh_ax_state = XPP - LEN_HIP_THIGHIMU.*(sin(QLH).*QLHP.^2-cos(QLH).*QLHPP);
lthigh_ay_state = YPP + LEN_HIP_THIGHIMU.*(cos(QLH).*QLHP.^2+sin(QLH).*QLHPP);
lthigh_wz_state = QLHP;
rthigh_ax_state = XPP - LEN_HIP_THIGHIMU.*(sin(QRH).*QRHP.^2-cos(QRH).*QRHPP);
rthigh_ay_state = YPP + LEN_HIP_THIGHIMU.*(cos(QRH).*QRHP.^2+sin(QRH).*QRHPP);
rthigh_wz_state = QRHP;
lshank_ax_state = XPP - L_TH.*(sin(QLH).*QLHP.^2-cos(QLH).*QLHPP) - LEN_KNEE_SHANKIMU.*(sin(QLH+QLK).*(QLHP+QLKP).^2-cos(QLH+QLK).*(QLHPP+QLKPP));
lshank_ay_state = YPP + L_TH.*(cos(QLH).*QLHP.^2+sin(QLH).*QLHPP) + LEN_KNEE_SHANKIMU.*(cos(QLH+QLK).*(QLHP+QLKP).^2+sin(QLH+QLK).*(QLHPP+QLKPP));
lshank_wz_state = (QLHP+QLKP);
rshank_ax_state = XPP - L_TH.*(sin(QRH).*QRHP.^2-cos(QRH).*QRHPP) - LEN_KNEE_SHANKIMU.*(sin(QRH+QRK).*(QRHP+QRKP).^2-cos(QRH+QRK).*(QRHPP+QRKPP));
rshank_ay_state = YPP + L_TH.*(cos(QRH).*QRHP.^2+sin(QRH).*QRHPP) + LEN_KNEE_SHANKIMU.*(cos(QRH+QRK).*(QRHP+QRKP).^2+sin(QRH+QRK).*(QRHPP+QRKPP));
rshank_wz_state = (QRHP+QRKP);
lfoot_ax_state = XPP - L_TH.*(sin(QLH).*QLHP.^2-cos(QLH).*QLHPP) - L_SH.*(sin(QLH+QLK).*(QLHP+QLKP).^2-cos(QLH+QLK).*(QLHPP+QLKPP)) - LEN_ANKLE_FOOTIMU.*(cos(QLA+QLH+QLK).*(QLAP+QLHP+QLKP).^2+sin(QLA+QLH+QLK).*(QLAPP+QLHPP+QLKPP));
lfoot_ay_state = YPP + L_TH.*(cos(QLH).*QLHP.^2+sin(QLH).*QLHPP) + L_SH.*(cos(QLH+QLK).*(QLHP+QLKP).^2+sin(QLH+QLK).*(QLHPP+QLKPP)) - LEN_ANKLE_FOOTIMU.*(sin(QLA+QLH+QLK).*(QLAP+QLHP+QLKP).^2-cos(QLA+QLH+QLK).*(QLAPP+QLHPP+QLKPP));
lfoot_wz_state = QLAP + (QLHP+QLKP);
rfoot_ax_state = XPP - L_TH.*(sin(QRH).*QRHP.^2-cos(QRH).*QRHPP) - L_SH.*(sin(QRH+QRK).*(QRHP+QRKP).^2-cos(QRH+QRK).*(QRHPP+QRKPP)) - LEN_ANKLE_FOOTIMU.*(cos(QRA+QRH+QRK).*(QRAP+QRHP+QRKP).^2+sin(QRA+QRH+QRK).*(QRAPP+QRHPP+QRKPP));
rfoot_ay_state = YPP + L_TH.*(cos(QRH).*QRHP.^2+sin(QRH).*QRHPP) + L_SH.*(cos(QRH+QRK).*(QRHP+QRKP).^2+sin(QRH+QRK).*(QRHPP+QRKPP)) - LEN_ANKLE_FOOTIMU.*(sin(QRA+QRH+QRK).*(QRAP+QRHP+QRKP).^2-cos(QRA+QRH+QRK).*(QRAPP+QRHPP+QRKPP));
rfoot_wz_state = QRAP + (QRHP+QRKP);

%  Weights for Cost Function [imuacc, imuangvel, RES, Torques]
w = [3.157 83.967 1.0000e-03 1.0000e-04];

% Cost Function
Cost = w(1).*( (torso_ax_state - torso_ax_imu).^2 + (torso_ay_state - torso_ay_imu).^2 + (lthigh_ax_state - lthigh_ax_imu).^2 + (lthigh_ay_state - lthigh_ay_imu).^2 + (rthigh_ax_state - rthigh_ax_imu).^2 + (rthigh_ay_state - rthigh_ay_imu).^2 + (lshank_ax_state - lshank_ax_imu).^2 + (lshank_ay_state - lshank_ay_imu).^2 + (rshank_ax_state - rshank_ax_imu).^2 + (rshank_ay_state - rshank_ay_imu).^2 + (lfoot_ax_state - lfoot_ax_imu).^2 + (lfoot_ay_state - lfoot_ay_imu).^2 + (rfoot_ax_state - rfoot_ax_imu).^2 + (rfoot_ay_state - rfoot_ay_imu).^2 ) ...
     + w(2).*( (torso_wz_state - torso_wz_imu).^2 + (lthigh_wz_state - lthigh_wz_imu).^2 + (rthigh_wz_state - rthigh_wz_imu).^2  + (lshank_wz_state - lshank_wz_imu).^2 + (rshank_wz_state - rshank_wz_imu).^2 + (lfoot_wz_state - lfoot_wz_imu).^2 + (rfoot_wz_state - rfoot_wz_imu).^2 ) ...
     + w(3).*(RESX.^2 + RESY.^2) + w(4).*(T_T.^2 + T_LH.^2 + T_RH.^2 + T_LK.^2 + T_RK.^2 + T_LA.^2 + T_RA.^2);
end