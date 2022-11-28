%% RunUnconstrainedFusion

% Load Model Parameters and data
if(doExperimentalData == 1)
    SetModelParams_Experimental
    load('Data/SampleExperimentalData.mat')
elseif(doExperimentalData == 0)
    SetModelParams_Synthetic
    OpenSimExampleDataToSyntheticIMUAndVisionData
end

% Create Tables
imu_table = [t_cycle'; torso_ax_imu_cycle'; torso_ay_imu_cycle'; ...
    torso_wz_imu_cycle'; lthigh_ax_imu_cycle'; lthigh_ay_imu_cycle'; ...
    lthigh_wz_imu_cycle'; rthigh_ax_imu_cycle'; rthigh_ay_imu_cycle'; ...
    rthigh_wz_imu_cycle'; lshank_ax_imu_cycle'; lshank_ay_imu_cycle'; ...
    lshank_wz_imu_cycle'; rshank_ax_imu_cycle'; rshank_ay_imu_cycle'; ...
    rshank_wz_imu_cycle'; lfoot_ax_imu_cycle'; lfoot_ay_imu_cycle'; ...
    lfoot_wz_imu_cycle'; rfoot_ax_imu_cycle'; rfoot_ay_imu_cycle'; ...
    rfoot_wz_imu_cycle'];

keypoint_table = [t_cycle'; proxtorso_x_keypoint_cycle'; ...
    proxtorso_y_keypoint_cycle'; hipjoint_x_keypoint_cycle'; ...
    hipjoint_y_keypoint_cycle'; lkneejoint_x_keypoint_cycle'; ...
    lkneejoint_y_keypoint_cycle'; rkneejoint_x_keypoint_cycle'; ...
    rkneejoint_y_keypoint_cycle'; lanklejoint_x_keypoint_cycle'; ...
    lanklejoint_y_keypoint_cycle'; ranklejoint_x_keypoint_cycle'; ...
    ranklejoint_y_keypoint_cycle'; ldistfoot_x_keypoint_cycle'; ...
    ldistfoot_y_keypoint_cycle'; rdistfoot_x_keypoint_cycle'; ...
    rdistfoot_y_keypoint_cycle'];

% For validating agaist kinematics tracking simulations
kinematics_table = [t_cycle'; x_cycle'; y_cycle'; qt_cycle'; ...
                     qlh_cycle'; qrh_cycle'; qlk_cycle'; qrk_cycle'; ...
                     qla_cycle'; qra_cycle'];

% Run Unconstrained Optimization for Kinematics
optfunc = @(kins) UnconstrainedSimsCostFun(kins, t_cycle, imu_table, keypoint_table, doExperimentalData, chooseUnconstrainedTracking);
kins_0 = kinematics_table(2:end,:);
options = optimoptions('fmincon', 'Display', 'off', 'MaxIter', 1000, 'MaxFunEvals',1e6, 'PlotFcns','optimplotfval');
tstart = tic;
[optimal_kinematics, optJ] = fmincon(optfunc, kins_0, [], [], [], [], [], [], [], options);
RunTime = toc(tstart)

%% Outputs for State Vizualization
cycle_viz = cycle;
x_viz_state = smooth(interp1(linspace(0,100,length(t_cycle)), optimal_kinematics(1,:), cycle, 'spline', 'extrap'));
y_viz_state = smooth(interp1(linspace(0,100,length(t_cycle)), optimal_kinematics(2,:), cycle, 'spline', 'extrap'));
qt_viz_state = smooth(interp1(linspace(0,100,length(t_cycle)), optimal_kinematics(3,:), cycle, 'spline', 'extrap'));
qlh_viz_state = smooth(interp1(linspace(0,100,length(t_cycle)), optimal_kinematics(4,:), cycle, 'spline', 'extrap'));
qrh_viz_state = smooth(interp1(linspace(0,100,length(t_cycle)), optimal_kinematics(5,:), cycle, 'spline', 'extrap'));
qlk_viz_state = smooth(interp1(linspace(0,100,length(t_cycle)), optimal_kinematics(6,:), cycle, 'spline', 'extrap'));
qrk_viz_state = smooth(interp1(linspace(0,100,length(t_cycle)), optimal_kinematics(7,:), cycle, 'spline', 'extrap'));
qla_viz_state = smooth(interp1(linspace(0,100,length(t_cycle)), optimal_kinematics(8,:), cycle, 'spline', 'extrap'));
qra_viz_state = smooth(interp1(linspace(0,100,length(t_cycle)), optimal_kinematics(9,:), cycle, 'spline', 'extrap'));

% Calculate Joint Positions for Total Pose RMSE
proxtorso_x_viz_state = x_viz_state - L_T.*sin(qt_viz_state);
proxtorso_y_viz_state = y_viz_state + L_T.*cos(qt_viz_state);
hipjoint_x_viz_state = x_viz_state;
hipjoint_y_viz_state = y_viz_state;
lkneejoint_x_viz_state = x_viz_state + L_TH.*sin(qlh_viz_state);
lkneejoint_y_viz_state = y_viz_state - L_TH.*cos(qlh_viz_state);
rkneejoint_x_viz_state = x_viz_state + L_TH.*sin(qrh_viz_state);
rkneejoint_y_viz_state = y_viz_state - L_TH.*cos(qrh_viz_state);
lanklejoint_x_viz_state = x_viz_state + L_TH.*sin(qlh_viz_state) + L_SH.*sin(qlh_viz_state+qlk_viz_state);
lanklejoint_y_viz_state = y_viz_state - L_TH.*cos(qlh_viz_state) - L_SH.*cos(qlh_viz_state+qlk_viz_state);
ranklejoint_x_viz_state = x_viz_state + L_TH.*sin(qrh_viz_state) + L_SH.*sin(qrh_viz_state+qrk_viz_state);
ranklejoint_y_viz_state = y_viz_state - L_TH.*cos(qrh_viz_state) - L_SH.*cos(qrh_viz_state+qrk_viz_state);
ldistfoot_x_viz_state = x_viz_state + L_TH.*sin(qlh_viz_state) + L_SH.*sin(qlh_viz_state+qlk_viz_state) + L_FT.*cos(qla_viz_state+qlh_viz_state+qlk_viz_state);
ldistfoot_y_viz_state = y_viz_state - L_TH.*cos(qlh_viz_state) - L_SH.*cos(qlh_viz_state+qlk_viz_state) + L_FT.*sin(qla_viz_state+qlh_viz_state+qlk_viz_state);
rdistfoot_x_viz_state = x_viz_state + L_TH.*sin(qrh_viz_state) + L_SH.*sin(qrh_viz_state+qrk_viz_state) + L_FT.*cos(qra_viz_state+qrh_viz_state+qrk_viz_state);
rdistfoot_y_viz_state = y_viz_state - L_TH.*cos(qrh_viz_state) - L_SH.*cos(qrh_viz_state+qrk_viz_state) + L_FT.*sin(qra_viz_state+qrh_viz_state+qrk_viz_state);

% Calculate IMU Locations (for visualizing locations)
torso_x_viz_state = x_viz_state - LEN_HIP_TORSOIMU*sin(qt_viz_state);
torso_y_viz_state = y_viz_state + LEN_HIP_TORSOIMU*cos(qt_viz_state);
lthigh_x_viz_state = x_viz_state + LEN_HIP_THIGHIMU*sin(qlh_viz_state);
lthigh_y_viz_state = y_viz_state - LEN_HIP_THIGHIMU*cos(qlh_viz_state);
rthigh_x_viz_state = x_viz_state + LEN_HIP_THIGHIMU*sin(qrh_viz_state);
rthigh_y_viz_state = y_viz_state - LEN_HIP_THIGHIMU*cos(qrh_viz_state);
lshank_x_viz_state = x_viz_state + L_TH*sin(qlh_viz_state) + LEN_KNEE_SHANKIMU*sin(qlh_viz_state+qlk_viz_state);
lshank_y_viz_state = y_viz_state - L_TH*cos(qlh_viz_state) - LEN_KNEE_SHANKIMU*cos(qlh_viz_state+qlk_viz_state);
rshank_x_viz_state = x_viz_state + L_TH*sin(qrh_viz_state) + LEN_KNEE_SHANKIMU*sin(qrh_viz_state+qrk_viz_state);
rshank_y_viz_state = y_viz_state - L_TH*cos(qrh_viz_state) - LEN_KNEE_SHANKIMU*cos(qrh_viz_state+qrk_viz_state);
lfoot_x_viz_state = x_viz_state + L_TH*sin(qlh_viz_state) + L_SH*sin(qlh_viz_state+qlk_viz_state) + LEN_ANKLE_FOOTIMU*cos(qla_viz_state+qlh_viz_state+qlk_viz_state);
lfoot_y_viz_state = y_viz_state - L_TH*cos(qlh_viz_state) - L_SH*cos(qlh_viz_state+qlk_viz_state) + LEN_ANKLE_FOOTIMU*sin(qla_viz_state+qlh_viz_state+qlk_viz_state);
rfoot_x_viz_state = x_viz_state + L_TH*sin(qrh_viz_state) + L_SH*sin(qrh_viz_state+qrk_viz_state) + LEN_ANKLE_FOOTIMU*cos(qra_viz_state+qrh_viz_state+qrk_viz_state);
rfoot_y_viz_state = y_viz_state - L_TH*cos(qrh_viz_state) - L_SH*cos(qrh_viz_state+qrk_viz_state) + LEN_ANKLE_FOOTIMU*sin(qra_viz_state+qrh_viz_state+qrk_viz_state);

% Calculate Tracking Error Metrics
mean_pose_rmse_cm = 100.*mean([rms(proxtorso_x_viz_state - proxtorso_x_cycle), rms(proxtorso_y_viz_state - proxtorso_y_cycle), rms(hipjoint_x_viz_state - hipjoint_x_cycle), rms(hipjoint_y_viz_state - hipjoint_y_cycle), rms(lkneejoint_x_viz_state - lkneejoint_x_cycle), rms(lkneejoint_y_viz_state - lkneejoint_y_cycle), rms(rkneejoint_x_viz_state - rkneejoint_x_cycle), rms(rkneejoint_y_viz_state - rkneejoint_y_cycle), rms(lanklejoint_x_viz_state - lanklejoint_x_cycle), rms(lanklejoint_y_viz_state - lanklejoint_y_cycle), rms(ranklejoint_x_viz_state - ranklejoint_x_cycle), rms(ranklejoint_y_viz_state - ranklejoint_y_cycle), rms(ldistfoot_x_viz_state - ldistfoot_x_cycle), rms(ldistfoot_y_viz_state - ldistfoot_y_cycle), rms(rdistfoot_x_viz_state - rdistfoot_x_cycle), rms(rdistfoot_y_viz_state - rdistfoot_y_cycle)])
std_pose_rmse_cm = 100.*std([rms(proxtorso_x_viz_state - proxtorso_x_cycle), rms(proxtorso_y_viz_state - proxtorso_y_cycle), rms(hipjoint_x_viz_state - hipjoint_x_cycle), rms(hipjoint_y_viz_state - hipjoint_y_cycle), rms(lkneejoint_x_viz_state - lkneejoint_x_cycle), rms(lkneejoint_y_viz_state - lkneejoint_y_cycle), rms(rkneejoint_x_viz_state - rkneejoint_x_cycle), rms(rkneejoint_y_viz_state - rkneejoint_y_cycle), rms(lanklejoint_x_viz_state - lanklejoint_x_cycle), rms(lanklejoint_y_viz_state - lanklejoint_y_cycle), rms(ranklejoint_x_viz_state - ranklejoint_x_cycle), rms(ranklejoint_y_viz_state - ranklejoint_y_cycle), rms(ldistfoot_x_viz_state - ldistfoot_x_cycle), rms(ldistfoot_y_viz_state - ldistfoot_y_cycle), rms(rdistfoot_x_viz_state - rdistfoot_x_cycle), rms(rdistfoot_y_viz_state - rdistfoot_y_cycle)]);
mean_kin_rmse_degs = 180/pi .* mean([rms(qt_viz_state - qt_cycle), rms(qlh_viz_state - qlh_cycle), rms(qrh_viz_state - qrh_cycle), rms(qlk_viz_state - qlk_cycle), rms(qrk_viz_state - qrk_cycle), rms(qla_viz_state - qla_cycle), rms(qra_viz_state - qra_cycle)])
std_kin_rmse_degs = 180/pi .* std([rms(qt_viz_state - qt_cycle), rms(qlh_viz_state - qlh_cycle), rms(qrh_viz_state - qrh_cycle), rms(qlk_viz_state - qlk_cycle), rms(qrk_viz_state - qrk_cycle), rms(qla_viz_state - qla_cycle), rms(qra_viz_state - qra_cycle)]);

% Run Visualizer
VisualizeUnconstrainedSimulation