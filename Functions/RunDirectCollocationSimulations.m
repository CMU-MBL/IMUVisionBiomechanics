%% RunDirectCollocationSimulations

% Load Model Parameters and data
if(doExperimentalData == 1)
    SetModelParams_Experimental
    load('Data/SampleExperimentalData.mat')
elseif(doExperimentalData == 0)
    SetModelParams_Synthetic
    OpenSimExampleDataToSyntheticIMUAndVisionData
end

% Run Direct Collocation (using OptimTraj)

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%             Set up optimization params and lookup tables                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

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
    
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Set up function handles                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.func.dynamics = @(t,z,u)( BiomechanicalModelDynamics(t, z, u,  doExperimentalData) );

if(chooseDirectCollocationTracking == 0)
    problem.func.pathObj = @(t,z,u)( BiomechanicalModelCostFunction_Kinematics(t, z, u, kinematics_table, doExperimentalData) );

elseif(chooseDirectCollocationTracking == 1)
    problem.func.pathObj = @(t,z,u)( BiomechanicalModelCostFunction_IMUs(t, z, u, imu_table, doExperimentalData) );

elseif(chooseDirectCollocationTracking == 2)
    problem.func.pathObj = @(t,z,u)( BiomechanicalModelCostFunction_Fusion(t, z, u, imu_table, keypoint_table, doExperimentalData) );
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Set up bounds on time, state, and control                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
t0 = t_cycle(1);  tF = t_cycle(end);
problem.bounds.initialTime.low = t0;
problem.bounds.initialTime.upp = t0;
problem.bounds.finalTime.low = tF;
problem.bounds.finalTime.upp = tF;

x0 = x_cycle(1);
xp0 = xp_cycle(1);
y0 = y_cycle(1);
yp0 = yp_cycle(1);
qt0 = qt_cycle(1);
qtp0 = qtp_cycle(1);
qlh0 = qlh_cycle(1);
qlhp0 = qlhp_cycle(1);
qrh0 = qrh_cycle(1);
qrhp0 = qrhp_cycle(1);
qlk0 = qlk_cycle(1);
qlkp0 = qlkp_cycle(1);
qrk0 = qrk_cycle(1);
qrkp0 = qrkp_cycle(1);
qla0 = qla_cycle(1);
qlap0 = qlap_cycle(1);
qra0 = qra_cycle(1);
qrap0 = qrap_cycle(1);

problem.bounds.initialState.low = [x0; y0; qt0; qlh0; qrh0; qlk0; qrk0; qla0; qra0; xp0; yp0; qtp0; qlhp0; qrhp0; qlkp0; qrkp0; qlap0; qrap0];
problem.bounds.initialState.upp = [x0; y0; qt0; qlh0; qrh0; qlk0; qrk0; qla0; qra0; xp0; yp0; qtp0; qlhp0; qrhp0; qlkp0; qrkp0; qlap0; qrap0];

RESX_low = -200; RESX_upp = 200;
RESY_low = -200; RESY_upp = 200;
T_T_low = -500; T_T_upp = 500;
T_LH_low = -500; T_LH_upp = 500;
T_RH_low = -500; T_RH_upp = 500;
T_LK_low = -500; T_LK_upp = 500;
T_RK_low = -500; T_RK_upp = 500;
T_LA_low = -500; T_LA_upp = 500;
T_RA_low = -500; T_RA_upp = 500;
LGRFX_low = -500; LGRFX_upp = 500;
LGRFY_low = 0; LGRFY_upp = 1000;
RGRFX_low = -500; RGRFX_upp = 500;
RGRFY_low = 0; RGRFY_upp = 1000;

problem.bounds.control.low = [RESX_low; RESY_low; T_T_low; T_LH_low; T_RH_low; T_LK_low; T_RK_low; T_LA_low; T_RA_low; LGRFX_low; LGRFY_low; RGRFX_low; RGRFY_low];
problem.bounds.control.upp = [RESX_upp; RESY_upp; T_T_upp; T_LH_upp; T_RH_upp; T_LK_upp; T_RK_upp; T_LA_upp; T_RA_upp; LGRFX_upp; LGRFY_upp; RGRFX_upp; RGRFY_upp];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              Create an initial guess for the trajectory                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
t_guess = t_cycle';
z_guess = [x_cycle'; y_cycle'; qt_cycle'; qlh_cycle'; qrh_cycle'; qlk_cycle'; qrk_cycle';  qla_cycle'; qra_cycle'; xp_cycle'; yp_cycle'; qtp_cycle'; qlhp_cycle'; qrhp_cycle'; qlkp_cycle'; qrkp_cycle';  qlap_cycle'; qrap_cycle'];
u_guess = [zeros(1,length(t_cycle)); zeros(1,length(t_cycle)); pelvtilt_cycle'; lhipmoment_cycle'; rhipmoment_cycle'; lkneemoment_cycle'; rkneemoment_cycle'; lanklemoment_cycle'; ranklemoment_cycle'; grflx_cycle'; grfly_cycle'; grfrx_cycle'; grfry_cycle'];

problem.guess.time = t_guess;
problem.guess.state = z_guess;
problem.guess.control = u_guess;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Options:                                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

method = 'trapezoid';
% method = 'hermiteSimpson';
% method = 'chebyshev';
% method = 'rungeKutta';

switch method
    case 'trapezoid'
        
        % First iteration: get a more reasonable guess
        problem.options(1).nlpOpt = optimset(...
            'Display','iter',...   % {'iter','final','off'}
            'TolFun',1e-3,...
            'MaxFunEvals',1e4,...
            'MaxIter', 10e3,...
            'PlotFcns','optimplotfval');   %options for fmincon
        problem.options(1).verbose = 3; % How much to print out?
        problem.options(1).method = 'trapezoid'; % Select the transcription method
        problem.options(1).trapezoid.nGrid = 10;  %method-specific options

        
        % Second iteration: refine guess to get precise soln
        problem.options(2).nlpOpt = optimset(...
            'Display','iter',...   % {'iter','final','off'}
            'TolFun',1e-5,...
            'MaxFunEvals',1e6,...   %options for fmincon
            'MaxIter', 200,...
            'PlotFcns','optimplotfval');
        problem.options(2).verbose = 3; % How much to print out
        problem.options(2).method = 'trapezoid'; % Select the transcription method
        problem.options(2).trapezoid.nGrid = 20;  %method-specific options
        
    case 'hermiteSimpson'
        
        % First iteration: get a more reasonable guess
        problem.options(1).nlpOpt = optimset(...
            'Display','iter',...   % {'iter','final','off'}
            'TolFun',1e-4,...
            'MaxFunEvals',1e5);   %options for fmincon
        problem.options(1).verbose = 3; % How much to print out?
        problem.options(1).method = 'hermiteSimpson'; % Select the transcription method
        problem.options(1).hermiteSimpson.nSegment = 10;  %method-specific options
        
        
        % Second iteration: refine guess to get precise soln
        problem.options(2).nlpOpt = optimset(...
            'Display','iter',...   % {'iter','final','off'}
            'TolFun',1e-6,...
            'MaxFunEvals',1e6);   %options for fmincon
        problem.options(2).verbose = 3; % How much to print out?
        problem.options(2).method = 'hermiteSimpson'; % Select the transcription method
        problem.options(2).hermiteSimpson.nSegment = 20;  %method-specific options
        
        
    case 'chebyshev'
        
        % First iteration: get a more reasonable guess
        problem.options(1).nlpOpt = optimset(...
            'Display','iter',...   % {'iter','final','off'}
            'TolFun',1e-3,...
            'MaxFunEvals',15e5);   %options for fmincon
        problem.options(1).verbose = 3; % How much to print out?
        problem.options(1).method = 'chebyshev'; % Select the transcription method
        problem.options(1).chebyshev.nColPts = 9;  %method-specific options
        
        
        % Second iteration: refine guess to get precise soln
        problem.options(2).nlpOpt = optimset(...
            'Display','iter',...   % {'iter','final','off'}
            'TolFun',1e-8,...
            'MaxFunEvals',15e5);   %options for fmincon
        problem.options(2).verbose = 3; % How much to print out?
        problem.options(2).method = 'chebyshev'; % Select the transcription method
        problem.options(2).chebyshev.nColPts = 15;  %method-specific options
        
    case 'rungeKutta'
        problem.options(1).nlpOpt = optimset('TolFun', 1e-3, 'MaxIter', 1e3, 'PlotFcns','optimplotfval', 'Display', 'iter'); % options for fmincon
        problem.options(1).method = 'rungeKutta'; % Select the transcription method
        problem.options(1).defaultAccuracy = 'low';
        problem.options(1).verbose = 1; % How much to print out?
        problem.options(2).nlpOpt = optimset('TolFun', 1e-5, 'MaxFunEvals', 1e5, 'MaxIter', 1e3, 'PlotFcns','optimplotfval', 'Display', 'iter'); % options for fmincon
        problem.options(2).method = 'rungeKutta'; % Select the transcription method
        problem.options(2).defaultAccuracy = 'medium';
        problem.options(2).verbose = 1; % How much to print out?
        
    otherwise
        error('Invalid method!');
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Solve!                                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%%% THE KEY LINE:
tstart = tic;
soln = optimTraj(problem);
RunTime = toc(tstart)

% Transcription Grid points:
t_grid = soln(end).grid.time;
x_grid = soln(end).grid.state(1,:);
y_grid = soln(end).grid.state(2,:);
qt_grid = soln(end).grid.state(3,:);
qlh_grid = soln(end).grid.state(4,:);
qrh_grid = soln(end).grid.state(5,:);
qlk_grid = soln(end).grid.state(6,:);
qrk_grid = soln(end).grid.state(7,:);
qla_grid = soln(end).grid.state(8,:);
qra_grid = soln(end).grid.state(9,:);
xp_grid = soln(end).grid.state(10,:);
yp_grid = soln(end).grid.state(11,:);
qtp_grid = soln(end).grid.state(12,:);
qlhp_grid = soln(end).grid.state(13,:);
qrhp_grid = soln(end).grid.state(14,:);
qlkp_grid = soln(end).grid.state(15,:);
qrkp_grid = soln(end).grid.state(16,:);
qlap_grid = soln(end).grid.state(17,:);
qrap_grid = soln(end).grid.state(18,:);
RESx_grid = soln(end).grid.control(1,:);
RESy_grid = soln(end).grid.control(2,:);
T_t_grid = soln(end).grid.control(3,:);
T_lh_grid = soln(end).grid.control(4,:);
T_rh_grid = soln(end).grid.control(5,:);
T_lk_grid = soln(end).grid.control(6,:);
T_rk_grid = soln(end).grid.control(7,:);
T_la_grid = soln(end).grid.control(8,:);
T_ra_grid = soln(end).grid.control(9,:);
LGRFx_control_grid = soln(end).grid.control(10,:);
LGRFy_control_grid = soln(end).grid.control(11,:);
RGRFx_control_grid = soln(end).grid.control(12,:);
RGRFy_control_grid = soln(end).grid.control(13,:);

% Outputs for State Vizualization
cycle_viz = cycle;
x_viz_state = interp1(linspace(0,100,length(t_grid)), x_grid, cycle, 'spline', 'extrap');
y_viz_state = interp1(linspace(0,100,length(t_grid)), y_grid, cycle, 'spline', 'extrap');
qt_viz_state = interp1(linspace(0,100,length(t_grid)), qt_grid, cycle, 'spline', 'extrap');
qlh_viz_state = interp1(linspace(0,100,length(t_grid)), qlh_grid, cycle, 'spline', 'extrap');
qrh_viz_state = interp1(linspace(0,100,length(t_grid)), qrh_grid, cycle, 'spline', 'extrap');
qlk_viz_state = interp1(linspace(0,100,length(t_grid)), qlk_grid, cycle, 'spline', 'extrap');
qrk_viz_state = interp1(linspace(0,100,length(t_grid)), qrk_grid, cycle, 'spline', 'extrap');
qla_viz_state = interp1(linspace(0,100,length(t_grid)), qla_grid, cycle, 'spline', 'extrap');
qra_viz_state = interp1(linspace(0,100,length(t_grid)), qra_grid, cycle, 'spline', 'extrap');
xp_viz_state = interp1(linspace(0,100,length(t_grid)), xp_grid, cycle, 'spline', 'extrap');
yp_viz_state = interp1(linspace(0,100,length(t_grid)), yp_grid, cycle, 'spline', 'extrap');
qtp_viz_state = interp1(linspace(0,100,length(t_grid)), qtp_grid, cycle, 'spline', 'extrap');
qlhp_viz_state = interp1(linspace(0,100,length(t_grid)), qlhp_grid, cycle, 'spline', 'extrap');
qrhp_viz_state = interp1(linspace(0,100,length(t_grid)), qrhp_grid, cycle, 'spline', 'extrap');
qlkp_viz_state = interp1(linspace(0,100,length(t_grid)), qlkp_grid, cycle, 'spline', 'extrap');
qrkp_viz_state = interp1(linspace(0,100,length(t_grid)), qrkp_grid, cycle, 'spline', 'extrap');
qlap_viz_state = interp1(linspace(0,100,length(t_grid)), qlap_grid, cycle, 'spline', 'extrap');
qrap_viz_state = interp1(linspace(0,100,length(t_grid)), qrap_grid, cycle, 'spline', 'extrap');
RESx_viz_state = interp1(linspace(0,100,length(t_grid)), RESx_grid, cycle, 'spline', 'extrap');
RESy_viz_state = interp1(linspace(0,100,length(t_grid)), RESy_grid, cycle, 'spline', 'extrap');
T_t_viz_state = interp1(linspace(0,100,length(t_grid)), T_t_grid, cycle, 'spline', 'extrap');
T_lh_viz_state = interp1(linspace(0,100,length(t_grid)), T_lh_grid, cycle, 'spline', 'extrap');
T_rh_viz_state = interp1(linspace(0,100,length(t_grid)), T_rh_grid, cycle, 'spline', 'extrap');
T_lk_viz_state = interp1(linspace(0,100,length(t_grid)), T_lk_grid, cycle, 'spline', 'extrap');
T_rk_viz_state = interp1(linspace(0,100,length(t_grid)), T_rk_grid, cycle, 'spline', 'extrap');
T_la_viz_state = interp1(linspace(0,100,length(t_grid)), T_la_grid, cycle, 'spline', 'extrap');
T_ra_viz_state = interp1(linspace(0,100,length(t_grid)), T_ra_grid, cycle, 'spline', 'extrap');
LGRFx_viz_state = interp1(linspace(0,100,length(t_grid)), LGRFx_control_grid, cycle, 'spline', 'extrap');
LGRFy_viz_state = interp1(linspace(0,100,length(t_grid)), LGRFy_control_grid, cycle, 'spline', 'extrap');
RGRFx_viz_state = interp1(linspace(0,100,length(t_grid)), RGRFx_control_grid, cycle, 'spline', 'extrap');
RGRFy_viz_state = interp1(linspace(0,100,length(t_grid)), RGRFy_control_grid, cycle, 'spline', 'extrap');

% Differentiate for higher order state values
xpp_viz_state = gradient(xp_viz_state, t_cycle);
ypp_viz_state = gradient(yp_viz_state, t_cycle);
qtpp_viz_state = gradient(qtp_viz_state, t_cycle);
qlhpp_viz_state = gradient(qlhp_viz_state, t_cycle);
qrhpp_viz_state = gradient(qrhp_viz_state, t_cycle);
qlkpp_viz_state = gradient(qlkp_viz_state, t_cycle);
qrkpp_viz_state = gradient(qrkp_viz_state, t_cycle);
qlapp_viz_state = gradient(qlap_viz_state, t_cycle);
qrapp_viz_state = gradient(qrap_viz_state, t_cycle);

% Calculate Contact Point Pose
lcontact_x_state = x_viz_state + L_TH.*sin(qlh_viz_state) + L_SH.*sin(qlh_viz_state+qlk_viz_state) + 0.5.*L_FT.*cos(qla_viz_state+qlh_viz_state+qlk_viz_state);
lcontact_y_state = y_viz_state - L_TH.*cos(qlh_viz_state) - L_SH.*cos(qlh_viz_state+qlk_viz_state) + 0.5.*L_FT.*sin(qla_viz_state+qlh_viz_state+qlk_viz_state);
rcontact_x_state = x_viz_state + L_TH.*sin(qrh_viz_state) + L_SH.*sin(qrh_viz_state+qrk_viz_state) + 0.5.*L_FT.*cos(qra_viz_state+qrh_viz_state+qrk_viz_state);
rcontact_y_state = y_viz_state - L_TH.*cos(qrh_viz_state) - L_SH.*cos(qrh_viz_state+qrk_viz_state) + 0.5.*L_FT.*sin(qra_viz_state+qrh_viz_state+qrk_viz_state);

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

% Calculate IMU State Profiles
torso_ax_viz_state = xpp_viz_state + LEN_HIP_TORSOIMU.*(sin(qt_viz_state).*qtp_viz_state.^2-cos(qt_viz_state).*qtpp_viz_state);
torso_ay_viz_state = ypp_viz_state - LEN_HIP_TORSOIMU.*(cos(qt_viz_state).*qtp_viz_state.^2+sin(qt_viz_state).*qtpp_viz_state);
torso_wz_viz_state = qtp_viz_state;
lthigh_ax_viz_state = xpp_viz_state - LEN_HIP_THIGHIMU.*(sin(qlh_viz_state).*qlhp_viz_state.^2-cos(qlh_viz_state).*qlhpp_viz_state);
lthigh_ay_viz_state = ypp_viz_state + LEN_HIP_THIGHIMU.*(cos(qlh_viz_state).*qlhp_viz_state.^2+sin(qlh_viz_state).*qlhpp_viz_state);
lthigh_wz_viz_state = qlhp_viz_state;
rthigh_ax_viz_state = xpp_viz_state - LEN_HIP_THIGHIMU.*(sin(qrh_viz_state).*qrhp_viz_state.^2-cos(qrh_viz_state).*qrhpp_viz_state);
rthigh_ay_viz_state = ypp_viz_state + LEN_HIP_THIGHIMU.*(cos(qrh_viz_state).*qrhp_viz_state.^2+sin(qrh_viz_state).*qrhpp_viz_state);
rthigh_wz_viz_state = qrhp_viz_state;
lshank_ax_viz_state = xpp_viz_state - L_TH.*(sin(qlh_viz_state).*qlhp_viz_state.^2-cos(qlh_viz_state).*qlhpp_viz_state) - LEN_KNEE_SHANKIMU.*(sin(qlh_viz_state+qlk_viz_state).*(qlhp_viz_state+qlkp_viz_state).^2-cos(qlh_viz_state+qlk_viz_state).*(qlhpp_viz_state+qlkpp_viz_state));
lshank_ay_viz_state = ypp_viz_state + L_TH.*(cos(qlh_viz_state).*qlhp_viz_state.^2+sin(qlh_viz_state).*qlhpp_viz_state) + LEN_KNEE_SHANKIMU.*(cos(qlh_viz_state+qlk_viz_state).*(qlhp_viz_state+qlkp_viz_state).^2+sin(qlh_viz_state+qlk_viz_state).*(qlhpp_viz_state+qlkpp_viz_state));
lshank_wz_viz_state = (qlhp_viz_state+qlkp_viz_state);
rshank_ax_viz_state = xpp_viz_state - L_TH.*(sin(qrh_viz_state).*qrhp_viz_state.^2-cos(qrh_viz_state).*qrhpp_viz_state) - LEN_KNEE_SHANKIMU.*(sin(qrh_viz_state+qrk_viz_state).*(qrh_viz_state+qrkp_viz_state).^2-cos(qrh_viz_state+qrk_viz_state).*(qrhpp_viz_state+qrkpp_viz_state));
rshank_ay_viz_state = ypp_viz_state + L_TH.*(cos(qrh_viz_state).*qrhp_viz_state.^2+sin(qrh_viz_state).*qrhpp_viz_state) + LEN_KNEE_SHANKIMU.*(cos(qrh_viz_state+qrk_viz_state).*(qrhp_viz_state+qrkp_viz_state).^2+sin(qrh_viz_state+qrk_viz_state).*(qrhpp_viz_state+qrkpp_viz_state));
rshank_wz_viz_state = (qrhp_viz_state+qrkp_viz_state);
lfoot_ax_viz_state = xpp_viz_state - L_TH.*(sin(qlh_viz_state).*qlhp_viz_state.^2-cos(qlh_viz_state).*qlhpp_viz_state) - L_SH.*(sin(qlh_viz_state+qlk_viz_state).*(qlhp_viz_state+qlkp_viz_state).^2-cos(qlh_viz_state+qlk_viz_state).*(qlhpp_viz_state+qlkpp_viz_state)) - LEN_ANKLE_FOOTIMU.*(cos(qla_viz_state+qlh_viz_state+qlk_viz_state).*(qlap_viz_state+qlhp_viz_state+qlkp_viz_state).^2+sin(qla_viz_state+qlh_viz_state+qlk_viz_state).*(qlapp_viz_state+qlhpp_viz_state+qlkpp_viz_state));
lfoot_ay_viz_state = ypp_viz_state + L_TH.*(cos(qlh_viz_state).*qlhp_viz_state.^2+sin(qlh_viz_state).*qlhpp_viz_state) + L_SH.*(cos(qlh_viz_state+qlk_viz_state).*(qlhp_viz_state+qlkp_viz_state).^2+sin(qlh_viz_state+qlk_viz_state).*(qlhpp_viz_state+qlkpp_viz_state)) - LEN_ANKLE_FOOTIMU.*(sin(qla_viz_state+qlh_viz_state+qlk_viz_state).*(qlap_viz_state+qlhp_viz_state+qlkp_viz_state).^2-cos(qla_viz_state+qlh_viz_state+qlk_viz_state).*(qlapp_viz_state+qlhpp_viz_state+qlkpp_viz_state));
lfoot_wz_viz_state = qlap_viz_state + (qlhp_viz_state+qlkp_viz_state);
rfoot_ax_viz_state = xpp_viz_state - L_TH.*(sin(qrh_viz_state).*qrhp_viz_state.^2-cos(qrh_viz_state).*qrhpp_viz_state) - L_SH.*(sin(qrh_viz_state+qrk_viz_state).*(qrhp_viz_state+qrkp_viz_state).^2-cos(qrh_viz_state+qrk_viz_state).*(qrhpp_viz_state+qrkpp_viz_state)) - LEN_ANKLE_FOOTIMU.*(cos(qra_viz_state+qrh_viz_state+qrk_viz_state).*(qrap_viz_state+qrhp_viz_state+qrkp_viz_state).^2+sin(qra_viz_state+qrh_viz_state+qrk_viz_state).*(qrapp_viz_state+qrhpp_viz_state+qrkpp_viz_state));
rfoot_ay_viz_state = ypp_viz_state + L_TH.*(cos(qrh_viz_state).*qrhp_viz_state.^2+sin(qrh_viz_state).*qrhpp_viz_state) + L_SH.*(cos(qrh_viz_state+qrk_viz_state).*(qrhp_viz_state+qrkp_viz_state).^2+sin(qrh_viz_state+qrk_viz_state).*(qrhpp_viz_state+qrkpp_viz_state)) - LEN_ANKLE_FOOTIMU.*(cos(qra_viz_state+qrh_viz_state+qrk_viz_state).*(qrap_viz_state+qrhp_viz_state+qrkp_viz_state).^2-cos(qra_viz_state+qrh_viz_state+qrk_viz_state).*(qrapp_viz_state+qrhpp_viz_state+qrkpp_viz_state));
rfoot_wz_viz_state = qrap_viz_state + (qrhp_viz_state+qrkp_viz_state);
grflx_viz = LGRFx_viz_state;
grfly_viz = LGRFy_viz_state;
grfrx_viz = RGRFx_viz_state;
grfry_viz = RGRFy_viz_state;

% Calculate Tracking Error Metrics
mean_pose_rmse_cm = 100.*mean([rms(proxtorso_x_viz_state - proxtorso_x_cycle'), rms(proxtorso_y_viz_state - proxtorso_y_cycle'), rms(hipjoint_x_viz_state - hipjoint_x_cycle'), rms(hipjoint_y_viz_state - hipjoint_y_cycle'), rms(lkneejoint_x_viz_state - lkneejoint_x_cycle'), rms(lkneejoint_y_viz_state - lkneejoint_y_cycle'), rms(rkneejoint_x_viz_state - rkneejoint_x_cycle'), rms(rkneejoint_y_viz_state - rkneejoint_y_cycle'), rms(lanklejoint_x_viz_state - lanklejoint_x_cycle'), rms(lanklejoint_y_viz_state - lanklejoint_y_cycle'), rms(ranklejoint_x_viz_state - ranklejoint_x_cycle'), rms(ranklejoint_y_viz_state - ranklejoint_y_cycle'), rms(ldistfoot_x_viz_state - ldistfoot_x_cycle'), rms(ldistfoot_y_viz_state - ldistfoot_y_cycle'), rms(rdistfoot_x_viz_state - rdistfoot_x_cycle'), rms(rdistfoot_y_viz_state - rdistfoot_y_cycle')])
std_pose_rmse_cm = 100.*std([rms(proxtorso_x_viz_state - proxtorso_x_cycle'), rms(proxtorso_y_viz_state - proxtorso_y_cycle'), rms(hipjoint_x_viz_state - hipjoint_x_cycle'), rms(hipjoint_y_viz_state - hipjoint_y_cycle'), rms(lkneejoint_x_viz_state - lkneejoint_x_cycle'), rms(lkneejoint_y_viz_state - lkneejoint_y_cycle'), rms(rkneejoint_x_viz_state - rkneejoint_x_cycle'), rms(rkneejoint_y_viz_state - rkneejoint_y_cycle'), rms(lanklejoint_x_viz_state - lanklejoint_x_cycle'), rms(lanklejoint_y_viz_state - lanklejoint_y_cycle'), rms(ranklejoint_x_viz_state - ranklejoint_x_cycle'), rms(ranklejoint_y_viz_state - ranklejoint_y_cycle'), rms(ldistfoot_x_viz_state - ldistfoot_x_cycle'), rms(ldistfoot_y_viz_state - ldistfoot_y_cycle'), rms(rdistfoot_x_viz_state - rdistfoot_x_cycle'), rms(rdistfoot_y_viz_state - rdistfoot_y_cycle')]);
mean_kin_rmse_degs = 180/pi .* mean([rms(qt_viz_state - qt_cycle'), rms(qlh_viz_state - qlh_cycle'), rms(qrh_viz_state - qrh_cycle'), rms(qlk_viz_state - qlk_cycle'), rms(qrk_viz_state - qrk_cycle'), rms(qla_viz_state - qla_cycle'), rms(qra_viz_state - qra_cycle')])
std_kin_rmse_degs = 180/pi .* std([rms(qt_viz_state - qt_cycle'), rms(qlh_viz_state - qlh_cycle'), rms(qrh_viz_state - qrh_cycle'), rms(qlk_viz_state - qlk_cycle'), rms(qrk_viz_state - qrk_cycle'), rms(qla_viz_state - qla_cycle'), rms(qra_viz_state - qra_cycle')]);

% Run Visualizer
VisualizeDirectCollocationSimulation