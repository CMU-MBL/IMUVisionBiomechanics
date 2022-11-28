%% OpenSimExampleDataToSyntheticIMUAndVisionData
% This script generates a cycle of synthetic IMU and keypoint data from
% ground truth trajectories extracted from OpenSim example data
seed = 1; desiredSNRdB = 15; desiredRMSE = 0.035;

% Set the random seed to control generation of synthetic data
rng(seed)

% Load in ground truth data from OpenSim format
ikData = ReadOpenSimData('9dof_ik.mot');
grfData = ReadOpenSimData('9dof_grf.mot');
idData = ReadOpenSimData('9dof_id.sto');

% Extract trajectories (DEGREES | METERS | NEWTONS)
t = ikData.time;
x = ikData.data(:,find(strcmp(string(ikData.labels),'pelvis_tx')));
y = ikData.data(:,find(strcmp(string(ikData.labels),'pelvis_ty')));
qt = ikData.data(:,find(strcmp(string(ikData.labels),'pelvis_tilt')));
qrh = ikData.data(:,find(strcmp(string(ikData.labels),'hip_flexion_r')));
qlh = ikData.data(:,find(strcmp(string(ikData.labels),'hip_flexion_l')));
qrk = ikData.data(:,find(strcmp(string(ikData.labels),'knee_angle_r')));
qlk = ikData.data(:,find(strcmp(string(ikData.labels),'knee_angle_l')));
qra = ikData.data(:,find(strcmp(string(ikData.labels),'ankle_angle_r')));
qla = ikData.data(:,find(strcmp(string(ikData.labels),'ankle_angle_l')));
grfTime = grfData.time;
grflx = grfData.data(:,find(strcmp(string(grfData.labels), '1_ground_force_vx')));
grfly = grfData.data(:,find(strcmp(string(grfData.labels), '1_ground_force_vy')));
grfrx = grfData.data(:,find(strcmp(string(grfData.labels), 'ground_force_vx')));
grfry = grfData.data(:,find(strcmp(string(grfData.labels), 'ground_force_vy')));
idTime = idData.time;
pelvtx = idData.data(:,find(strcmp(string(idData.labels), 'pelvis_tx_force')));
pelvty = idData.data(:,find(strcmp(string(idData.labels), 'pelvis_ty_force')));
pelvtilt = idData.data(:,find(strcmp(string(idData.labels), 'pelvis_tilt_moment')));
lhipmoment = idData.data(:,find(strcmp(string(idData.labels), 'hip_flexion_l_moment')));
rhipmoment = idData.data(:,find(strcmp(string(idData.labels), 'hip_flexion_r_moment')));
lkneemoment = idData.data(:,find(strcmp(string(idData.labels), 'knee_angle_l_moment')));
rkneemoment = idData.data(:,find(strcmp(string(idData.labels), 'knee_angle_r_moment')));
lanklemoment = idData.data(:,find(strcmp(string(idData.labels), 'ankle_angle_l_moment')));
ranklemoment = idData.data(:,find(strcmp(string(idData.labels), 'ankle_angle_r_moment')));

% Downsample so GRF and ID matches IK
grflx = interp1(grfTime,grflx,t, 'spline', 'extrap');
grfly = interp1(grfTime,grfly,t, 'spline', 'extrap');
grfrx = interp1(grfTime,grfrx,t, 'spline', 'extrap');
grfry = interp1(grfTime,grfry,t, 'spline', 'extrap');
pelvtx = interp1(idTime, pelvtx, t, 'spline', 'extrap');
pelvty = interp1(idTime, pelvty, t, 'spline', 'extrap');
pelvtilt = interp1(idTime, pelvtilt, t, 'spline', 'extrap');
lhipmoment = interp1(idTime, lhipmoment, t, 'spline', 'extrap');
rhipmoment = interp1(idTime, rhipmoment, t, 'spline', 'extrap');
lkneemoment = interp1(idTime, lkneemoment, t, 'spline', 'extrap');
rkneemoment = interp1(idTime, rkneemoment, t, 'spline', 'extrap');
lanklemoment = interp1(idTime, lanklemoment, t, 'spline', 'extrap');
ranklemoment = interp1(idTime, ranklemoment, t, 'spline', 'extrap');

% Convert from degrees to radians
qt = qt.*pi./180;
qrh = qrh.*pi./180;
qlh = qlh.*pi./180;
qrk = qrk.*pi./180;
qlk = qlk.*pi./180;
qra = qra.*pi./180;
qla = qla.*pi./180;

% Cut into single gait cycle
[~,locs] = findpeaks(qlh);
locs = [locs(2), locs(4)];
t = t(locs(1):locs(2));
t = t - t(1);
endPeriodIndex = length(t);
x = x(locs(1):locs(2));
y = y(locs(1):locs(2));
qt = qt(locs(1):locs(2));
qrh = qrh(locs(1):locs(2));
qlh = qlh(locs(1):locs(2));
qrk = qrk(locs(1):locs(2));
qlk = qlk(locs(1):locs(2));
qra = qra(locs(1):locs(2));
qla = qla(locs(1):locs(2));
grflx = grflx(locs(1):locs(2));
grfly = grfly(locs(1):locs(2));
grfrx = grfrx(locs(1):locs(2));
grfry = grfry(locs(1):locs(2));
pelvtx = pelvtx(locs(1):locs(2));
pelvty = pelvty(locs(1):locs(2));
pelvtilt = pelvtilt(locs(1):locs(2));
lhipmoment = lhipmoment(locs(1):locs(2));
rhipmoment = rhipmoment(locs(1):locs(2));
lkneemoment = lkneemoment(locs(1):locs(2));
rkneemoment = rkneemoment(locs(1):locs(2));
lanklemoment = lanklemoment(locs(1):locs(2));
ranklemoment = ranklemoment(locs(1):locs(2));

% Shift X-COM by treadmill velocity (1 m/s) for overground walking
xp = gradient(x, t);
xp = xp + ones(size(xp));
x = cumtrapz(t,xp);

% Compute derivatives
xp = gradient(x,t);
xpp = gradient(xp,t);
yp = gradient(y,t);
ypp = gradient(yp,t);
qtp = gradient(qt,t);
qtpp = gradient(qtp,t);
qrhp = gradient(qrh,t);
qrhpp = gradient(qrhp,t);
qlhp = gradient(qlh,t);
qlhpp = gradient(qlhp,t);
qrkp = gradient(qrk,t);
qrkpp = gradient(qrkp,t);
qlkp = gradient(qlk,t);
qlkpp = gradient(qlkp,t);
qrap = gradient(qra,t);
qrapp = gradient(qrap,t);
qlap = gradient(qla,t);
qlapp = gradient(qlap,t);

% Compute Synthetic IMU Accelerations and AngVels Without Noise
torso_ax = xpp + LEN_HIP_TORSOIMU.*(sin(qt).*qtp.^2-cos(qt).*qtpp);
torso_ay = ypp - LEN_HIP_TORSOIMU.*(cos(qt).*qtp.^2+sin(qt).*qtpp);
torso_wz = qtp;
lthigh_ax = xpp - LEN_HIP_THIGHIMU.*(sin(qlh).*qlhp.^2-cos(qlh).*qlhpp);
lthigh_ay = ypp + LEN_HIP_THIGHIMU.*(cos(qlh).*qlhp.^2+sin(qlh).*qlhpp);
lthigh_wz = qlhp;
rthigh_ax = xpp - LEN_HIP_THIGHIMU.*(sin(qrh).*qrhp.^2-cos(qrh).*qrhpp);
rthigh_ay = ypp + LEN_HIP_THIGHIMU.*(cos(qrh).*qrhp.^2+sin(qrh).*qrhpp);
rthigh_wz = qrhp;
lshank_ax = xpp - L_TH.*(sin(qlh).*qlhp.^2-cos(qlh).*qlhpp) - LEN_KNEE_SHANKIMU.*(sin(qlh+qlk).*(qlhp+qlkp).^2-cos(qlh+qlk).*(qlhpp+qlkpp));
lshank_ay = ypp + L_TH.*(cos(qlh).*qlhp.^2+sin(qlh).*qlhpp) + LEN_KNEE_SHANKIMU.*(cos(qlh+qlk).*(qlhp+qlkp).^2+sin(qlh+qlk).*(qlhpp+qlkpp));
lshank_wz = (qlhp+qlkp);
rshank_ax = xpp - L_TH.*(sin(qrh).*qrhp.^2-cos(qrh).*qrhpp) - LEN_KNEE_SHANKIMU.*(sin(qrh+qrk).*(qrhp+qrkp).^2-cos(qrh+qrk).*(qrhpp+qrkpp));
rshank_ay = ypp + L_TH.*(cos(qrh).*qrhp.^2+sin(qrh).*qrhpp) + LEN_KNEE_SHANKIMU.*(cos(qrh+qrk).*(qrhp+qrkp).^2+sin(qrh+qrk).*(qrhpp+qrkpp));
rshank_wz = (qrhp+qrkp);
lfoot_ax = xpp - L_TH.*(sin(qlh).*qlhp.^2-cos(qlh).*qlhpp) - L_SH.*(sin(qlh+qlk).*(qlhp+qlkp).^2-cos(qlh+qlk).*(qlhpp+qlkpp)) - LEN_ANKLE_FOOTIMU.*(cos(qla+qlh+qlk).*(qlap+qlhp+qlkp).^2+sin(qla+qlh+qlk).*(qlapp+qlhpp+qlkpp));
lfoot_ay = ypp + L_TH.*(cos(qlh).*qlhp.^2+sin(qlh).*qlhpp) + L_SH.*(cos(qlh+qlk).*(qlhp+qlkp).^2+sin(qlh+qlk).*(qlhpp+qlkpp)) - LEN_ANKLE_FOOTIMU.*(sin(qla+qlh+qlk).*(qlap+qlhp+qlkp).^2-cos(qla+qlh+qlk).*(qlapp+qlhpp+qlkpp));
lfoot_wz = qlap + (qlhp+qlkp);
rfoot_ax = xpp - L_TH.*(sin(qrh).*qrhp.^2-cos(qrh).*qrhpp) - L_SH.*(sin(qrh+qrk).*(qrhp+qrkp).^2-cos(qrh+qrk).*(qrhpp+qrkpp)) - LEN_ANKLE_FOOTIMU.*(cos(qra+qrh+qrk).*(qrap+qrhp+qrkp).^2+sin(qra+qrh+qrk).*(qrapp+qrhpp+qrkpp));
rfoot_ay = ypp + L_TH.*(cos(qrh).*qrhp.^2+sin(qrh).*qrhpp) + L_SH.*(cos(qrh+qrk).*(qrhp+qrkp).^2+sin(qrh+qrk).*(qrhpp+qrkpp)) - LEN_ANKLE_FOOTIMU.*(sin(qra+qrh+qrk).*(qrap+qrhp+qrkp).^2-cos(qra+qrh+qrk).*(qrapp+qrhpp+qrkpp));
rfoot_wz = qrap + (qrhp+qrkp);

% Gaussian Noise Components for the Synthetic IMU
randNoise_torso_ax = randn(size(t));
randNoise_torso_ay = randn(size(t));
randNoise_torso_wz = randn(size(t));
randNoise_lthigh_ax = randn(size(t));
randNoise_lthigh_ay = randn(size(t));
randNoise_lthigh_wz = randn(size(t));
randNoise_rthigh_ax = randn(size(t));
randNoise_rthigh_ay = randn(size(t));
randNoise_rthigh_wz = randn(size(t));
randNoise_lshank_ax = randn(size(t));
randNoise_lshank_ay = randn(size(t));
randNoise_lshank_wz = randn(size(t));
randNoise_rshank_ax = randn(size(t));
randNoise_rshank_ay = randn(size(t));
randNoise_rshank_wz = randn(size(t));
randNoise_lfoot_ax = randn(size(t));
randNoise_lfoot_ay = randn(size(t));
randNoise_lfoot_wz = randn(size(t));
randNoise_rfoot_ax = randn(size(t));
randNoise_rfoot_ay = randn(size(t));
randNoise_rfoot_wz = randn(size(t));

% Bias factor and scale factor noise following from Park & Gao (2008)
maxTorsoAxMag = abs(max(torso_ax));
maxTorsoAyMag = abs(max(torso_ay));
maxTorsoWzMag = abs(max(torso_wz));
maxLThighAxMag = abs(max(lthigh_ax));
maxLThighAyMag = abs(max(lthigh_ay));
maxLThighWzMag = abs(max(lthigh_wz));
maxRThighAxMag = abs(max(rthigh_ax));
maxRThighAyMag = abs(max(rthigh_ay));
maxRThighWzMag = abs(max(rthigh_wz));
maxLShankAxMag = abs(max(lshank_ax));
maxLShankAyMag = abs(max(lshank_ay));
maxLShankWzMag = abs(max(lshank_wz));
maxRShankAxMag = abs(max(rshank_ax));
maxRShankAyMag = abs(max(rshank_ay));
maxRShankWzMag = abs(max(rshank_wz));
maxLFootAxMag = abs(max(lfoot_ax));
maxLFootAyMag = abs(max(lfoot_ay));
maxLFootWzMag = abs(max(lfoot_wz));
maxRFootAxMag = abs(max(rfoot_ax));
maxRFootAyMag = abs(max(rfoot_ay));
maxRFootWzMag = abs(max(rfoot_wz));
B_TorsoAxMag = normrnd(2.3961e-3*9.81/9.81*maxTorsoAxMag,0.8858e-3*9.81/9.81*maxTorsoAxMag,size(t));
B_TorsoAyMag = normrnd(2.3961e-3*9.81/9.81*maxTorsoAyMag,0.8858e-3*9.81/9.81*maxTorsoAyMag,size(t));
B_TorsoWzMag = normrnd(0.2813/80*maxTorsoWzMag ,0.0407/80*maxTorsoWzMag,size(t));
B_LThighAxMag = normrnd(2.3961e-3*9.81/9.81*maxLThighAxMag,0.8858e-3*9.81/9.81*maxLThighAxMag,size(t));
B_LThighAyMag = normrnd(2.3961e-3*9.81/9.81*maxLThighAyMag,0.8858e-3*9.81/9.81*maxLThighAyMag,size(t));
B_LThighWzMag = normrnd(0.2813/80*maxLThighWzMag,0.0407/80*maxLThighWzMag,size(t));
B_RThighAxMag = normrnd(2.3961e-3*9.81/9.81*maxRThighAxMag,0.8858e-3*9.81/9.81*maxRThighAxMag,size(t));
B_RThighAyMag = normrnd(2.3961e-3*9.81/9.81*maxRThighAyMag,0.8858e-3*9.81/9.81*maxRThighAyMag,size(t));
B_RThighWzMag = normrnd(0.2813/80*maxRThighWzMag,0.0407/80*maxRThighWzMag,size(t));
B_LShankAxMag = normrnd(2.3961e-3*9.81/9.81*maxLShankAxMag,0.8858e-3*9.81/9.81*maxLShankAxMag,size(t));
B_LShankAyMag = normrnd(2.3961e-3*9.81/9.81*maxLShankAyMag,0.8858e-3*9.81/9.81*maxLShankAyMag,size(t));
B_LShankWzMag = normrnd(0.2813/80*maxLShankWzMag,0.0407/80*maxLShankWzMag,size(t));
B_RShankAxMag = normrnd(2.3961e-3*9.81/9.81*maxRShankAxMag,0.8858e-3*9.81/9.81*maxRShankAxMag,size(t));
B_RShankAyMag = normrnd(2.3961e-3*9.81/9.81*maxRShankAyMag,0.8858e-3*9.81/9.81*maxRShankAyMag,size(t));
B_RShankWzMag = normrnd(0.2813/80*maxRShankWzMag,0.0407/80*maxRShankWzMag,size(t));
B_LFootAxMag = normrnd(2.3961e-3*9.81/9.81*maxLFootAxMag,0.8858e-3*9.81/9.81*maxLFootAxMag,size(t));
B_LFootAyMag = normrnd(2.3961e-3*9.81/9.81*maxLFootAyMag,0.8858e-3*9.81/9.81*maxLFootAyMag,size(t));
B_LFootWzMag = normrnd(0.2813/80*maxLFootWzMag,0.0407/80*maxLFootWzMag,size(t));
B_RFootAxMag = normrnd(2.3961e-3*9.81/9.81*maxRFootAxMag,0.8858e-3*9.81/9.81*maxRFootAxMag,size(t));
B_RFootAyMag = normrnd(2.3961e-3*9.81/9.81*maxRFootAyMag,0.8858e-3*9.81/9.81*maxRFootAyMag,size(t));
B_RFootWzMag = normrnd(0.2813/80*maxRFootWzMag,0.0407/80*maxRFootWzMag,size(t));
S_TorsoAxMag = normrnd(-0.093e-2,0.0358e-2, size(t));
S_TorsoAyMag = normrnd(-0.093e-2,0.0358e-2, size(t));
S_TorsoWzMag = normrnd(-0.3984e-2,0.0449e-2,size(t));
S_LThighAxMag = normrnd(-0.093e-2,0.0358e-2, size(t));
S_LThighAyMag = normrnd(-0.093e-2,0.0358e-2, size(t));
S_LThighWzMag = normrnd(-0.3984e-2,0.0449e-2,size(t));
S_RThighAxMag = normrnd(-0.093e-2,0.0358e-2, size(t));
S_RThighAyMag = normrnd(-0.093e-2,0.0358e-2, size(t));
S_RThighWzMag = normrnd(-0.3984e-2,0.0449e-2,size(t));
S_LShankAxMag = normrnd(-0.093e-2,0.0358e-2, size(t));
S_LShankAyMag = normrnd(-0.093e-2,0.0358e-2, size(t));
S_LShankWzMag = normrnd(-0.3984e-2,0.0449e-2,size(t));
S_RShankAxMag = normrnd(-0.093e-2,0.0358e-2, size(t));
S_RShankAyMag = normrnd(-0.093e-2,0.0358e-2, size(t));
S_RShankWzMag =  normrnd(-0.3984e-2,0.0449e-2,size(t));
S_LFootAxMag = normrnd(-0.093e-2,0.0358e-2, size(t));
S_LFootAyMag = normrnd(-0.093e-2,0.0358e-2, size(t));
S_LFootWzMag =  normrnd(-0.3984e-2,0.0449e-2,size(t));
S_RFootAxMag = normrnd(-0.093e-2,0.0358e-2, size(t));
S_RFootAyMag = normrnd(-0.093e-2,0.0358e-2, size(t));
S_RFootWzMag =  normrnd(-0.3984e-2,0.0449e-2,size(t));

% Compute base 10 SNR
% Do adjustment because thse models underestimate IMU noise challenges
% compared to experimental data when tracking by quite a bit
desiredSNR = 10^( (desiredSNRdB - 5) /10); 

% Add Noise and Tune the Guassian Noise to be of the right SNR
fun = @(x) findAmplificationSNR(x, torso_ax, randNoise_torso_ax, desiredSNR);
[amplification_torso_ax, error_torso_ax] = fminsearch(fun,1);
actualSNR_torso_ax = ( (rms(torso_ax)^2 / rms(amplification_torso_ax*randNoise_torso_ax)^2 ) );
torso_ax_imu = torso_ax + S_TorsoAxMag.*torso_ax + B_TorsoAxMag + amplification_torso_ax .* randNoise_torso_ax;

fun = @(x) findAmplificationSNR(x, torso_ay, randNoise_torso_ay, desiredSNR);
[amplification_torso_ay, error_torso_ay] = fminsearch(fun,1);
actualSNR_torso_ay = ( (rms(torso_ay)^2 / rms(amplification_torso_ay*randNoise_torso_ay)^2 ) );
torso_ay_imu = torso_ay + S_TorsoAyMag.*torso_ay + B_TorsoAyMag + amplification_torso_ay .* randNoise_torso_ay;

fun = @(x) findAmplificationSNR(x, torso_wz, randNoise_torso_wz, desiredSNR);
[amplification_torso_wz, error_torso_wz] = fminsearch(fun,1);
actualSNR_torso_wz = ( (rms(torso_wz)^2 / rms(amplification_torso_wz*randNoise_torso_wz)^2 ) );
torso_wz_imu = torso_wz + S_TorsoWzMag.*torso_wz + B_TorsoWzMag + amplification_torso_wz .* randNoise_torso_wz;

fun = @(x) findAmplificationSNR(x, lthigh_ax, randNoise_lthigh_ax, desiredSNR);
[amplification_lthigh_ax, error_lthigh_ax] = fminsearch(fun,1);
actualSNR_lthigh_ax = ( (rms(lthigh_ax)^2 / rms(amplification_lthigh_ax*randNoise_lthigh_ax)^2 ) );
lthigh_ax_imu = lthigh_ax + S_LThighAxMag.*lthigh_ax + B_LThighAxMag + amplification_lthigh_ax .* randNoise_lthigh_ax;

fun = @(x) findAmplificationSNR(x, lthigh_ay, randNoise_lthigh_ay, desiredSNR);
[amplification_lthigh_ay, error_lthigh_ay] = fminsearch(fun,1);
actualSNR_lthigh_ay = ( (rms(lthigh_ay)^2 / rms(amplification_lthigh_ay*randNoise_lthigh_ay)^2 ) );
lthigh_ay_imu = lthigh_ay + S_LThighAyMag.*lthigh_ay + B_LThighAyMag + amplification_lthigh_ay .* randNoise_lthigh_ay;

fun = @(x) findAmplificationSNR(x, lthigh_wz, randNoise_lthigh_wz, desiredSNR);
[amplification_lthigh_wz, error_lthigh_wz] = fminsearch(fun,1);
actualSNR_lthigh_wz = ( (rms(lthigh_wz)^2 / rms(amplification_lthigh_wz*randNoise_lthigh_wz)^2 ) );
lthigh_wz_imu = lthigh_wz + S_LThighWzMag.*lthigh_wz + B_LThighWzMag + amplification_lthigh_wz .* randNoise_lthigh_wz;

fun = @(x) findAmplificationSNR(x, rthigh_ax, randNoise_rthigh_ax, desiredSNR);
[amplification_rthigh_ax, error_rthigh_ax] = fminsearch(fun,1);
actualSNR_rthigh_ax = ( (rms(rthigh_ax)^2 / rms(amplification_rthigh_ax*randNoise_rthigh_ax)^2 ) );
rthigh_ax_imu = rthigh_ax + S_RThighAxMag.*rthigh_ax + B_RThighAxMag + amplification_rthigh_ax .* randNoise_rthigh_ax;

fun = @(x) findAmplificationSNR(x, rthigh_ay, randNoise_rthigh_ay, desiredSNR);
[amplification_rthigh_ay, error_rthigh_ay] = fminsearch(fun,1);
actualSNR_rthigh_ay = ( (rms(rthigh_ay)^2 / rms(amplification_rthigh_ay*randNoise_rthigh_ay)^2 ) );
rthigh_ay_imu = rthigh_ay + S_RThighAyMag.*rthigh_ay + B_RThighAyMag + amplification_rthigh_ay .* randNoise_rthigh_ay;

fun = @(x) findAmplificationSNR(x, rthigh_wz, randNoise_rthigh_wz, desiredSNR);
[amplification_rthigh_wz, error_rthigh_wz] = fminsearch(fun,1);
actualSNR_rthigh_wz = ( (rms(rthigh_wz)^2 / rms(amplification_rthigh_wz*randNoise_rthigh_wz)^2 ) );
rthigh_wz_imu = rthigh_wz + S_RThighWzMag.*rthigh_wz + B_RThighWzMag + amplification_rthigh_wz .* randNoise_rthigh_wz;

fun = @(x) findAmplificationSNR(x, lshank_ax, randNoise_lshank_ax, desiredSNR);
[amplification_lshank_ax, error_lshank_ax] = fminsearch(fun,1);
actualSNR_lshank_ax = ( (rms(lshank_ax)^2 / rms(amplification_lshank_ax*randNoise_lshank_ax)^2 ) );
lshank_ax_imu = lshank_ax + S_LShankAxMag.*lshank_ax + B_LShankAxMag + amplification_lshank_ax .* randNoise_lshank_ax;

fun = @(x) findAmplificationSNR(x, lshank_ay, randNoise_lshank_ay, desiredSNR);
[amplification_lshank_ay, error_lshank_ay] = fminsearch(fun,1);
actualSNR_lshank_ay = ( (rms(lshank_ay)^2 / rms(amplification_lshank_ay*randNoise_lshank_ay)^2 ) );
lshank_ay_imu = lshank_ay + S_LShankAyMag.*lshank_ay + B_LShankAyMag + amplification_lshank_ay .* randNoise_lshank_ay;

fun = @(x) findAmplificationSNR(x, lshank_wz, randNoise_lshank_wz, desiredSNR);
[amplification_lshank_wz, error_lshank_wz] = fminsearch(fun,1);
actualSNR_lshank_wz = ( (rms(lshank_wz)^2 / rms(amplification_lshank_wz*randNoise_lshank_wz)^2 ) );
lshank_wz_imu = lshank_wz + S_LShankWzMag.*lshank_wz + B_LShankWzMag + amplification_lshank_wz .* randNoise_lshank_wz;

fun = @(x) findAmplificationSNR(x, rshank_ax, randNoise_rshank_ax, desiredSNR);
[amplification_rshank_ax, error_rshank_ax] = fminsearch(fun,1);
actualSNR_rshank_ax = ( (rms(rshank_ax)^2 / rms(amplification_rshank_ax*randNoise_rshank_ax)^2 ) );
rshank_ax_imu = rshank_ax + S_RShankAxMag.*rshank_ax + B_RShankAxMag + amplification_rshank_ax .* randNoise_rshank_ax;

fun = @(x) findAmplificationSNR(x, rshank_ay, randNoise_rshank_ay, desiredSNR);
[amplification_rshank_ay, error_rshank_ay] = fminsearch(fun,1);
actualSNR_rshank_ay = ( (rms(rshank_ay)^2 / rms(amplification_rshank_ay*randNoise_rshank_ay)^2 ) );
rshank_ay_imu = rshank_ay + S_RShankAyMag.*rshank_ay + B_RShankAyMag + amplification_rshank_ay .* randNoise_rshank_ay;

fun = @(x) findAmplificationSNR(x, rshank_wz, randNoise_rshank_wz, desiredSNR);
[amplification_rshank_wz, error_rshank_wz] = fminsearch(fun,1);
actualSNR_rshank_wz = ( (rms(rshank_wz)^2 / rms(amplification_rshank_wz*randNoise_rshank_wz)^2 ) );
rshank_wz_imu = rshank_wz + S_RShankWzMag.*rshank_wz + B_RShankWzMag + amplification_rshank_wz .* randNoise_rshank_wz;

fun = @(x) findAmplificationSNR(x, lfoot_ax, randNoise_lfoot_ax, desiredSNR);
[amplification_lfoot_ax, error_lfoot_ax] = fminsearch(fun,1);
actualSNR_lfoot_ax = ( (rms(lfoot_ax)^2 / rms(amplification_lfoot_ax*randNoise_lfoot_ax)^2 ) );
lfoot_ax_imu = lfoot_ax + S_LFootAxMag.*lfoot_ax + B_LFootAxMag + amplification_lfoot_ax .* randNoise_lfoot_ax;

fun = @(x) findAmplificationSNR(x, lfoot_ay, randNoise_lfoot_ay, desiredSNR);
[amplification_lfoot_ay, error_lfoot_ay] = fminsearch(fun,1);
actualSNR_lfoot_ay = ( (rms(lfoot_ay)^2 / rms(amplification_lfoot_ay*randNoise_lfoot_ay)^2 ) );
lfoot_ay_imu = lfoot_ay + S_LFootAyMag.*lfoot_ay + B_LFootAyMag + amplification_lfoot_ay .* randNoise_lfoot_ay;

fun = @(x) findAmplificationSNR(x, lfoot_wz, randNoise_lfoot_wz, desiredSNR);
[amplification_lfoot_wz, error_lfoot_wz] = fminsearch(fun,1);
actualSNR_lfoot_wz = ( (rms(lfoot_wz)^2 / rms(amplification_lfoot_wz*randNoise_lfoot_wz)^2 ) );
lfoot_wz_imu = lfoot_wz + S_LFootWzMag.*lfoot_wz + B_LFootWzMag + amplification_lfoot_wz .* randNoise_lfoot_wz;

fun = @(x) findAmplificationSNR(x, rfoot_ax, randNoise_rfoot_ax, desiredSNR);
[amplification_rfoot_ax, error_rfoot_ax] = fminsearch(fun,1);
actualSNR_rfoot_ax = ( (rms(rfoot_ax)^2 / rms(amplification_rfoot_ax*randNoise_rfoot_ax)^2 ) );
rfoot_ax_imu = rfoot_ax + S_RFootAxMag.*rfoot_ax + B_RFootAxMag + amplification_rfoot_ax .* randNoise_rfoot_ax;

fun = @(x) findAmplificationSNR(x, rfoot_ay, randNoise_rfoot_ay, desiredSNR);
[amplification_rfoot_ay, error_rfoot_ay] = fminsearch(fun,1);
actualSNR_rfoot_ay = ( (rms(rfoot_ay)^2 / rms(amplification_rfoot_ay*randNoise_rfoot_ay)^2 ) );
rfoot_ay_imu = rfoot_ay + S_RFootAyMag.*rfoot_ay + B_RFootAyMag + amplification_rfoot_ay .* randNoise_rfoot_ay;

fun = @(x) findAmplificationSNR(x, rfoot_wz, randNoise_rfoot_wz, desiredSNR);
[amplification_rfoot_wz, error_rfoot_wz] = fminsearch(fun,1);
actualSNR_rfoot_wz = ( (rms(rfoot_wz)^2 / rms(amplification_rfoot_wz*randNoise_rfoot_wz)^2 ) );
rfoot_wz_imu = rfoot_wz + S_RFootWzMag.*rfoot_wz + B_RFootWzMag + amplification_rfoot_wz .* randNoise_rfoot_wz;

% Calculate Keypoint Locations Without Noise
proxtorso_x = x - L_T.*sin(qt);
proxtorso_y = y + L_T.*cos(qt);
hipjoint_x = x;
hipjoint_y = y;
lkneejoint_x = x + L_TH.*sin(qlh);
lkneejoint_y = y - L_TH.*cos(qlh);
rkneejoint_x = x + L_TH.*sin(qrh);
rkneejoint_y = y - L_TH.*cos(qrh);
lanklejoint_x = x + L_TH.*sin(qlh) + L_SH.*sin(qlh+qlk);
lanklejoint_y = y - L_TH.*cos(qlh) - L_SH.*cos(qlh+qlk);
ranklejoint_x = x + L_TH.*sin(qrh) + L_SH.*sin(qrh+qrk);
ranklejoint_y = y - L_TH.*cos(qrh) - L_SH.*cos(qrh+qrk);
ldistfoot_x = x + L_TH.*sin(qlh) + L_SH.*sin(qlh+qlk) + L_FT.*cos(qla+qlh+qlk);
ldistfoot_y = y - L_TH.*cos(qlh) - L_SH.*cos(qlh+qlk) + L_FT.*sin(qla+qlh+qlk);
rdistfoot_x = x + L_TH.*sin(qrh) + L_SH.*sin(qrh+qrk) + L_FT.*cos(qra+qrh+qrk);
rdistfoot_y = y - L_TH.*cos(qrh) - L_SH.*cos(qrh+qrk) + L_FT.*sin(qra+qrh+qrk);

randNoise_proxtorso_x = randn(size(t));
randNoise_proxtorso_y = randn(size(t));
randNoise_hipjoint_x = randn(size(t));
randNoise_hipjoint_y = randn(size(t));
randNoise_lkneejoint_x = randn(size(t));
randNoise_lkneejoint_y = randn(size(t));
randNoise_rkneejoint_x = randn(size(t));
randNoise_rkneejoint_y = randn(size(t));
randNoise_lanklejoint_x = randn(size(t));
randNoise_lanklejoint_y = randn(size(t));
randNoise_ranklejoint_x = randn(size(t));
randNoise_ranklejoint_y = randn(size(t));
randNoise_ldistfoot_x = randn(size(t));
randNoise_ldistfoot_y = randn(size(t));
randNoise_rdistfoot_x = randn(size(t));
randNoise_rdistfoot_y = randn(size(t));

% Add Noise and Optimize for Necessary Amplification
fun = @(z) findAmplificationRMSE(z, proxtorso_x, randNoise_proxtorso_x, desiredRMSE);
[amplification_proxtorso_x, error_proxtorso_x] = fminsearch(fun,1);
actualRMSE_proxtorso_x = rms(proxtorso_x - (proxtorso_x + amplification_proxtorso_x.*randNoise_proxtorso_x));
proxtorso_x_keypoint = proxtorso_x + amplification_proxtorso_x .* randNoise_proxtorso_x;

fun = @(z) findAmplificationRMSE(z, proxtorso_y, randNoise_proxtorso_y, desiredRMSE);
[amplification_proxtorso_y, error_proxtorso_y] = fminsearch(fun,1);
actualRMSE_proxtorso_y = rms(proxtorso_y - (proxtorso_y + amplification_proxtorso_y.*randNoise_proxtorso_y));
proxtorso_y_keypoint = proxtorso_y + amplification_proxtorso_y .* randNoise_proxtorso_y;

fun = @(z) findAmplificationRMSE(z, hipjoint_x, randNoise_hipjoint_x, desiredRMSE);
[amplification_hipjoint_x, error_hipjoint_x] = fminsearch(fun,1);
actualRMSE_hipjoint_x = rms(hipjoint_x - (hipjoint_x + amplification_hipjoint_x.*randNoise_hipjoint_x));
hipjoint_x_keypoint = hipjoint_x + amplification_hipjoint_x .* randNoise_hipjoint_x;

fun = @(z) findAmplificationRMSE(z, hipjoint_y, randNoise_hipjoint_y, desiredRMSE);
[amplification_hipjoint_y, error_hipjoint_y] = fminsearch(fun,1);
actualRMSE_hipjoint_y = rms(hipjoint_y - (hipjoint_y + amplification_hipjoint_y.*randNoise_hipjoint_y));
hipjoint_y_keypoint = hipjoint_y + amplification_hipjoint_y .* randNoise_hipjoint_y;

fun = @(z) findAmplificationRMSE(z, lkneejoint_x, randNoise_lkneejoint_x, desiredRMSE);
[amplification_lkneejoint_x, error_lkneejoint_x] = fminsearch(fun,1);
actualRMSE_lkneejoint_x = rms(lkneejoint_x - (lkneejoint_x + amplification_lkneejoint_x.*randNoise_lkneejoint_x));
lkneejoint_x_keypoint = lkneejoint_x + amplification_lkneejoint_x .* randNoise_lkneejoint_x;

fun = @(z) findAmplificationRMSE(z, lkneejoint_y, randNoise_lkneejoint_y, desiredRMSE);
[amplification_lkneejoint_y, error_lkneejoint_y] = fminsearch(fun,1);
actualRMSE_lkneejoint_y = rms(lkneejoint_y - (lkneejoint_y + amplification_lkneejoint_y.*randNoise_lkneejoint_y));
lkneejoint_y_keypoint = lkneejoint_y + amplification_lkneejoint_y .* randNoise_lkneejoint_y;

fun = @(z) findAmplificationRMSE(z, rkneejoint_x, randNoise_rkneejoint_x, desiredRMSE);
[amplification_rkneejoint_x, error_rkneejoint_x] = fminsearch(fun,1);
actualRMSE_rkneejoint_x = rms(rkneejoint_x - (rkneejoint_x + amplification_rkneejoint_x.*randNoise_rkneejoint_x));
rkneejoint_x_keypoint = rkneejoint_x + amplification_rkneejoint_x .* randNoise_rkneejoint_x;

fun = @(z) findAmplificationRMSE(z, rkneejoint_y, randNoise_rkneejoint_y, desiredRMSE);
[amplification_rkneejoint_y, error_rkneejoint_y] = fminsearch(fun,1);
actualRMSE_rkneejoint_y = rms(rkneejoint_y - (rkneejoint_y + amplification_rkneejoint_y.*randNoise_rkneejoint_y));
rkneejoint_y_keypoint = rkneejoint_y + amplification_rkneejoint_y .* randNoise_rkneejoint_y;

fun = @(z) findAmplificationRMSE(z, lanklejoint_x, randNoise_lanklejoint_x, desiredRMSE);
[amplification_lanklejoint_x, error_lanklejoint_x] = fminsearch(fun,1);
actualRMSE_lanklejoint_x = rms(lanklejoint_x - (lanklejoint_x + amplification_lanklejoint_x.*randNoise_lanklejoint_x));
lanklejoint_x_keypoint = lanklejoint_x + amplification_lanklejoint_x .* randNoise_lanklejoint_x;

fun = @(z) findAmplificationRMSE(z, lanklejoint_y, randNoise_lanklejoint_y, desiredRMSE);
[amplification_lanklejoint_y, error_lanklejoint_y] = fminsearch(fun,1);
actualRMSE_lanklejoint_y = rms(lanklejoint_y - (lanklejoint_y + amplification_lanklejoint_y.*randNoise_lanklejoint_y));
lanklejoint_y_keypoint = lanklejoint_y + amplification_lanklejoint_y .* randNoise_lanklejoint_y;

fun = @(z) findAmplificationRMSE(z, ranklejoint_x, randNoise_ranklejoint_x, desiredRMSE);
[amplification_ranklejoint_x, error_ranklejoint_x] = fminsearch(fun,1);
actualRMSE_ranklejoint_x = rms(ranklejoint_x - (ranklejoint_x + amplification_ranklejoint_x.*randNoise_ranklejoint_x));
ranklejoint_x_keypoint = ranklejoint_x + amplification_ranklejoint_x .* randNoise_ranklejoint_x;

fun = @(z) findAmplificationRMSE(z, ranklejoint_y, randNoise_ranklejoint_y, desiredRMSE);
[amplification_ranklejoint_y, error_ranklejoint_y] = fminsearch(fun,1);
actualRMSE_ranklejoint_y = rms(ranklejoint_y - (ranklejoint_y + amplification_ranklejoint_y.*randNoise_ranklejoint_y));
ranklejoint_y_keypoint = ranklejoint_y + amplification_ranklejoint_y .* randNoise_ranklejoint_y;

fun = @(z) findAmplificationRMSE(z, ldistfoot_x, randNoise_ldistfoot_x, desiredRMSE);
[amplification_ldistfoot_x, error_ldistfoot_x] = fminsearch(fun,1);
actualRMSE_ldistfoot_x = rms(ldistfoot_x - (ldistfoot_x + amplification_ldistfoot_x.*randNoise_ldistfoot_x));
ldistfoot_x_keypoint = ldistfoot_x + amplification_ldistfoot_x .* randNoise_ldistfoot_x;

fun = @(z) findAmplificationRMSE(z, ldistfoot_y, randNoise_ldistfoot_y, desiredRMSE);
[amplification_ldistfoot_y, error_ldistfoot_y] = fminsearch(fun,1);
actualRMSE_ldistfoot_y = rms(ldistfoot_y - (ldistfoot_y + amplification_ldistfoot_y.*randNoise_ldistfoot_y));
ldistfoot_y_keypoint = ldistfoot_y + amplification_ldistfoot_y .* randNoise_ldistfoot_y;

fun = @(z) findAmplificationRMSE(z, rdistfoot_x, randNoise_rdistfoot_x, desiredRMSE);
[amplification_rdistfoot_x, error_rdistfoot_x] = fminsearch(fun,1);
actualRMSE_rdistfoot_x = rms(rdistfoot_x - (rdistfoot_x + amplification_rdistfoot_x.*randNoise_rdistfoot_x));
rdistfoot_x_keypoint = rdistfoot_x + amplification_rdistfoot_x .* randNoise_rdistfoot_x;

fun = @(z) findAmplificationRMSE(z, rdistfoot_y, randNoise_rdistfoot_y, desiredRMSE);
[amplification_rdistfoot_y, error_rdistfoot_y] = fminsearch(fun,1);
actualRMSE_rdistfoot_y = rms(rdistfoot_y - (rdistfoot_y + amplification_rdistfoot_y.*randNoise_rdistfoot_y));
rdistfoot_y_keypoint = rdistfoot_y + amplification_rdistfoot_y .* randNoise_rdistfoot_y;

% Repackage into variables with _cycle tag for easy switching between
% IMUVision and IMU version which runs on one gait cycle of data
cycle = linspace(0,100,endPeriodIndex);
t_cycle = t;
x_cycle = x;
xp_cycle = xp;
xpp_cycle = xpp;
y_cycle = y;
yp_cycle = yp;
ypp_cycle = ypp;
qt_cycle = qt;
qtp_cycle = qtp;
qtpp_cycle = qtpp;
qlh_cycle = qlh;
qlhp_cycle = qlhp;
qlhpp_cycle = qlhpp;
qrh_cycle = qrh;
qrhp_cycle = qrhp;
qrhpp_cycle = qrhpp;
qlk_cycle = qlk;
qlkp_cycle = qlkp;
qlkpp_cycle = qlkpp;
qrk_cycle = qrk;
qrkp_cycle = qrkp;
qrkpp_cycle = qrkpp;
qla_cycle = qla;
qlap_cycle = qlap;
qlapp_cycle = qlapp;
qra_cycle = qra;
qrap_cycle = qrap;
qrapp_cycle = qrapp;

grflx_cycle = grflx;
grfly_cycle = grfly;
grfrx_cycle = grfrx;
grfry_cycle = grfry;

pelvtx_cycle = pelvtx;
pelvty_cycle = pelvty;
pelvtilt_cycle = pelvtilt;
lhipmoment_cycle = lhipmoment;
rhipmoment_cycle = rhipmoment;
lkneemoment_cycle = lkneemoment;
rkneemoment_cycle = rkneemoment;
lanklemoment_cycle = lanklemoment;
ranklemoment_cycle = ranklemoment;

torso_ax_imu_cycle = torso_ax_imu;
torso_ay_imu_cycle = torso_ay_imu;
torso_wz_imu_cycle = torso_wz_imu;
lthigh_ax_imu_cycle = lthigh_ax_imu;
lthigh_ay_imu_cycle = lthigh_ay_imu;
lthigh_wz_imu_cycle = lthigh_wz_imu;
rthigh_ax_imu_cycle = rthigh_ax_imu;
rthigh_ay_imu_cycle = rthigh_ay_imu;
rthigh_wz_imu_cycle = rthigh_wz_imu;
lshank_ax_imu_cycle = lshank_ax_imu;
lshank_ay_imu_cycle = lshank_ay_imu;
lshank_wz_imu_cycle = lshank_wz_imu;
rshank_ax_imu_cycle = rshank_ax_imu;
rshank_ay_imu_cycle = rshank_ay_imu;
rshank_wz_imu_cycle = rshank_wz_imu;
lfoot_ax_imu_cycle = lfoot_ax_imu;
lfoot_ay_imu_cycle = lfoot_ay_imu;
lfoot_wz_imu_cycle = lfoot_wz_imu;
rfoot_ax_imu_cycle = rfoot_ax_imu;
rfoot_ay_imu_cycle = rfoot_ay_imu;
rfoot_wz_imu_cycle = rfoot_wz_imu;

proxtorso_x_cycle = proxtorso_x;
proxtorso_y_cycle = proxtorso_y;
hipjoint_x_cycle = hipjoint_x;
hipjoint_y_cycle = hipjoint_y;
lkneejoint_x_cycle = lkneejoint_x;
lkneejoint_y_cycle = lkneejoint_y;
rkneejoint_x_cycle = rkneejoint_x;
rkneejoint_y_cycle = rkneejoint_y;
lanklejoint_x_cycle = lanklejoint_x;
lanklejoint_y_cycle = lanklejoint_y;
ranklejoint_x_cycle = ranklejoint_x;
ranklejoint_y_cycle = ranklejoint_y;
ldistfoot_x_cycle = ldistfoot_x;
ldistfoot_y_cycle = ldistfoot_y;
rdistfoot_x_cycle = rdistfoot_x;
rdistfoot_y_cycle = rdistfoot_y;

proxtorso_x_keypoint_cycle = proxtorso_x_keypoint;
proxtorso_y_keypoint_cycle = proxtorso_y_keypoint;
hipjoint_x_keypoint_cycle = hipjoint_x_keypoint;
hipjoint_y_keypoint_cycle = hipjoint_y_keypoint;
lkneejoint_x_keypoint_cycle = lkneejoint_x_keypoint;
lkneejoint_y_keypoint_cycle = lkneejoint_y_keypoint;
rkneejoint_x_keypoint_cycle = rkneejoint_x_keypoint;
rkneejoint_y_keypoint_cycle = rkneejoint_y_keypoint;
lanklejoint_x_keypoint_cycle = lanklejoint_x_keypoint;
lanklejoint_y_keypoint_cycle = lanklejoint_y_keypoint;
ranklejoint_x_keypoint_cycle = ranklejoint_x_keypoint;
ranklejoint_y_keypoint_cycle = ranklejoint_y_keypoint;
ldistfoot_x_keypoint_cycle = ldistfoot_x_keypoint;
ldistfoot_y_keypoint_cycle = ldistfoot_y_keypoint;
rdistfoot_x_keypoint_cycle = rdistfoot_x_keypoint;
rdistfoot_y_keypoint_cycle = rdistfoot_y_keypoint;

lcontact_X_cycle = x + L_TH.*sin(qlh) + L_SH.*sin(qlh+qlk) + 0.5.*L_FT.*cos(qla+qlh+qlk);
lcontact_y_cycle = y - L_TH.*cos(qlh) - L_SH.*cos(qlh+qlk) + 0.5.*L_FT.*sin(qla+qlh+qlk);
rcontact_x_cycle = x + L_TH.*sin(qrh) + L_SH.*sin(qrh+qrk) + 0.5.*L_FT.*cos(qra+qrh+qrk);
rcontact_y_cycle = y - L_TH.*cos(qrh) - L_SH.*cos(qrh+qrk) + 0.5.*L_FT.*sin(qra+qrh+qrk);