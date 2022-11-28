% Set Params for Nine DOF Musculoskeletal Model for Example OpenSim Subject
%-------------------------------+--------------------------+-------------------+-----------------
% Quantity                      | Value                    | Units             | Description
%-------------------------------|--------------------------|-------------------|-----------------
G                               =  9.81;                    % m/s2                Constant
L_T                             =  0.8781;                  % m                   Constant
L_TH                            =  0.4889;                  % m                   Constant
L_SH                            =  0.3728;                  % m                   Constant
L_FT                            =  0.250;                   % m                   Constant
M_T                             =  33.068 + 11.37517;       % kg                  Constant
M_TH                            =  8.98403823076288;        % kg                  Constant
M_SH                            =  3.58100089669871;        % kg                  Constant
M_FT   =  0.0965880214888392 + 1.20735026861049 + 0.209209654544826;  % kg        Constant
I_T                             =  1/12.*M_T.*L_T.^2;       % kg*m^2              Constant
I_TH                            =  1/12.*M_TH.*L_TH.^2;     % kg*m^2              Constant
I_SH                            =  1/12.*M_SH.*L_SH.^2;     % kg*m^2              Constant
I_FT                            =  1/12.*M_FT.*L_FT.^2;     % kg*m^2              Constant
LEN_HIP_TORSOIMU                =  0.25;                    % m                   Constant
LEN_HIP_THIGHIMU                =  0.25;                    % m                   Constant
LEN_KNEE_SHANKIMU               =  0.25;                    % m                   Constant
LEN_ANKLE_FOOTIMU               =  0.15;                    % m                   Constant

TOTAL_MASS = M_T + 2*M_TH + 2*M_SH + 2*M_FT;