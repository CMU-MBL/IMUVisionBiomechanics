% Set Params for 9 DOF Musculoskeletal Model for Sample Experimental Data
%-------------------------------+--------------------------+-------------------+
% Quantity                      | Value                    | Units             |
%-------------------------------|--------------------------|-------------------|
TOTAL_MASS                        =  73.8423;                % kg               
HEIGHT                            =  1.4134;                 % m                
G                                 =  9.81;                   % m/s2             
L_TH                              =  0.3752;                 % m                
L_SH                              =  0.1905;                 % m                
L_FT                              =  0.0425 * HEIGHT;        % m                
L_T                              =  0.6140;                  % m                
M_T                               =  (0.0668 + 0.4258 + 2*(0.0255 + 0.0138 + 0.0056))*TOTAL_MASS;
M_TH                              =  0.1478*TOTAL_MASS;      % kg                 
M_SH                              =  0.0481*TOTAL_MASS;      % kg       
M_FT                              =  0.0129*TOTAL_MASS;      % kg       
I_T                               =  1/12.*M_T.*L_T.^2;      % kg*m^2   
I_TH                              =  1/12.*M_TH.*L_TH.^2;    % kg*m^2   
I_SH                              =  1/12.*M_SH.*L_SH.^2;    % kg*m^2   
I_FT                              =  1/12.*M_FT.*L_FT.^2;    % kg*m^2   
LEN_HIP_TORSOIMU                  =  1/16*L_T;               % m        
LEN_HIP_THIGHIMU                  =  1/3*L_TH;               % m        
LEN_KNEE_SHANKIMU                 =  4/6*L_SH;               % m        
LEN_ANKLE_FOOTIMU                 =  0.5*L_FT;               % m        