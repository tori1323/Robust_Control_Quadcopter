%% Loads the Properties of the Quad Model

%% Start Code Below:
% Physical Properties
m = 1.0;                    % Mass [kg]
g = 9.8;                    % Gravity [m/s^]
L = 0.155;                  % Length of Arm [m]
eta = 1;                    % Propeller Efficiency [nd]
I = [0.01151    0       0;
        0   0.01169     0;
        0       0   0.01275];

% Linear Model & Controller
load('Linear_Model.mat')

mph = [1:15]; 
Wind_Force_Nm2 = [0.18 0.76 1.73 3.07 4.80 6.89 9.39...
    12.24 15.52 19.13 23.18 27.58 32.34 37.54 43.06];
Area = 0.002669;            % m^2
Wind_Force_N = Wind_Force_Nm2.*Area;

% Sensor Noise Properties

% Sensor Sampling 
T = 0.01;                   % s