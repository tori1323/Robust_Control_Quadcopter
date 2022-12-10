%% Solve for the Controller of the Quadcopter
% This script will LQR Method to solve for the input value 
% u = -Kx.
% This should only need to be ran once.

%% Start Code Below:
% Solve for the Equilibrium points
run('Properties_Quad.m')
clear A B C D K
syms theta psi phi U V W x y z p q r F1 F2 F3 F4
x_vec = [x;q;theta;y;p;phi;r;psi;z];
u = [F1;F2;F3;F4];

% Start Defining Equations
Thrust = F1 + F2 + F3 + F4;
F_Thrust_E = -Thrust .* [cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
                         cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
                         cos(phi)*cos(theta)];
X_dot = (1/m).*([0;0;m*g] + F_Thrust_E);
M = [L*(F4-F2);L*(F1-F3);eta*(F1-F2+F3-F4)];
omega = [p;q;r];
omega_dot = I^-1 * (M - cross(omega,(I*omega)));
phi_dot = p + (q*sin(phi) + r*cos(phi))*tan(theta);
theta_dot = q*cos(phi) - r*sin(phi);
psi_dot = (q*sin(phi) + r*cos(phi))*sec(theta);

% Combine the EqUations into Matrix
f_T = [X_dot;omega_dot;phi_dot;theta_dot;psi_dot];
f = [f_T(1,:);f_T(5,:);f_T(8,:);f_T(2,:);f_T(4,:);f_T(7,:);...
    f_T(6,:);f_T(9,:);f_T(3,:)]
x_e = [0;0;0;0;0;0;0;0;0];              % Equilibrium States

% Solve for u_e
gg = subs(f,x_vec,x_e);                 % Plug in x_e

f_e = gg == [0;0;0;0;0;0;0;0;0];        % Set the equation f(x_e,u_e) = 0

u_temp = struct2cell(solve(f_e,u));     % Currently in a struct
u_e = zeros(4,1);                       % Initialize variable
for i = 1:4
   u_e(i,1) = double(u_temp{i});
end
u_e                                     % Print out u_e for publishing

%% Solve for A & B
equil = [x_e;u_e]                       % Stitch x_e and u_e to linearize
a = jacobian(f,x_vec);                  % Create Jacobian matrix for A
b = jacobian(f,u);                      % Create Jacobian matrix for B
A = double(subs(a,[x_vec;u],equil))     % Plug in equilibrium points
B = double(subs(b,[x_vec;u],equil))     % Plug in equilibrium points
C = eye(9);                             % Define the C matrix for Sys 
D = zeros(9,4)                          % Define the D matrix for Sys

%% Create Controller
% Break Apart the Matricies
A1 = A(1:3,1:3)
B1 = B(1:3,:)
C1 = eye(3)
D1 = zeros(3,4);

A2 = A(4:6,4:6)
B2 = B(4:6,:);
C2 = eye(3);
D2 = zeros(3,4);

A3 = A(7:8,7:8)
B3 = B(7:8,:)
C3 = eye(2);
D3 = zeros(2,4)

A4 = A(9,9)
B4 = B(9,:)
C4 = 1;
D4 = zeros(1,0);

% Now keep the proper order:: 
x_vec = [x;y;z;p;q;r;phi;theta;psi];
u = [F1;F2;F3;F4];

% Start Defining Equations
Thrust = F1 + F2 + F3 + F4;
F_Thrust_E = -Thrust .* [cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
                         cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
                         cos(phi)*cos(theta)];
X_dot = (1/m).*([0;0;m*g] + F_Thrust_E);
M = [L*(F4-F2);L*(F1-F3);eta*(F1-F2+F3-F4)];
omega = [p;q;r];
omega_dot = I^-1 * (M - cross(omega,(I*omega)));
phi_dot = p + (q*sin(phi) + r*cos(phi))*tan(theta);
theta_dot = q*cos(phi) - r*sin(phi);
psi_dot = (q*sin(phi) + r*cos(phi))*sec(theta);

% Combine the EqUations into Matrix
f = [X_dot;omega_dot;phi_dot;theta_dot;psi_dot];
x_e = [0;0;0;0;0;0;0;0;0];              % Equilibrium States

% Solve for u_e
gg = subs(f,x_vec,x_e);                 % Plug in x_e

f_e = gg == [0;0;0;0;0;0;0;0;0];        % Set the equation f(x_e,u_e) = 0

u_temp = struct2cell(solve(f_e,u));     % Currently in a struct
u_e = zeros(4,1);                       % Initialize variable
for i = 1:4
   u_e(i,1) = double(u_temp{i});
end
u_e                                     % Print out u_e for publishing

%% Solve for A & B
equil = [x_e;u_e]                       % Stitch x_e and u_e to linearize
a = jacobian(f,x_vec);                  % Create Jacobian matrix for A
b = jacobian(f,u);                      % Create Jacobian matrix for B
A = double(subs(a,[x_vec;u],equil))     % Plug in equilibrium points
B = double(subs(b,[x_vec;u],equil))     % Plug in equilibrium points
C = eye(9);                             % Define the C matrix for Sys 
D = zeros(9,4)                          % Define the D matrix for Sys

save('Linear_Model.mat','A','B','A1','B1','C1','D1','A2','B2','C2','D2',...
    'A3','B3','C3','D3','A4','B4','C4','D4','u_e','x_e')
clear;clc;