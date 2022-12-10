%% Solve for the Controller of the Quadcopter
% This script will LQR Method to solve for the input value 
% u = -Kx.
% This should only need to be ran once.

%% Start Code Below:
% Solve for the Equilibrium points
run('Properties_Quad.m')
clear A B C D K
syms theta psi phi U V W x y z p q r F x_dot y_dot z_dot
x_vec = [x;y;z;x_dot;y_dot;z_dot];
u = [F;phi;theta;psi];

% Start Defining Equations
Thrust = F;
F_Thrust_E = -Thrust .* [cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
                         cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
                         cos(phi)*cos(theta)];
X_dot = (1/m).*([0;0;m*g] + F_Thrust_E);
x_dot = [x_dot;y_dot;z_dot];

% Combine the EqUations into Matrix
f = [x_dot;X_dot];
x_e = [0;0;0;0;0;0];                          % Equilibrium States

% Solve for u_e
gg = subs(f,x_vec,x_e);                 % Plug in x_e

f_e = gg == [0;0;0;0;0;0];        % Set the equation f(x_e,u_e) = 0

u_e = [9.8;0;0;0];

% u_temp = struct2cell(solve(f_e,u));     % Currently in a struct
% u_e = zeros(4,1);                       % Initialize variable
% for i = 1:4
%    u_e(i,1) = double(u_temp{i});
% end
% u_e                                     % Print out u_e for publishing

%% Solve for A & B
equil = [x_e;u_e]                         % Stitch x_e and u_e to linearize
a = jacobian(f,x_vec);                    % Create Jacobian matrix for A
b = jacobian(f,u);                        % Create Jacobian matrix for B
A_o = double(subs(a,[x_vec;u],equil))     % Plug in equilibrium points
B_o = double(subs(b,[x_vec;u],equil))     % Plug in equilibrium points
C_o = eye(6);                             % Define the C matrix for Sys 
D_o = 0;                                  % Define the D matrix for Sys


save('Linear_Outter.mat','A_o','B_o','C_o','D_o','u_e','x_e')
clear;clc;