%% Solve for the Controller of the Quadcopter
% This script will LQR Method to solve for the input value 
% u = -Kx.
% This should only need to be ran once.

%% Start Code Below:
% Solve for the Equilibrium points
run('Properties_Quad.m')
clear A B C D K
syms theta psi phi U V W x y z p q r F1 F2 F3 F4
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

%% Create Controller LMIs
m = 4;
n = 9;
alpha = 1;

% LMI Solve
cvx_begin sdp quiet

% Variable definiton
variable S(n,n) symmetric
variable Z(m,n) 

% LMIs
A*S + S*A' - B*Z - Z'*B' + alpha*S <= -eps * eye(n)
S >= eps * eye(n)

cvx_end

K = Z*S^-1;                        % Solve for K matrix

save('Linear_Main.mat','A','B','C','D','K','u_e','x_e')
clear;clc;