syms theta psi phi U V W x y z p q r F1 F2 F3 F4

% Define Constants
m = 0.1;                    % [kg]
I_x = 0.00062;              % [kg-m^2]
I_y = I_x;              % [kg-m^2]
I_z = 0.9 * (I_x + I_y);    % [kg-m^2]
I = [I_x 0 0;0 I_y 0; 0 0 I_z];
g = 9.8;                    % GraVity [m/s^]
L = 0.114;                 % [m]
eta = 1;

x_vec = [x;y;z;p;q;r;phi;theta;psi];
u = [F1;F2;F3;F4];

% Start Defining EqUations
Thrust = F1 + F2 + F3 + F4;
F_Thrust_E = -Thrust .* [cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
                         cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
                         cos(phi)*cos(theta)];
X_dot = (1/m).*([0;0;m*g] + F_Thrust_E);
p_dot = (1/I_x)*(L*(F4-F2) - q*r*(I_z - I_y));
q_dot = (1/I_y)*(L*(F1-F3) - p*r*(I_x - I_z));
r_dot = (1/I_z)*eta*(F1 - F2 + F3 - F4);
phi_dot = p + (q*sin(phi) + r*cos(phi))*tan(theta);
theta_dot = q*cos(phi) - r*sin(phi);
psi_dot = (q*sin(phi) + r*cos(phi))*sec(theta);

% Combine the EqUations into Matrix
f = [X_dot;p_dot;q_dot;r_dot;phi_dot;theta_dot;psi_dot];
x_e = [0;0;0;0;0;0;0;0;0];          % Equilibrium States

% Solve for u_e
gg = subs(f,x_vec,x_e);                  % Plug in x_e

f_e = gg == [0;0;0;0;0;0;0;0;0];     % Set the equation f(x_e,u_e) = 0

u_temp = struct2cell(solve(f_e,u)); % Currently in a struct
u_e = zeros(4,1);                   % Initialize variable
for i = 1:4
   u_e(i,1) = double(u_temp{i});
end
u_e                                 % Print out u_e for publishing

%% Solve for A & B
equil = [x_e;u_e]                   % Stitch x_e and u_e to linearize
a = jacobian(f,x_vec);                  % Create Jacobian matrix for A
b = jacobian(f,u);                  % Create Jacobian matrix for B
A = double(subs(a,[x_vec;u],equil))     % Plug in equilibrium points
B = double(subs(b,[x_vec;u],equil))     % Plug in equilibrium points
% C = [1 0 0 0 0 0;                   % Define the C matrix for Sys
%      0 1 0 0 0 0;
%      0 0 1 0 0 0] 
% D = zeros(3,2)                      % Define the D matrix for Sys

%% Create Controller
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
%% Test it out V2
span = [0 10];
x0 = [.5;.5;.5;0;0;0;0;0;0];
[t,y] = ode45(@(t,y) nonlinear(y,K,u_e,x_e),span,x0);  % ODE45

figure
hold on
sgtitle('States of the System (Nonlinear)')
subplot(3,3,1)
plot(t,y(:,1))
xlabel('Time (sec)')
ylabel('x [m]')
grid
subplot(3,3,2)
plot(t,y(:,2))
xlabel('Time (sec)')
ylabel('y [m]')
grid
subplot(3,3,3)
plot(t,y(:,3))
xlabel('Time (sec)')
ylabel('z [m]')
grid
subplot(3,3,4)
plot(t,y(:,4))
xlabel('Time (sec)')
ylabel('p [rad/s]')
grid
subplot(3,3,5)
plot(t,y(:,5))
xlabel('Time (sec)')
ylabel('q [rad/s]')
grid
subplot(3,3,6)
plot(t,y(:,6))
xlabel('Time (sec)')
ylabel('r [rad/s]')
grid
subplot(3,3,7)
plot(t,y(:,7))
xlabel('Time (sec)')
ylabel('phi [rads]')
grid
subplot(3,3,8)
plot(t,y(:,8))
xlabel('Time (sec)')
ylabel('theta [rads]')
grid
subplot(3,3,9)
plot(t,y(:,9))
xlabel('Time (sec)')
ylabel('psi [rads]')
grid
hold off

%% Logsout Graph
sim('Quadcopter_Model',10)

logsout = ans.logsout;
figure
hold on
sgtitle('States of the System (Nonlinear)')
subplot(3,3,1)
plot(logsout{11}.Values.Time',logsout{11}.Values.Data)
xlabel('Time (sec)')
ylabel('x_{dot} [m/s]')
grid
subplot(3,3,2)
plot(logsout{12}.Values.Time',logsout{12}.Values.Data)
xlabel('Time (sec)')
ylabel('y_{dot} [m/s]')
grid
subplot(3,3,3)
plot(logsout{13}.Values.Time',logsout{13}.Values.Data)
xlabel('Time (sec)')
ylabel('z_{dot} [m/s]')
grid
subplot(3,3,4)
plot(logsout{5}.Values.Time',logsout{5}.Values.Data)
xlabel('Time (sec)')
ylabel('p [rad/s]')
grid
subplot(3,3,5)
plot(logsout{8}.Values.Time',logsout{8}.Values.Data)
xlabel('Time (sec)')
ylabel('q [rad/s]')
grid
subplot(3,3,6)
plot(logsout{9}.Values.Time',logsout{9}.Values.Data)
xlabel('Time (sec)')
ylabel('r [rad/s]')
grid
subplot(3,3,7)
plot(logsout{6}.Values.Time',logsout{6}.Values.Data)
xlabel('Time (sec)')
ylabel('phi [rad/s]')
grid
subplot(3,3,8)
plot(logsout{10}.Values.Time',logsout{10}.Values.Data)
xlabel('Time (sec)')
ylabel('theta [rad/s]')
grid
subplot(3,3,9)
plot(logsout{7}.Values.Time',logsout{7}.Values.Data)
xlabel('Time (sec)')
ylabel(' [rad/s]')
grid
hold off

figure
hold on
sgtitle('Inputs of the System')
subplot(2,2,1)
plot(logsout{1}.Values.Time',logsout{1}.Values.Data)
xlabel('Time (sec)')
ylabel('U [m/s]')
grid
subplot(2,2,2)
plot(logsout{2}.Values.Time',logsout{2}.Values.Data)
xlabel('Time (sec)')
ylabel('V [m/s]')
grid
subplot(2,2,3)
plot(logsout{3}.Values.Time',logsout{3}.Values.Data)
xlabel('Time (sec)')
ylabel('W [m/s]')
grid
subplot(2,2,4)
plot(logsout{4}.Values.Time',logsout{4}.Values.Data)
xlabel('Time (sec)')
ylabel('x [rad/s]')
grid
hold off

%% ============ Define Local Functions ===============
function[x_dot]= linear(x,A,B,K)
    u = -K*x;
    x_dot = A*x+B*u;
end

function[d_dot]= nonlinear(d,K,u_e,x_e)
        u = -K*(d-x_e) + u_e;
        p = d(4);
        q = d(5);
        r = d(6);
        phi = d(7);
        theta = d(8);
        psi = d(9);
        F1 = u(1);
        F2 = u(2);
        F3 = u(3);
        F4 = u(4);
    % Insert Equations
    d_dot = double([
                     
 
                                                                           -10*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(F1 + F2 + F3 + F4)
                                                                            10*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*(F1 + F2 + F3 + F4)
                                                                                                    49/5 - 10*cos(phi)*cos(theta)*(F1 + F2 + F3 + F4)
                                                                                                              (5700*F4)/31 - (5700*F2)/31 - (4*q*r)/5
                                                                                                              (5700*F1)/31 - (5700*F3)/31 + (4*p*r)/5
(3940901891670251*F1)/4398046511104 - (3940901891670251*F2)/4398046511104 + (3940901891670251*F3)/4398046511104 - (3940901891670251*F4)/4398046511104
                                                                                                             p + tan(theta)*(r*cos(phi) + q*sin(phi))
                                                                                                                              q*cos(phi) - r*sin(phi)
                                                                                                                 (r*cos(phi) + q*sin(phi))/cos(theta)
 
                     ]);
end
