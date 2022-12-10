syms theta psi phi U V W x y z p q r F1 F2 F3 F4

% Define Constants
m = 0.1;                    % [kg]
I_x = 0.00062;              % [kg-m^2]
I_y = I_x;              % [kg-m^2]
I_z = 0.9 * (I_x + I_y);    % [kg-m^2]
g = 9.8;                    % GraVity [m/s^]
dx = 0.114;                 % [m]
dy = 0.0825;                % [m]

x = [U;V;W;x;y;z;p;q;r;phi;theta;psi];
u = [F1;F2;F3;F4];

% Start Defining EqUations
L = (F2+F3)*dy - (F1+F4)*dy;
M = (F1+F3)*dx - (F2+F4)*dx;
N = F1*(dy-dx) + F2*(dx-dy) - F3*(dx+dy) + F4*(dx+dy);
U_dot = -g*sin(theta) + r*V - q*W;
V_dot = g*sin(phi)*cos(theta) + p*W - r*U;
W_dot = g*cos(theta)*cos(phi) - (1/m)*(F1+F2+F3+F4) + q*U - p*V;
u_b = U;
v_b = V;
w_b = W;
p_dot = (1/I_x)*(L + (I_y - I_z)*q*r);
q_dot = (1/I_y)*(M + (I_z - I_x)*r*p);
r_dot = (1/I_z)*(N + (I_x - I_y)*p*q);
phi_dot = p + (q*sin(phi) + r*cos(phi))*tan(theta);
theta_dot = q*cos(phi) - r*sin(phi);
psi_dot = (q*sin(phi) + r*cos(phi))*sec(theta);

% Combine the EqUations into Matrix
f = [U_dot;V_dot;W_dot;u_b;v_b;w_b;p_dot;q_dot;r_dot;phi_dot;theta_dot;psi_dot];
x_e = [0;0;0;0;0;0;0;0;0;0;0;0];          % Equilibrium States

% Solve for u_e
g = subs(f,x,x_e);                  % Plug in x_e

f_e = g == [0;0;0;0;0;0;0;0;0;0;0;0];     % Set the equation f(x_e,u_e) = 0

u_temp = struct2cell(solve(f_e,u)); % Currently in a struct
u_e = zeros(4,1);                   % Initialize variable
for i = 1:4
   u_e(i,1) = double(u_temp{i});
end
u_e                                 % Print out u_e for publishing

%% Solve for A & B
equil = [x_e;u_e]                   % Stitch x_e and u_e to linearize
a = jacobian(f,x);                  % Create Jacobian matrix for A
b = jacobian(f,u);                  % Create Jacobian matrix for B
A = double(subs(a,[x;u],equil))     % Plug in equilibrium points
B = double(subs(b,[x;u],equil))     % Plug in equilibrium points
% C = [1 0 0 0 0 0;                   % Define the C matrix for Sys
%      0 1 0 0 0 0;
%      0 0 1 0 0 0] 
% D = zeros(3,2)                      % Define the D matrix for Sys

%% Create Controller
m = 4;
n = 12;
alpha = 15;

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
span = [0 3];
x0 = [5;5;5;0;0;0;0;0;0;0;0;0];
%[t,y] = ode45(@(t,y) linear(y,A,B,K),span,x0);  % ODE45
[t,y] = ode45(@(t,y) nonlinear(y,K),span,x0);  % ODE45

figure
hold on
sgtitle('States of the System')
subplot(4,3,1)
plot(t,y(:,1))
xlabel('Time (sec)')
ylabel('U [m/s]')
grid
subplot(4,3,2)
plot(t,y(:,2))
xlabel('Time (sec)')
ylabel('V [m/s]')
grid
subplot(4,3,3)
plot(t,y(:,3))
xlabel('Time (sec)')
ylabel('W [m/s]')
grid
subplot(4,3,4)
plot(t,y(:,4))
xlabel('Time (sec)')
ylabel('x [rad/s]')
grid
subplot(4,3,5)
plot(t,y(:,5))
xlabel('Time (sec)')
ylabel('y [rad/s]')
grid
subplot(4,3,6)
plot(t,y(:,6))
xlabel('Time (sec)')
ylabel('z [rad/s]')
grid
subplot(4,3,7)
plot(t,y(:,4))
xlabel('Time (sec)')
ylabel('p [rad/s]')
grid
subplot(4,3,8)
plot(t,y(:,5))
xlabel('Time (sec)')
ylabel('q [rad/s]')
grid
subplot(4,3,9)
plot(t,y(:,6))
xlabel('Time (sec)')
ylabel('r [rad/s]')
grid
subplot(4,3,10)
plot(t,y(:,7))
xlabel('Time (sec)')
ylabel('phi [rads]')
grid
subplot(4,3,11)
plot(t,y(:,8))
xlabel('Time (sec)')
ylabel('theta [rads]')
grid
subplot(4,3,12)
plot(t,y(:,9))
xlabel('Time (sec)')
ylabel('psi [rads]')
grid
hold off

%% Test it out
span = [0 3];
x0 = [5;5;5;0;0;0;0;0;0;0;0;0];
[t,y] = ode45(@(t,y) linear(y,A,B,K),span,x0);  % ODE45
%[t,y] = ode45(@(t,y) nonlinear(y,K),span,x0);  % ODE45

figure
hold on
sgtitle('States of the System')
subplot(5,2,1)
plot(t,y(:,1))
xlabel('Time (sec)')
ylabel('U [m/s]')
grid
subplot(5,2,2)
plot(t,y(:,2))
xlabel('Time (sec)')
ylabel('V [m/s]')
grid
subplot(5,2,3)
plot(t,y(:,3))
xlabel('Time (sec)')
ylabel('W [m/s]')
grid
subplot(5,2,4)
plot(t,y(:,4))
xlabel('Time (sec)')
ylabel('p [rad/s]')
grid
subplot(5,2,5)
plot(t,y(:,5))
xlabel('Time (sec)')
ylabel('q [rad/s]')
grid
subplot(5,2,6)
plot(t,y(:,6))
xlabel('Time (sec)')
ylabel('r [rad/s]')
grid
subplot(5,2,7)
plot(t,y(:,7))
xlabel('Time (sec)')
ylabel('phi [rads]')
grid
subplot(5,2,8)
plot(t,y(:,8))
xlabel('Time (sec)')
ylabel('theta [rads]')
grid
subplot(5,2,9)
plot(t,y(:,9))
xlabel('Time (sec)')
ylabel('psi [rads]')
grid
hold off

%% Test it out V3
span = [0 3];
x0 = [5;5;5;0;0;0;0;0;0];
[t,y] = ode45(@(t,y) linear(y,A,B,K),span,x0);  % ODE45
%[t,y] = ode45(@(t,y) nonlinear(y,K),span,x0);  % ODE45

figure
hold on
sgtitle('States of the System')
subplot(3,3,1)
plot(t,y(:,1))
xlabel('Time (sec)')
ylabel('U [m/s]')
grid
subplot(3,3,2)
plot(t,y(:,2))
xlabel('Time (sec)')
ylabel('V [m/s]')
grid
subplot(3,3,3)
plot(t,y(:,3))
xlabel('Time (sec)')
ylabel('W [m/s]')
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


%% ============ Define Local Functions ===============
function[x_dot]= linear(x,A,B,K)
    u = -K*x;
    x_dot = A*x+B*u;
end

function[x_dot]= nonlinear(x,K)
        u = -K*x;
        U = x(1);
        V = x(2);
        W = x(3);
        p = x(7);
        q = x(8);
        r = x(9);
        phi = x(10);
        theta = x(11);
        psi = x(12);
        F1 = u(1);
        F2 = u(2);
        F3 = u(3);
        F4 = u(4);
    % Insert Equations
    x_dot = double([
                     V*r - (49*sin(theta))/5 - W*q
                                                                             (49*cos(theta)*sin(phi))/5 - U*r + W*p
                                             (49*cos(phi)*cos(theta))/5 - 10*F2 - 10*F3 - 10*F4 - 10*F1 + U*q - V*p
                                             U
                                             V
                                             W
          (4125*F2)/31 - (4125*F1)/31 + (4125*F3)/31 - (4125*F4)/31 - (6413125869375584375*q*r)/8935141660703064064
(11400*F1)/113 - (11400*F2)/113 + (11400*F3)/113 - (11400*F4)/113 + (27526000922488465625*p*r)/32570032505143427072
                                                         20*F2 - 20*F1 - (2620*F3)/21 + (2620*F4)/21 - (34*p*q)/105
                                                                           p + tan(theta)*(r*cos(phi) + q*sin(phi))
                                                                                            q*cos(phi) - r*sin(phi)
                                                                               (r*cos(phi) + q*sin(phi))/cos(theta)
                     ]);
end
