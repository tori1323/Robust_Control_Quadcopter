% load('Linear_Model.mat')
load('Linear_Model.mat')

%% Subsystem 1
s = tf('s');
G_o = C*(s*eye(9)-A)^-1*B;
sys = ss(A,B,C,zeros(9,4));
G = tf(sys);

figure; sigma(G_o)
figure; sigma(G)
%%

% Weight 1 - x_dot
e_s = .01;
M_s = 2;
omega_b = 1;
k_s = 2;
W1_s = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;
% W1_s = 0;
e_t = .001;
M_t = 2;
omega_bt = 10;
k_t = 2;
W1_t = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;
% W1_t = 0;

% Weight 2
e_s = .1;
M_s = 2;
omega_b = .01;
k_s = 6;
W2_s = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;
W2_s = 0;
e_t = .01;
M_t = 6;
omega_bt = 90;
k_t = 2;
W2_t = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;
W2_t = 0;

% Weight 3
e_s = .01;
M_s = 2;
omega_b = 1;
k_s = 1;
W3_s = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;
W3_s = 0;
e_t = .001;
M_t = 2;
omega_bt = 100;
k_t = 2;
W3_t = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;
W3_t = 0;

% Weight 4 - y_dot
e_s = .001;
M_s = 2;
omega_b = .100;
k_s = 2;
W4_s = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;
% W4_s = 0;
e_t = .001;
M_t = 2;
omega_bt = 100;
k_t = 2;
W4_t = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;
% W4_t = 0;

% Weight 5
e_s = .1;
M_s = 2;
omega_b = .01;
k_s = 6;
W5_s = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;
W5_s = 0;
e_t = .01;
M_t = 6;
omega_bt = 90;
k_t = 2;
W5_t = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;
W5_t = 0;

% Weight 6
e_s = .1;
M_s = 2;
omega_b = .01;
k_s = 6;
W6_s = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;
W6_s = 0;
e_t = .01;
M_t = 6;
omega_bt = 90;
k_t = 2;
W6_t = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;
W6_t = 0;

% Weight 7
e_s = .1;
M_s = 2;
omega_b = 1;
k_s = 2;
W7_s = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;
% W7_s = 0;
e_t = .01;
M_t = 2;
omega_bt = 10;
k_t = 1;
W7_t = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;
% W7_t = 0;

% Weight 8
e_s = .001;
M_s = 2;
omega_b = 1;
k_s = 1;
W8_s = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;
% W8_s = 0;
e_t = .01;
M_t = 2;
omega_bt = 10;
k_t = 1;
W8_t = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;
% W8_t = 0;

% Weight 9 - z_dot
e_s = .001;
M_s = 2;
omega_b = 1;
k_s = 1;
W9_s = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;
% W9_s = 0;
e_t = .01;
M_t = 2;
omega_bt = 10;
k_t = 2;
W9_t = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;
% W9_t = 0;

W_s = [W1_s 0 0 0 0 0 0 0 0;
       0 W2_s 0 0 0 0 0 0 0;
       0 0 W3_s 0 0 0 0 0 0;
       0 0 0 W4_s 0 0 0 0 0;
       0 0 0 0 W5_s 0 0 0 0;
       0 0 0 0 0 W6_s 0 0 0;
       0 0 0 0 0 0 W7_s 0 0;
       0 0 0 0 0 0 0 W8_s 0;
       0 0 0 0 0 0 0 0 W9_s];

W_t = [W1_t 0 0 0 0 0 0 0 0;
       0 W2_t 0 0 0 0 0 0 0;
       0 0 W3_t 0 0 0 0 0 0;
       0 0 0 W4_t 0 0 0 0 0;
       0 0 0 0 W5_t 0 0 0 0;
       0 0 0 0 0 W6_t 0 0 0;
       0 0 0 0 0 0 W7_t 0 0;
       0 0 0 0 0 0 0 W8_t 0;
       0 0 0 0 0 0 0 0 W9_t];

P = augw(G,W_s,[],W_t);
P = minreal(P);
[K,~,gamma] = hinfsyn(P,9,4)
C_tf = tf(K);

figure
sigma(C_tf)

%%
R = feedback(series(C_tf,G),eye(9));
t = 0:0.01:30;                                 % Time of Sim
r = [ones(1,length(t)); 0.*t;0.*t;...
     ones(1,length(t)); 0.*t;0.*t;...
     0.*t;0.*t;ones(1,length(t))];             % Refernce Signal
[a] = lsim(R,r,t);

% % Transfer Functions
% R = feedback(G3*C_tf,eye(2));
% D = feedback(G3,C_tf,-1);
% N = -feedback(G3*C_tf,eye(2));
% % Set-Up Sims
% t = 0:0.01:30;                     % Time of Sim
% t2 = 0:0.01:100;                   % Time of Sim
% t3 = 0:0.01:0.5;                   % Time of Sim
% r = [ones(1,length(t)); ones(1,length(t))];           % Refernce Signal
% d = [sin(0.1.*t2)];                % Disturbance
% n = [sin(100.*t3)];                % Noise
% % Matlab Simulation Function
% [a] = lsim(R,r,t);
% [b] = lsim(D,d,t2);
% [c] = lsim(N,n,t3);

% Graphing Reference Signal
figure 
hold on
subplot(3,3,1)
title('Reference Tracking: 1')
hold on
plot(t,a(:,1))
plot(t,r(1,:))
line([0,30],[.99 .99],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('$\dot{x}$','Interpreter','latex')
hold off
subplot(3,3,2)
title('Reference Tracking: 2')
hold on
plot(t,a(:,2))
plot(t,r(2,:))
line([0,30],[.99 .99],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('q')
hold off
subplot(3,3,3)
title('Reference Tracking: 3')
hold on
plot(t,a(:,3))
plot(t,r(3,:))
line([0,30],[.99 .99],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('\theta')
hold off
subplot(3,3,4)
title('Reference Tracking: 4')
hold on
plot(t,a(:,4))
plot(t,r(4,:))
line([0,30],[.99 .99],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('$\dot{y}$','Interpreter','latex')
hold off
subplot(3,3,5)
title('Reference Tracking: 5')
hold on
plot(t,a(:,5))
plot(t,r(5,:))
line([0,30],[.99 .99],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('p')
hold off
subplot(3,3,6)
title('Reference Tracking: 6')
hold on
plot(t,a(:,6))
plot(t,r(6,:))
line([0,30],[.99 .99],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('\phi')
hold off
subplot(3,3,7)
title('Reference Tracking: 7')
hold on
plot(t,a(:,7))
plot(t,r(7,:))
line([0,30],[.99 .99],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('r')
hold off
subplot(3,3,8)
title('Reference Tracking: 8')
hold on
plot(t,a(:,8))
plot(t,r(8,:))
line([0,30],[.99 .99],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('\psi')
hold off
subplot(3,3,9)
title('Reference Tracking: 9')
hold on
plot(t,a(:,9))
plot(t,r(9,:))
line([0,30],[.99 .99],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('$\dot{z}$','Interpreter','latex')
legend('Sim','Signal','Requirements')
hold off
hold off
