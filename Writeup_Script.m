%% This Script is for the Write-Up
load('Linear_Model.mat')

% Find the open-loop TF
%s = tf('s');
syms s
mid = (eye(9)*s - A)^-1
TF = eye(9)*s^3*mid*B

% Find the poles of system
E = eig(A)

% Test for Controllability
Cont = ctrb(A,B);
rank(Cont)

% Test for Observability
Obs = obsv(A,eye(9));
rank(Obs)

% SVD
[U1,S1,V1] = svd(A1)
k1 = cond(A1)
[U2,S2,V2] = svd(A2)
k2 = cond(A2)
[U3,S3,V3] = svd(A3)
k3 = cond(A3)
[U4,S4,V4] = svd(A4)
k4 = cond(A4)
[U,S,V] = svd(A)

% Frequncy Response
sys1 = ss(A1,B1,C1,zeros(3,4));
sys2 = ss(A2,B2,C2,zeros(3,4));
sys3 = ss(A3,B3,C3,zeros(2,4));
sys4 = ss(A4,B4,C4,zeros(1,4));
sys = ss(A,B,C,zeros(9,4));

figure(1)
bode(sys1)
figure(2)
bode(sys2)
figure(3)
bode(sys3)
figure(4)
bode(sys4)
% figure(5)
% bode(sys)

% 
tf1 = tf(sys1);
tf2 = tf(sys2);
tf3 = tf(sys3);
tf4 = tf(sys4);
tft = tf(sys);
plotoptions = sigmaoptions
plotoptions.XLim = [10^0 10^3]
sigmaplot(tft,plotoptions)

%% SAVE FOR LATER
% Weight S
e_s = .0001;
M_s = 4;
omega_b = 1;
k_s = 2;
W1_1 = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;
% e_s = .1;
% M_s = 2;
% omega_b = 1;
% k_s = 1;
W1_2 = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;
W_s = [W1_1   0;
        0   W1_2];

% Weight T
e_t = .01;
M_t = 2;
omega_bt = 1000;
k_t = 1;
W2_1 = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;
% e_t = .001;
% M_t = 2;
% omega_bt = 10;
% k_t = 2;
W2_2 = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;
W_t = [W2_1   0;
        0   W2_2];
