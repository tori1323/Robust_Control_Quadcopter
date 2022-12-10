load('Linear_Model.mat')        % Load the matrices of the system (rearranged states)

%% Subsystem 1
s = tf('s');
B1 = [0;13.26;0];               % Collapse the B matrix
C1 = [1 0 0];                   % We can only control one state- measure that one
G1 = C1*(s*eye(3)-A1)^-1*B1;    % Solve for transfer function of plant

% Weight S - I find it weird that this transfer function is order 6
e_s = .1;
M_s = 2;
omega_b = .01;
k_s = 6;
W1_s = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;

% Weight T - Having M_t as 6 seems off to me... usually its ~2 for zeta =
% 0.7.  
e_t = .01;
M_t = 6;
omega_bt = 90;
k_t = 2;
W1_t = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;

% Solve for the controller and its transfer function
[K1,~,gamma1] = mixsyn(G1,W1_s,[],W1_t)
C_tf = tf(K1);  
figure
sigma(C_tf)                        % Plot the sigma graph of the controller
figure
sigma(1/W1_s,1/W1_t)               % Plot the sigma graphs of the weights

% Transfer Functions
R = feedback(G1*C_tf,1);           % Transfer function of refence signal to y
D = feedback(G1,C_tf,-1);          % Transfer fucntion of disturbance to y
N = -feedback(G1*C_tf,1);          % Transfer function of noise to y

% Set-Up Sims
t = 0:0.01:30;                     % Time of Sim
t2 = 0:0.01:100;                   % Time of Sim
t3 = 0:0.01:0.5;                   % Time of Sim
r = [ones(1,length(t))];           % Refernce Signal
d = [sin(0.1.*t2)];                % Disturbance
n = [sin(100.*t3)];                % Noise

% Matlab Simulation Function
[a] = lsim(R,r,t);
[b] = lsim(D,d,t2);
[c] = lsim(N,n,t3);

% Plot responses to reference, disturbance, and noise
figure 
hold on
subplot(3,1,1)
title('Reference Tracking')
hold on
plot(t,a(:,1))
plot(t,r)
line([0,30],[.99 .99],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('R Signal')
hold off
subplot(3,1,2)
title('Distrubance Rejection')
hold on
plot(t2,b(:,1))
plot(t2,d)
line([0,100],[.1 .1],'Color','green','LineStyle','--')
line([0,100],[-.1 -.1],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('D Signal')
hold off
subplot(3,1,3)
title('Noise Rejection')
hold on
plot(t3,c(:,1))
plot(t3,n)
line([0,0.5],[.01 .01],'Color','green','LineStyle','--')
line([0,0.5],[-.01 -.01],'Color','green','LineStyle','--')
hold off
xlabel('Time[s]')
ylabel('N Signal')
legend('Sim','Signal','Requirements')
hold off

%% Subsystem 2
% The response in this system should be basically the same thing as
% subsystem 1.  The difference would show up when breaking apart the u
% signal for the whole system input.

B2 = [0;13.26;0];                   % Collapse the B matrix into one input
C2 = [1 0 0];                       % We can only control one state- measure that one
G2 = C2*(s*eye(3)-A2)^-1*B2;        % Solve for transfer function of plant

% Weight S
e_s = .1;
M_s = 2;
omega_b = .01;
k_s = 6;
W2_s = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;

% Weight T
e_t = .01;
M_t = 6;
omega_bt = 90;
k_t = 2;
W2_t = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;

% Solve for the controller and its transfer function
[K2,~,gamma2] = mixsyn(G2,W2_s,[],W2_t);
C_tf = tf(K2);

% Transfer Functions
R = feedback(G2*C_tf,1);           % Transfer function of refence signal to y
D = feedback(G2,C_tf,-1);          % Transfer fucntion of disturbance to y
N = -feedback(G2*C_tf,1);          % Transfer function of noise to y

% Set-Up Sims
t = 0:0.01:30;                     % Time of Sim
t2 = 0:0.01:100;                   % Time of Sim
t3 = 0:0.01:0.5;                   % Time of Sim
r = [ones(1,length(t))];           % Refernce Signal
d = [sin(0.1.*t2)];                % Disturbance
n = [sin(100.*t3)];                % Noise

% Matlab Simulation Function
[a] = lsim(R,r,t);
[b] = lsim(D,d,t2);
[c] = lsim(N,n,t3);

% Plot responses to reference, disturbance, and noise
figure 
hold on
subplot(3,1,1)
title('Reference Tracking')
hold on
plot(t,a(:,1))
plot(t,r)
line([0,30],[.99 .99],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('R Signal')
hold off
subplot(3,1,2)
title('Distrubance Rejection')
hold on
plot(t2,b(:,1))
plot(t2,d)
line([0,100],[.1 .1],'Color','green','LineStyle','--')
line([0,100],[-.1 -.1],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('D Signal')
hold off
subplot(3,1,3)
title('Noise Rejection')
hold on
plot(t3,c(:,1))
plot(t3,n)
line([0,0.5],[.01 .01],'Color','green','LineStyle','--')
line([0,0.5],[-.01 -.01],'Color','green','LineStyle','--')
hold off
xlabel('Time[s]')
ylabel('N Signal')
legend('Sim','Signal','Requirements')
hold off

%% Subsystem 3
% I did not collapse anything since I could use hinfsyn() with this system
% just fine. (I tried collasping the the B matrix and it did not work out 
% well). The weights are the same across the diagonal, but left as is
% so they could be different.  I put less time with this one just because I
% only needed to stabilize these states.  The gamma value here is
% concerning, but the response of the system looks good?  The problem I
% quickly run into is actually getting performance from the system though
% without these weights.  It seemed to like having matching weights when I
% was designing the controller earlier.  The controller looks like it has
% high gains- which historically causes problems.  
G3 = C3*(s*eye(2)-A3)^-1*B3;

% Weight S
e_s = .0001;
M_s = 4;
omega_b = 1;
k_s = 2;
W1_1 = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;
% e_s = .0001;
% M_s = 4;
% omega_b = 1;
% k_s = 2;
W1_2 = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;
W_s = [W1_1   0;
        0   W1_2];

% Weight T
e_t = .01;
M_t = 2;
omega_bt = 1000;
k_t = 1;
W2_1 = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;
% e_t = .01;
% M_t = 2;
% omega_bt = 1000;
% k_t = 1;
W2_2 = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;
W_t = [W2_1   0;
        0   W2_2];

% Solve for the controller and its transfer function
P = augw(G3,W_s,[],W_t);
P = minreal(P);
[K3,~,gamma3] = hinfsyn(P,2,4)
C_tf = tf(K3);
figure
sigma(C_tf)

R = feedback(series(C_tf,G3),eye(2));                   % Transfer function of refence signal to y
t = 0:0.01:30;                                          % Time of Sim
r = [zeros(1,length(t)); ones(1,length(t))];            % Refernce Signal
[a] = lsim(R,r,t);

% Graph only the y values for reference tracking
figure 
hold on
subplot(2,1,1)
title('Reference Tracking: 1')
hold on
plot(t,a(:,1))
plot(t,r(1,:))
line([0,30],[.01 .01],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('R Signal: 1')
hold off
subplot(2,1,2)
title('Reference Tracking: 2')
hold on
plot(t,a(:,2))
plot(t,r(2,:))
line([0,30],[.99 .99],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('R Signal: 2')
legend('Sim','Signal','Requirements')
hold off

%% Subsystem 4
% This system has weights that are closer to "normal".  The controller also
% has the weird dip at omega = 1 rad/sec.
B4 = -4;                            % Collapse the B matrix
G4 = C4*(s*eye(1)-A4)^-1*B4;        % Transfer function of plant

% Weight S
e_s = .001;
M_s = 2;
omega_b = 1;
k_s = 3;
W4_s = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;

% Weight T
e_t = .001;
M_t = 2;
omega_bt = 10;
k_t = 2;
W4_t = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;

% Solve for the controller and its transfer function
[K4,~,gamma4] = mixsyn(G4,W4_s,[],W4_t);
C_tf4 = tf(K4);
figure
sigma(C_tf)                         % Plot the sigma graph of the controller

% Transfer Functions
R = feedback(G4*C_tf4,1);          % Transfer function of refence signal to y
D = feedback(G4,C_tf4,-1);         % Transfer fucntion of disturbance to y
N = -feedback(G4*C_tf4,1);         % Transfer function of noise to y

% Set-Up Sims
t = 0:0.01:30;                     % Time of Sim
t2 = 0:0.01:100;                   % Time of Sim
t3 = 0:0.01:0.5;                   % Time of Sim
r = [ones(1,length(t))];           % Refernce Signal
d = [sin(0.1.*t2)];                % Disturbance
n = [sin(100.*t3)];                % Noise

% Matlab Simulation Function
[a] = lsim(R,r,t);
[b] = lsim(D,d,t2);
[c] = lsim(N,n,t3);

% Plot responses to reference, disturbance, and noise
figure 
hold on
subplot(3,1,1)
title('Reference Tracking')
hold on
plot(t,a(:,1))
plot(t,r)
line([0,30],[.99 .99],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('R Signal')
hold off
subplot(3,1,2)
title('Distrubance Rejection')
hold on
plot(t2,b(:,1))
plot(t2,d)
line([0,100],[.1 .1],'Color','green','LineStyle','--')
line([0,100],[-.1 -.1],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('D Signal')
hold off
subplot(3,1,3)
title('Noise Rejection')
hold on
plot(t3,c(:,1))
plot(t3,n)
line([0,0.5],[.01 .01],'Color','green','LineStyle','--')
line([0,0.5],[-.01 -.01],'Color','green','LineStyle','--')
hold off
xlabel('Time[s]')
ylabel('N Signal')
legend('Sim','Signal','Requirements')
hold off


%% Subsystem 1... Let's try something different...
s = tf('s');
B1 = [0;13.26;0];               % Collapse the B matrix
C1 = [1 0 0];                   % We can only control one state- measure that one
G1_o = C1*(s*eye(3)-A1)^-1*B1;    % Solve for transfer function of plant
sys1 = ss(A1,B1,C1,zeros(1,1));
G1 = tf(sys1);

% Weight S - I find it weird that this transfer function is order 6
e_s = .01;
M_s = 2;
omega_b = .1;
k_s = 7;
W1_s = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;

% Weight T - Having M_t as 6 seems off to me... usually its ~2 for zeta =
% 0.7.  
e_t = .001;
M_t = 2;
omega_bt = 100;
k_t = 3;
W1_t = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;

% Solve for the controller and its transfer function
[K1,~,gamma1] = mixsyn(sys1,W1_s,[],W1_t)
C_tf = tf(K1);  
figure
sigma(C_tf)                        % Plot the sigma graph of the controller
figure
sigma(1/W1_s,1/W1_t)               % Plot the sigma graphs of the weights

% Transfer Functions
R = feedback(G1*C_tf,1);           % Transfer function of refence signal to y
D = feedback(G1,C_tf,-1);          % Transfer fucntion of disturbance to y
N = -feedback(G1*C_tf,1);          % Transfer function of noise to y

% Set-Up Sims
t = 0:0.01:30;                     % Time of Sim
t2 = 0:0.01:100;                   % Time of Sim
t3 = 0:0.01:0.5;                   % Time of Sim
r = [ones(1,length(t))];           % Refernce Signal
d = [sin(0.1.*t2)];                % Disturbance
n = [sin(100.*t3)];                % Noise

% Matlab Simulation Function
[a] = lsim(R,r,t);
[b] = lsim(D,d,t2);
[c] = lsim(N,n,t3);

% Plot responses to reference, disturbance, and noise
figure 
hold on
subplot(3,1,1)
title('Reference Tracking')
hold on
plot(t,a(:,1))
plot(t,r)
line([0,30],[.99 .99],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('R Signal')
hold off
subplot(3,1,2)
title('Distrubance Rejection')
hold on
plot(t2,b(:,1))
plot(t2,d)
line([0,100],[.1 .1],'Color','green','LineStyle','--')
line([0,100],[-.1 -.1],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('D Signal')
hold off
subplot(3,1,3)
title('Noise Rejection')
hold on
plot(t3,c(:,1))
plot(t3,n)
line([0,0.5],[.01 .01],'Color','green','LineStyle','--')
line([0,0.5],[-.01 -.01],'Color','green','LineStyle','--')
hold off
xlabel('Time[s]')
ylabel('N Signal')
legend('Sim','Signal','Requirements')
hold off

%% Subsystem 2
% The response in this system should be basically the same thing as
% subsystem 1.  The difference would show up when breaking apart the u
% signal for the whole system input.

B2 = [0;13.47;0];                   % Collapse the B matrix into one input
C2 = [1 0 0];                       % We can only control one state- measure that one
G2 = C2*(s*eye(3)-A2)^-1*B2;        % Solve for transfer function of plant
sys2 = ss(A2,B2,C2,zeros(1,1));
G2 = tf(sys2);

% Weight S
e_s = .01;
M_s = 2;
omega_b = .1;
k_s = 7;
W2_s = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;

% Weight T
e_t = .001;
M_t = 2;
omega_bt = 100;
k_t = 3;
W2_t = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;

% Solve for the controller and its transfer function
[K2,~,gamma2] = mixsyn(sys2,W2_s,[],W2_t);
C_tf = tf(K2);

% Transfer Functions
R = feedback(G2*C_tf,1);           % Transfer function of refence signal to y
D = feedback(G2,C_tf,-1);          % Transfer fucntion of disturbance to y
N = -feedback(G2*C_tf,1);          % Transfer function of noise to y

% Set-Up Sims
t = 0:0.01:30;                     % Time of Sim
t2 = 0:0.01:100;                   % Time of Sim
t3 = 0:0.01:0.5;                   % Time of Sim
r = [ones(1,length(t))];           % Refernce Signal
d = [sin(0.1.*t2)];                % Disturbance
n = [sin(100.*t3)];                % Noise

% Matlab Simulation Function
[a] = lsim(R,r,t);
[b] = lsim(D,d,t2);
[c] = lsim(N,n,t3);

% Plot responses to reference, disturbance, and noise
figure 
hold on
subplot(3,1,1)
title('Reference Tracking')
hold on
plot(t,a(:,1))
plot(t,r)
line([0,30],[.99 .99],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('R Signal')
hold off
subplot(3,1,2)
title('Distrubance Rejection')
hold on
plot(t2,b(:,1))
plot(t2,d)
line([0,100],[.1 .1],'Color','green','LineStyle','--')
line([0,100],[-.1 -.1],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('D Signal')
hold off
subplot(3,1,3)
title('Noise Rejection')
hold on
plot(t3,c(:,1))
plot(t3,n)
line([0,0.5],[.01 .01],'Color','green','LineStyle','--')
line([0,0.5],[-.01 -.01],'Color','green','LineStyle','--')
hold off
xlabel('Time[s]')
ylabel('N Signal')
legend('Sim','Signal','Requirements')
hold off

%% Subsystem 3
G3 = C3*(s*eye(2)-A3)^-1*B3;
sys3 = ss(A3,B3,C3,zeros(2,4));
G3 = tf(sys3);

% Weight S
e_s = .1;
M_s = 2;
omega_b = 100;
k_s = 2;
W1_1 = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;
e_s = .001;
M_s = 2;
omega_b = 1;
k_s = 1;
W1_2 = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;
W_s = [W1_1   0;
        0   W1_2];

% Weight T
e_t = .01;
M_t = 2;
omega_bt = 100;
k_t = 1;
W2_1 = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;
% e_t = .01;
% M_t = 2;
% omega_bt = 1000;
% k_t = 1;
W2_2 = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;
W_t = [W2_1   0;
        0   W2_2];

% Solve for the controller and its transfer function
P = augw(sys3,W_s,[],W_t);
P = minreal(P);
[K3,~,gamma3] = hinfsyn(P,2,4)
C_tf = tf(K3);
figure
sigma(C_tf)

R = feedback(series(C_tf,G3),eye(2));                   % Transfer function of refence signal to y
D = feedback(G3,C_tf,-1);                               % Transfer fucntion of disturbance to y
N = -feedback(G3*C_tf,eye(2));                          % Transfer function of noise to y

t = 0:0.01:30;                                          % Time of Sim
t2 = 0:0.01:100;                                        % Time of Sim
t3 = 0:0.01:0.5;                                        % Time of Sim

r = [zeros(1,length(t)); ones(1,length(t))];            % Refernce Signal
d = [sin(0.1.*t2);sin(0.1.*t2);0.*t2;0.*t2];                                     % Disturbance
n = [sin(100.*t3);sin(100.*t3)];                                     % Noise

[a] = lsim(R,r,t);
[b] = lsim(D,d,t2);
[c] = lsim(N,n,t3);

% Graph only the y values for reference tracking
figure 
hold on
subplot(2,1,1)
title('Reference Tracking: 1')
hold on
plot(t,a(:,1))
plot(t,r(1,:))
line([0,30],[.01 .01],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('R Signal: 1')
hold off
subplot(2,1,2)
title('Reference Tracking: 2')
hold on
plot(t,a(:,2))
plot(t,r(2,:))
line([0,30],[.99 .99],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('R Signal: 2')
legend('Sim','Signal','Requirements')
hold off

figure 
hold on
subplot(2,1,1)
title('Reference Tracking: 1')
hold on
plot(t2,b(:,1))
plot(t2,d(1,:))
line([0,100],[.01 .01],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('R Signal: 1')
hold off
subplot(2,1,2)
title('Reference Tracking: 2')
hold on
plot(t2,b(:,2))
plot(t2,d(2,:))
line([0,100],[.99 .99],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('R Signal: 2')
legend('Sim','Signal','Requirements')
hold off

figure 
hold on
subplot(2,1,1)
title('Reference Tracking: 1')
hold on
plot(t3,c(:,1))
plot(t3,n(1,:))
line([0,0.5],[.01 .01],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('R Signal: 1')
hold off
subplot(2,1,2)
title('Reference Tracking: 2')
hold on
plot(t3,c(:,2))
plot(t3,n(2,:))
line([0,0.5],[.01 .01],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('R Signal: 2')
legend('Sim','Signal','Requirements')
hold off


%% Subsystem 4
% This system has weights that are closer to "normal".  The controller also
% has the weird dip at omega = 1 rad/sec.
B4 = -4;                            % Collapse the B matrix
G4 = C4*(s*eye(1)-A4)^-1*B4;        % Transfer function of plant
% sys4 = ss(A4,B4,C4,0);
% G4 = tf(sys4);

% Weight S
e_s = .001;
M_s = 2;
omega_b = 1;
k_s = 3;
W4_s = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;

% Weight T
e_t = .001;
M_t = 2;
omega_bt = 10;
k_t = 2;
W4_t = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;

% Solve for the controller and its transfer function
[K4,~,gamma4] = mixsyn(G4,W4_s,[],W4_t)
C_tf4 = tf(K4);
figure
sigma(C_tf4)                         % Plot the sigma graph of the controller

% Transfer Functions
R = feedback(G4*C_tf4,1);          % Transfer function of refence signal to y
D = feedback(G4,C_tf4,-1);         % Transfer fucntion of disturbance to y
N = -feedback(G4*C_tf4,1);         % Transfer function of noise to y

% Set-Up Sims
t = 0:0.01:30;                     % Time of Sim
t2 = 0:0.01:100;                   % Time of Sim
t3 = 0:0.01:0.5;                   % Time of Sim
r = [ones(1,length(t))];           % Refernce Signal
d = [sin(0.1.*t2)];                % Disturbance
n = [sin(100.*t3)];                % Noise

% Matlab Simulation Function
[a] = lsim(R,r,t);
[b] = lsim(D,d,t2);
[c] = lsim(N,n,t3);

% Plot responses to reference, disturbance, and noise
figure 
hold on
subplot(3,1,1)
title('Reference Tracking')
hold on
plot(t,a(:,1))
plot(t,r)
line([0,30],[.99 .99],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('R Signal')
hold off
subplot(3,1,2)
title('Distrubance Rejection')
hold on
plot(t2,b(:,1))
plot(t2,d)
line([0,100],[.1 .1],'Color','green','LineStyle','--')
line([0,100],[-.1 -.1],'Color','green','LineStyle','--')
xlabel('Time[s]')
ylabel('D Signal')
hold off
subplot(3,1,3)
title('Noise Rejection')
hold on
plot(t3,c(:,1))
plot(t3,n)
line([0,0.5],[.01 .01],'Color','green','LineStyle','--')
line([0,0.5],[-.01 -.01],'Color','green','LineStyle','--')
hold off
xlabel('Time[s]')
ylabel('N Signal')
legend('Sim','Signal','Requirements')
hold off