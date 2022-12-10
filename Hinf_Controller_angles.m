load('Linear_Main.mat')        % Load the matrices of the system (rearranged states)
s = tf('s');

system = ss(A,B,C,0);
G = tf(system);

% Design phi,theta,psi controllers
quadModel1 = G(7,4);
quadModel2 = G(8,1);
quadModel3 = G(9,1);

%% Mixsyn Approach

% Default weights
W1 = 1; 
W2 = [];
W3 = 1;

% % Some realistic weights
W1 = makeweight(50,[1 0.5],0.01);
W2 = makeweight(0.1,[32 0.32],1); % Actuator bandwidth.
W3 = 1;%makeweight(0.01,[1 0.1],10); % No sensor noise or model uncertainty for now.

% Weight S - I find it weird that this transfer function is order 6
e_s = .001;
M_s = 1;
omega_b = .001;
k_s = 5;
W1 = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;

% Weight T - Having M_t as 6 seems off to me... usually its ~2 for zeta =
% 0.7.  
e_t = .01;
M_t = 2;
omega_bt = 100;
k_t = 2;
W3 = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;

figure; clf;
bodemag(1/W1,1/W2,1/W3);
legend('1/W1','1/W2','1/W3');


% Default weights
W1_3 = 1; 
W2_3 = [];
W3_3 = 1;

% % Some realistic weights
W1_3 = makeweight(50,[2 0.5],0.01);
W2_3 = makeweight(0.1,[32 0.32],1); % Actuator bandwidth.
W3_3 = 1;%makeweight(0.01,[1 0.1],10); % No sensor noise or model uncertainty for now.

% Weight S - I find it weird that this transfer function is order 6
e_s = .001;
M_s = 1;
omega_b = .1;
k_s = 6;
W1_3 = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;

% Weight T - Having M_t as 6 seems off to me... usually its ~2 for zeta =
% 0.7.  
e_t = .001;
M_t = 3;
omega_bt = 100;
k_t = 2;
W3_3 = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;

figure; clf;
bodemag(1/W1_3,1/W2_3,1/W3_3);
legend('1/W1','1/W2','1/W3');

%% Design controller 

[K1,CL1,gamma1] = mixsyn(quadModel1,W1,W2,W3);
disp(['Gamma1: ', num2str(gamma1)]);

[K2,CL2,gamma2] = mixsyn(quadModel2,W1,W2,W3);
disp(['Gamma2: ', num2str(gamma2)]);

[K3,CL3,gamma3] = mixsyn(quadModel3,W1_3,W2_3,W3_3);
disp(['Gamma3: ', num2str(gamma3)]);


%% S & T
S1 = feedback(1,quadModel1*K1);
T1 = 1-S1;

S2 = feedback(1,quadModel2*K2);
T2 = 1-S2;

S3 = feedback(1,quadModel3*K3);
T3 = 1-S3;

%% Plot
% Plot 1
t = 0:0.01:2;
figure; clf;
subplot(2,2,1:2); bodemag(S1,T1,K1*S1); legend('S','T','KS','Location','SouthEast'); grid on;

subplot(2,2,4); u = step(CL1(2),t); 
plot(t,u); title('Control u(t)'); xlabel('t (sec)'); grid on;

subplot(2,2,3); y = step(CL1(3),t);
hold on; plot(t,y); title('Step Response'); ylim([0,1.1]); xlabel('t (sec)'); grid on;


set(findall(gcf,'type','line'),'linewidth',1);

% Plot 2
figure; clf;
subplot(2,2,1:2); bodemag(S2,T2,K2*S2); legend('S','T','KS','Location','SouthEast'); grid on;

subplot(2,2,4); u = step(CL2(2),t); 
plot(t,u); title('Control u(t)'); xlabel('t (sec)'); grid on;

subplot(2,2,3); y = step(CL2(3),t);
hold on; plot(t,y); title('Step Response'); ylim([0,1.1]); xlabel('t (sec)'); grid on;


set(findall(gcf,'type','line'),'linewidth',1);

% Plot 3
figure; clf;
subplot(2,2,1:2); bodemag(S3,T3,K3*S3); legend('S','T','KS','Location','SouthEast'); grid on;

subplot(2,2,4); u = step(CL3(2),t); 
plot(t,u); title('Control u(t)'); xlabel('t (sec)'); grid on;

subplot(2,2,3); y = step(CL3(3),t);
hold on; plot(t,y); title('Step Response'); ylim([0,1.1]); xlabel('t (sec)'); grid on;


set(findall(gcf,'type','line'),'linewidth',1);

%% LETS TRY THIS
C_tf = tf(K1);
% Transfer Functions
R = feedback(series(C_tf,quadModel1),1);           % Transfer function of refence signal to y
D = feedback(quadModel1,C_tf,-1);                  % Transfer fucntion of disturbance to y
N = -feedback(series(C_tf,quadModel1),1);          % Transfer function of noise to y

% Set-Up Sims
t = 0:0.01:30;                     % Time of Sim
t2 = 0:0.001:100;                   % Time of Sim
t3 = 0:0.001:0.5;                   % Time of Sim
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

C_tf = tf(K2);
% Transfer Functions
R = feedback(series(C_tf,quadModel2),1);           % Transfer function of refence signal to y
D = feedback(quadModel2,C_tf,-1);                  % Transfer fucntion of disturbance to y
N = -feedback(series(C_tf,quadModel2),1);          % Transfer function of noise to y

% Set-Up Sims
t = 0:0.01:30;                     % Time of Sim
t2 = 0:0.001:100;                   % Time of Sim
t3 = 0:0.001:0.5;                   % Time of Sim
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

C_tf = tf(K3);
% Transfer Functions
R = feedback(series(C_tf,quadModel3),1);           % Transfer function of refence signal to y
D = feedback(quadModel3,C_tf,-1);                  % Transfer fucntion of disturbance to y
N = -feedback(series(C_tf,quadModel3),1);          % Transfer function of noise to y

% Set-Up Sims
t = 0:0.01:30;                     % Time of Sim
t2 = 0:0.001:100;                   % Time of Sim
t3 = 0:0.001:0.5;                   % Time of Sim
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