load('Linear_Outter.mat')
system_o = ss(A_o,B_o,C_o,[])
G_o = tf(system_o)

% Design phi,theta,psi controllers
quadModel1 = G_o(1,3);
quadModel2 = G_o(2,2);
quadModel3 = G_o(3,1);

%% Mixsyn Approach

% Default weights
W1_1 = 1; 
W2_1 = [];
W3_1 = 1;

% % Some realistic weights
W1_1 = makeweight(50,[.3 0.1],0.01);
W2_1 = makeweight(0.1,[32 0.32],1); % Actuator bandwidth.
W3_1 = 1;%makeweight(0.01,[1 0.1],10); % No sensor noise or model uncertainty for now.

W1_2 = 1; 
W2_2 = [];
W3_2 = 1;

% % Some realistic weights
W1_2 = makeweight(50,[.3 0.15],0.01);
W2_2 = makeweight(0.1,[32 0.32],1); % Actuator bandwidth.
W3_2 = 1;%makeweight(0.01,[1 0.1],10); % No sensor noise or model uncertainty for now.

figure; clf;
bodemag(1/tf(W1_2), 1/tf(W2_2), 1/tf(W3_2));
legend('1/W1','1/W2','1/W3');

% Default weights
W1_3 = 1; 
W2_3 = [];
W3_3 = 1;

% % Some realistic weights
W1_3 = makeweight(60,[1 0.40],0.35);
W2_3 = makeweight(0.1,[32 0.32],1); % Actuator bandwidth.
W3_3 = 1;%makeweight(0.01,[1 0.1],10); % No sensor noise or model uncertainty for now.

% Weight S - I find it weird that this transfer function is order 6
% e_s = .0001;
% M_s = 2;
% omega_b = 10;
% k_s = 1;
% W1_3 = (s + omega_b*M_s^(1/k_s))/(M_s^(1/k_s)*(s+omega_b*e_s^(1/k_s)))^k_s;

% Weight T - Having M_t as 6 seems off to me... usually its ~2 for zeta =
% 0.7.  
% e_t = .001;
% M_t = 2;
% omega_bt = 100;
% k_t = 1;
% W3_3 = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;

figure; clf;
bodemag(1/tf(W1_3),1/tf(W2_3),1/tf(W3_3));
legend('1/W1','1/W2','1/W3');

%% Design controller 

[K1_o,CL1,gamma1] = mixsyn(quadModel1,W1_1,W2_1,W3_1);
disp(['Gamma1: ', num2str(gamma1)]);

[K2_o,CL2,gamma2] = mixsyn(quadModel2,W1_2,W2_2,W3_2);
disp(['Gamma2: ', num2str(gamma2)]);

[K3_o,CL3,gamma3] = mixsyn(quadModel3,W1_3,W2_3,W3_3);
disp(['Gamma3: ', num2str(gamma3)]);


%% S & T
S1 = feedback(1,quadModel1*K1_o);
T1 = 1-S1;

S2 = feedback(1,quadModel2*K2_o);
T2 = 1-S2;

S3 = feedback(1,quadModel3*K3_o);
T3 = 1-S3;

%% Plot
% Plot 1
t = 0:0.01:40;
figure; clf;
subplot(2,2,1:2); bodemag(S1,T1,K1_o*S1); legend('S','T','KS','Location','SouthEast'); grid on;

subplot(2,2,4); u = step(CL1(2),t); 
plot(t,u); title('Control u(t)'); xlabel('t (sec)'); grid on;

subplot(2,2,3); y = step(CL1(3),t);
hold on; plot(t,y); title('Step Response'); ylim([0,1.1]); xlabel('t (sec)'); grid on;


set(findall(gcf,'type','line'),'linewidth',1);

% Plot 2
figure; clf;
subplot(2,2,1:2); bodemag(S2,T2,K2_o*S2); legend('S','T','KS','Location','SouthEast'); grid on;

subplot(2,2,4); u = step(CL2(2),t); 
plot(t,u); title('Control u(t)'); xlabel('t (sec)'); grid on;

subplot(2,2,3); y = step(CL2(3),t);
hold on; plot(t,y); title('Step Response'); ylim([0,1.1]); xlabel('t (sec)'); grid on;


set(findall(gcf,'type','line'),'linewidth',1);

% Plot 3
figure; clf;
subplot(2,2,1:2); bodemag(S3,T3,K3_o*S3); legend('S','T','KS','Location','SouthEast'); grid on;

subplot(2,2,4); u = step(CL3(2),t); 
plot(t,u); title('Control u(t)'); xlabel('t (sec)'); grid on;

subplot(2,2,3); y = step(CL3(3),t);
hold on; plot(t,y); title('Step Response'); ylim([0,1.1]); xlabel('t (sec)'); grid on;


set(findall(gcf,'type','line'),'linewidth',1);

figure
y = step(CL1(3),t);
hold on; plot(t,y); title('Step Response'); ylim([0,1.1]); xlabel('t (sec)'); grid on;
ylabel('x')
set(findall(gcf,'type','line'),'linewidth',1);

figure
y = step(CL2(3),t);
hold on; plot(t,y); title('Step Response'); ylim([0,1.1]); xlabel('t (sec)'); grid on;
ylabel('y')
set(findall(gcf,'type','line'),'linewidth',1);

figure 
y = step(CL3(3),t);
hold on; plot(t,y); title('Step Response'); ylim([0,1.1]); xlabel('t (sec)'); grid on;
ylabel('z')
set(findall(gcf,'type','line'),'linewidth',1);

