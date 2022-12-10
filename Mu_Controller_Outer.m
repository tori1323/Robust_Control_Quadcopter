load('Linear_Outter.mat')
system_o = ss(A_o,B_o,C_o,[])
system_o.u = 'u'; system_o.y = 'y';
G_o = tf(system_o)

% Design phi,theta,psi controllers
quadModel1 = G_o(1,3);
quadModel1.u = 'u'; quadModel1.y = 'y';
quadModel2 = G_o(2,2);
quadModel2.u = 'u'; quadModel2.y = 'y';
quadModel3 = G_o(3,1);
quadModel3.u = 'u'; quadModel3.y = 'y';

%% Mixsyn Approach

% weights
W1_1 = makeweight(50,[.3 0.1],0.01);
% Weight T - Having M_t as 6 seems off to me... usually its ~2 for zeta =
% 0.7.  
e_t = .001;
M_t = 2;
omega_bt = 100;
k_t = 2;
W1_1 = ((s + (omega_bt/(M_t^(1/k_t))))/((e_t^(1/k_t))*s+omega_bt))^k_t;

W1_1.u = 'ey'; W1_1.y = 'e1'

W1_2 = makeweight(50,[.3 0.15],0.01);
W1_2 = W1_1;
W1_2.u = 'ey'; W1_2.y = 'e2'

W1_3 = makeweight(60,[1 0.40],0.35);
W1_3 = W1_1;
W1_3.u = 'ey'; W1_3.y = 'e3'

sum = sumblk('ey=r-y',1);

%% Design controller 
[K1_o,CL1,gamma1] = mixsyn(quadModel1,W1_1,[],[]);
disp(['Gamma1: ', num2str(gamma1)]);

[K2_o,CL2,gamma2] = mixsyn(quadModel2,W1_2,[],[]);
disp(['Gamma2: ', num2str(gamma2)]);

[K3_o,CL3,gamma3] = mixsyn(quadModel3,W1_3,[],[]);
disp(['Gamma3: ', num2str(gamma3)]);
%%

unc = 0.3*ultidyn('unc',[1 1]);
unc.u = 'u0'; unc.y = 'z';
mod1 = sumblk('u = u0 + z',1);

% New Plant with Uncertainty
P_U1 = connect(quadModel1,unc,W1_1,sum,mod1,{'r','u0'},{'e1','y'});
Lunc1 = lft(P_U1,K1_o);


% New Plant with Uncertainty
P_U2 = connect(quadModel2,unc,W1_2,sum,mod1,{'r','u0'},{'e2','y'});
Lunc2 = lft(P_U2,K2_o);


% New Plant with Uncertainty
P_U3 = connect(quadModel3,unc,W1_3,sum,mod1,{'r','u0'},{'e3','y'});
Lunc3 = lft(P_U3,K3_o);
%%
% Stabiliity Robustness
[STABMARG1,DESTABUNC,REPORT,INFO] = robuststab(Lunc1)
[STABMARG2,DESTABUNC,REPORT,INFO] = robuststab(Lunc2)
[STABMARG3,DESTABUNC,REPORT,INFO] = robuststab(Lunc3)

[PERFMARG,WCU] = robgain(Lunc1,0.99,0.1)
[PERFMARG,WCU] = robgain(Lunc2,0.99,0.1)
[PERFMARG,WCU] = robgain(Lunc3,0.99,0.1)
%%

[K_dk1,clperf] = musyn(P_U1,1,1)
[K_dk2,clperf] = musyn(P_U2,1,1)
[K_dk3,clperf] = musyn(P_U3,1,1)

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

