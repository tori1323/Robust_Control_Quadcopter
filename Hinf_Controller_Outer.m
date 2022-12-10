load('Linear_Outter.mat')
system_o = ss(A_o,B_o,C_o,[])
G_o = tf(system_o)

%% Mixsyn Approach

% Default weights
W1 = 1; 
W2 = [];
W3 = 1;

% % Some realistic weights
W1 = makeweight(50,[1 0.5],0.01);
W2 = makeweight(0.1,[32 0.32],1); % Actuator bandwidth.
W3 = 1;%makeweight(0.01,[1 0.1],10); % No sensor noise or model uncertainty for now.

figure; clf;
bodemag(W1,W2);
legend('W1','W2');

%% Design controller 

[K,CL,gamma] = mixsyn(system_o,W1*eye(3),W2,W3*eye(3));
disp(['Gamma: ', num2str(gamma)]);

%% S & T
S = feedback(eye(3),G_o*K);
T = 1-S;

%% Plot
% Plot 1
t = 0:0.01:2;
figure; clf;
subplot(2,2,1:2); bodemag(S,T,K*S); legend('S','T','KS','Location','SouthEast'); grid on;

subplot(2,2,4); u = step(CL(2),t); 
plot(t,u); title('Control u(t)'); xlabel('t (sec)'); grid on;

subplot(2,2,3); y = step(CL(3),t);
hold on; plot(t,y); title('Step Response'); ylim([0,1.1]); xlabel('t (sec)'); grid on;


set(findall(gcf,'type','line'),'linewidth',1);