clear; clc; 
s = tf('s');
d2r = pi/180;

% Vehicle parameters
% Ix = 12.674*1E-3;
Ix = 0.01151;
l = 0.155;
a = 0.78; b = 0.1;

%G = (13.847)/((s+eps)^2*(s/10+1));  %======== Pitch Dynamics

quadModel = (l/Ix)/s^2;
actModel = a/(1+b*s); % Actuator model .. should be used to define 1/W2.
G = quadModel;

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

[K,CL,gamma] = mixsyn(G,W1,W2,W3);
disp(['Gamma: ', num2str(gamma)]);


%% Plot
S = feedback(1,G*K);
T = 1-S;

t = 0:0.01:2;
figure; clf;
subplot(2,2,1:2); bodemag(S,T,K*S); legend('S','T','KS','Location','SouthEast'); grid on;

subplot(2,2,4); u = step(CL(2),t); 
plot(t,u); title('Control u(t)'); xlabel('t (sec)'); grid on;

subplot(2,2,3); y = step(CL(3),t);
hold on; plot(t,y); title('Step Response'); ylim([0,1.1]); xlabel('t (sec)'); grid on;


set(findall(gcf,'type','line'),'linewidth',1);


