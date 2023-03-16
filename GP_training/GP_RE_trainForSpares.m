%% Training GP models

% Transfer continuous tf to discrete. 
clear all; close all; clc;

load('G1.mat');
Fs = 10;
dt = 1/Fs;
% obtain ss function from the identified tr funciton
G1_13_T = 0.324;
G1_13_N = 2; % n=2 for pade approximation
[G1_13_num,G1_13_den] = pade(G1_13_T,G1_13_N); 

s = tf('s');
sys = exp(-G1_13_T*s);    
sys_delay = pade(sys,G1_13_N);

num_tr = [0, 5.41, 1];
den_tr = [4.15*4.15, 2*0.54*4.15, 1];
sys_trr = tf(num_tr,den_tr);
sys_tr = sys_trr*sys_delay;

sys_tr_d = c2d(sys_tr,dt);  % TRANFER CONTINUOUS TO DISCRETE !!!
[sys_tr_d_num, sys_tr_d_den] = tfdata(sys_tr_d, 'v');

%% Prepare data logdata1, logdata2, logdata3, two person in each dataset, 1/5 points for training.
% Group 1, logdata1_YN
N= length(logdata1_YN(:,8))-1;
G1_logdata1_YN_y = zeros(N-1, 1);

Y_G1_logdata1_YN_temp = [logdata1_YN(:,8)];
y_G1_logdata1_YN_diff = Y_G1_logdata1_YN_temp(1:length(G1_logdata1_YN_y)) - G1_logdata1_YN_y;
y_G1_logdata1_YN = y_G1_logdata1_YN_diff(2:length(G1_logdata1_YN_y));

X_G1_logdata1_YN_temp = [logdata1_YN(:,8) logdata1_YN(:,5)];
X_G1_logdata1_YN = X_G1_logdata1_YN_temp(1:length(G1_logdata1_YN_y)-1, :);

% Group 1, logdata1_ZH
N= length(logdata1_ZH(:,8))-1;
G1_logdata1_ZH_y = zeros(N-1, 1);

Y_G1_logdata1_ZH_temp = [logdata1_ZH(:,8)];
y_G1_logdata1_ZH_diff = Y_G1_logdata1_ZH_temp(1:length(G1_logdata1_ZH_y)) - G1_logdata1_ZH_y;
y_G1_logdata1_ZH = y_G1_logdata1_ZH_diff(2:length(G1_logdata1_ZH_y));

X_G1_logdata1_ZH_temp = [logdata1_ZH(:,8) logdata1_ZH(:,5)];
X_G1_logdata1_ZH = X_G1_logdata1_ZH_temp(1:length(G1_logdata1_ZH_y)-1, :);

% Group 1, logdata1_RZ
N= length(logdata1_RZ(:,8))-1;
G1_logdata1_RZ_y = zeros(N-1, 1);

Y_G1_logdata1_RZ_temp = [logdata1_RZ(:,8)];
y_G1_logdata1_RZ_diff = Y_G1_logdata1_RZ_temp(1:length(G1_logdata1_RZ_y)) - G1_logdata1_RZ_y;
y_G1_logdata1_RZ = y_G1_logdata1_RZ_diff(2:length(G1_logdata1_RZ_y));

X_G1_logdata1_RZ_temp = [logdata1_RZ(:,8) logdata1_RZ(:,5)];
X_G1_logdata1_RZ = X_G1_logdata1_RZ_temp(1:length(G1_logdata1_RZ_y)-1, :);

% Group 1, logdata3_YN
N= length(logdata3_YN(:,8))-1;
G1_logdata3_YN_y = zeros(N-1, 1);

Y_G1_logdata3_YN_temp = [logdata3_YN(:,8)];
y_G1_logdata3_YN_diff = Y_G1_logdata3_YN_temp(1:length(G1_logdata3_YN_y)) - G1_logdata3_YN_y;
y_G1_logdata3_YN = y_G1_logdata3_YN_diff(2:length(G1_logdata3_YN_y));

X_G1_logdata3_YN_temp = [logdata3_YN(:,8) logdata3_YN(:,5)];
X_G1_logdata3_YN = X_G1_logdata3_YN_temp(1:length(G1_logdata3_YN_y)-1, :);

% Group 1, logdata3_ZH
N= length(logdata3_ZH(:,8))-1;
G1_logdata3_ZH_y = zeros(N-1, 1);

Y_G1_logdata3_ZH_temp = [logdata3_ZH(:,8)];
y_G1_logdata3_ZH_diff = Y_G1_logdata3_ZH_temp(1:length(G1_logdata3_ZH_y)) - G1_logdata3_ZH_y;
y_G1_logdata3_ZH = y_G1_logdata3_ZH_diff(2:length(G1_logdata3_ZH_y));

X_G1_logdata3_ZH_temp = [logdata3_ZH(:,8) logdata3_ZH(:,5)];
X_G1_logdata3_ZH = X_G1_logdata3_ZH_temp(1:length(G1_logdata3_ZH_y)-1, :);

% Group 1, logdata3_RZ
N= length(logdata3_RZ(:,8))-1;
G1_logdata3_RZ_y = zeros(N-1, 1);

Y_G1_logdata3_RZ_temp = [logdata3_RZ(:,8)];
y_G1_logdata3_RZ_diff = Y_G1_logdata3_RZ_temp(1:length(G1_logdata3_RZ_y)) - G1_logdata3_RZ_y;
y_G1_logdata3_RZ = y_G1_logdata3_RZ_diff(2:length(G1_logdata3_RZ_y));

X_G1_logdata3_RZ_temp = [logdata3_RZ(:,8) logdata3_RZ(:,5)];
X_G1_logdata3_RZ = X_G1_logdata3_RZ_temp(1:length(G1_logdata3_RZ_y)-1, :);

% Group 1, logdata2_YN
N= length(logdata2_YN(:,8))-1;
G1_logdata2_YN_y = zeros(N-1, 1);

Y_G1_logdata2_YN_temp = [logdata2_YN(:,8)];
y_G1_logdata2_YN_diff = Y_G1_logdata2_YN_temp(1:length(G1_logdata2_YN_y)) - G1_logdata2_YN_y;
y_G1_logdata2_YN = y_G1_logdata2_YN_diff(2:length(G1_logdata2_YN_y));

X_G1_logdata2_YN_temp = [logdata2_YN(:,8) logdata2_YN(:,5)];
X_G1_logdata2_YN = X_G1_logdata2_YN_temp(1:length(G1_logdata2_YN_y)-1, :);

% Group 1, logdata2_ZH
N= length(logdata2_ZH(:,8))-1;
G1_logdata2_ZH_y = zeros(N-1, 1);

Y_G1_logdata2_ZH_temp = [logdata2_ZH(:,8)];
y_G1_logdata2_ZH_diff = Y_G1_logdata2_ZH_temp(1:length(G1_logdata2_ZH_y)) - G1_logdata2_ZH_y;
y_G1_logdata2_ZH = y_G1_logdata2_ZH_diff(2:length(G1_logdata2_ZH_y));

X_G1_logdata2_ZH_temp = [logdata2_ZH(:,8) logdata2_ZH(:,5)];
X_G1_logdata2_ZH = X_G1_logdata2_ZH_temp(1:length(G1_logdata2_ZH_y)-1, :);

% Group 1, logdata2_RZ
N= length(logdata2_RZ(:,8))-1;
G1_logdata2_RZ_y = zeros(N-1, 1);

Y_G1_logdata2_RZ_temp = [logdata2_RZ(:,8)];
y_G1_logdata2_RZ_diff = Y_G1_logdata2_RZ_temp(1:length(G1_logdata2_RZ_y)) - G1_logdata2_RZ_y;
y_G1_logdata2_RZ = y_G1_logdata2_RZ_diff(2:length(G1_logdata2_RZ_y));

X_G1_logdata2_RZ_temp = [logdata2_RZ(:,8) logdata2_RZ(:,5)];
X_G1_logdata2_RZ = X_G1_logdata2_RZ_temp(1:length(G1_logdata2_RZ_y)-1, :);

% % stack all
% y_G1_13 = [y_G1_logdata1_YN; y_G1_logdata1_ZH; y_G1_logdata1_RZ; y_G1_logdata3_YN; y_G1_logdata3_ZH; y_G1_logdata3_RZ; y_G1_logdata2_YN; y_G1_logdata2_ZH; y_G1_logdata2_RZ];
% X_G1_13 = [X_G1_logdata1_YN; X_G1_logdata1_ZH; X_G1_logdata1_RZ; X_G1_logdata3_YN; X_G1_logdata3_ZH; X_G1_logdata3_RZ; X_G1_logdata2_YN; X_G1_logdata2_ZH; X_G1_logdata2_RZ];

% % 1_YN 2_RZ 3_ZH for training
% y_G1_13_temp = [y_G1_logdata1_YN; y_G1_logdata1_ZH; y_G1_logdata2_ZH; y_G1_logdata2_RZ; y_G1_logdata3_ZH; y_G1_logdata3_RZ];
% X_G1_13_temp = [X_G1_logdata1_YN; X_G1_logdata1_ZH; X_G1_logdata2_ZH; X_G1_logdata2_RZ; X_G1_logdata3_ZH; X_G1_logdata3_RZ];
% 
% X_G1_13 = X_G1_13_temp(1:5:end, :);
% y_G1_13 = y_G1_13_temp(1:5:end);

% 1_YN 2_RZ 3_ZH for training
y_G1_13_temp = [y_G1_logdata1_YN; y_G1_logdata1_ZH; y_G1_logdata2_ZH; y_G1_logdata2_RZ; y_G1_logdata3_ZH; y_G1_logdata3_RZ];
X_G1_13_temp = [X_G1_logdata1_YN; X_G1_logdata1_ZH; X_G1_logdata2_ZH; X_G1_logdata2_RZ; X_G1_logdata3_ZH; X_G1_logdata3_RZ];

% % NEW training data logdata1 logdata2 ONLY NOT WORKING 
% y_G1_13_temp = [y_G1_logdata1_YN; y_G1_logdata1_ZH; y_G1_logdata1_RZ; y_G1_logdata2_YN; y_G1_logdata2_ZH; y_G1_logdata2_RZ];
% X_G1_13_temp = [X_G1_logdata1_YN; X_G1_logdata1_ZH; X_G1_logdata1_RZ; X_G1_logdata2_YN; X_G1_logdata2_ZH; X_G1_logdata2_RZ];

X_G1_13 = X_G1_13_temp(1:5:end, :);
y_G1_13 = y_G1_13_temp(1:5:end);
%%
% Train and save gpr. Comment this after  finshing training.
% https://www.mathworks.com/help/stats/fitrgp.html#butnn96-PredictMethod
% Set nondefault parameters by passing a vector of optimizableVariable objects 
% that have nondefault values. For example,
% params = hyperparameters('fitrgp',meas,species);

load('gpr_medium.mat')
% gpr_medium = compact(gpr_medium);
params=hyperparameters('fitrgp',X_G1_13,y_G1_13);

tic

% model = fitrgp(X_G1_13,y_G1_13,'KernelFunction','squaredExponential','FitMethod', ...
%                 'none','PredictMethod','exact','OptimizeHyperparameters',params);

model = fitrgp(X_G1_13,y_G1_13,'KernelFunction','squaredExponential','FitMethod', ...
                'none','PredictMethod','fic','ActiveSetSize',20);
% 
% model = fitrgp(X_G1_13,y_G1_13,'KernelFunction','squaredExponential','FitMethod', ...
%                 'none','PredictMethod','exact');

% model = fitrgp(X_G1_13,y_G1_13,'KernelFunction','squaredExponential','FitMethod', ...
%                 'none','PredictMethod','fic');
toc

% Elapsed time is 1620.147879 seconds. 1:5:end
% Elapsed time is 20309.581139 seconds. 1:2:end
%%
% save the trained GPR model
gpr_sparse = model;
save gpr_sparse;

%% Test with G1_logdata1

load gpr_sparse;
% gpr_sparse = compact(gpr_sparse);
% arx with gpr_G1_logdata1_YN

N= length(logdata1_YN(:,8))-1;
G1_logdata1_YN_u = [logdata1_YN(:,5)]';
G1_logdata1_YN_u = G1_logdata1_YN_u(1:(N-1));

G1_logdata1_YN_y = zeros(N-1, 1); 
G1_logdata1_YN_y_true = zeros(N-1, 1); 

y_gpr_G1_logdata1_YN = zeros(N-1, 1);
y_gpr_G1_logdata1_YN_sd = zeros(N-1, 1);

for k = 5:N-1
    [m,v,~] = predict(gpr_sparse, X_G1_logdata1_YN(k-1, :));
    y_gpr_G1_logdata1_YN(k-1,:) = m;
    y_gpr_G1_logdata1_YN_sd(k-1,:) = v;

    G1_logdata1_YN_y(k) = -sys_tr_d_den(2) * G1_logdata1_YN_y(k-1) - sys_tr_d_den(3) * G1_logdata1_YN_y(k-2) - sys_tr_d_den(4) * G1_logdata1_YN_y(k-3) - sys_tr_d_den(5) * G1_logdata1_YN_y(k-4) ...
          + sys_tr_d_num(2) * G1_logdata1_YN_u(k-1) + sys_tr_d_num(3) * G1_logdata1_YN_u(k-2) + sys_tr_d_num(4) * G1_logdata1_YN_u(k-3) + sys_tr_d_num(5) * G1_logdata1_YN_u(k-4);
    G1_logdata1_YN_y_true(k) = G1_logdata1_YN_y(k) + y_gpr_G1_logdata1_YN(k-1);
end

figure()
subplot(311)
plot(G1_logdata1_YN_y_true,'g','LineWidth',1.0); grid on; hold on;
% plot(y_true,'-bo','LineWidth',1.0, 'MarkerSize',3); grid on; hold on;
TIME = (1:N-1)';
% plot(TIME, G1_logdata1_YN_y(2:N-1)+y_gpr_G1_logdata1_YN,'k--','LineWidth',1.2); grid on; hold on;
patch([TIME;flipud(TIME)],[G1_logdata1_YN_y_true(1:N-1)+3*y_gpr_G1_logdata1_YN_sd;flipud(G1_logdata1_YN_y_true(1:N-1)-3*y_gpr_G1_logdata1_YN_sd)],'b','FaceAlpha',0.1); % Prediction intervals
plot(logdata1_YN(:,8),'m','LineWidth',1.0); grid on;
legend(["arx + gpr", "3\sigma", "human vel"],'Location','northeast');
ylabel('Human Velocity (m/s)','FontSize',14)
% xlabel('Time Steps (10HZ)','FontSize',14)
axis([0 N -10 40])
title("arx",'FontSize',16);

% arx with gpr_G1_logdata1_ZH

N= length(logdata1_ZH(:,8))-1;
G1_logdata1_ZH_u = [logdata1_ZH(:,5)]';
G1_logdata1_ZH_u = G1_logdata1_ZH_u(1:(N-1));

G1_logdata1_ZH_y = zeros(N-1, 1); 
G1_logdata1_ZH_y_true = zeros(N-1, 1); 

y_gpr_G1_logdata1_ZH = zeros(N-1, 1);
y_gpr_G1_logdata1_ZH_sd = zeros(N-1, 1);

for k = 5:N-1
    [m,v,~] = predict(gpr_sparse, X_G1_logdata1_ZH(k-1, :));
    y_gpr_G1_logdata1_ZH(k-1,:) = m;
    y_gpr_G1_logdata1_ZH_sd(k-1,:) = v;

    G1_logdata1_ZH_y(k) = -sys_tr_d_den(2) * G1_logdata1_ZH_y(k-1) - sys_tr_d_den(3) * G1_logdata1_ZH_y(k-2) - sys_tr_d_den(4) * G1_logdata1_ZH_y(k-3) - sys_tr_d_den(5) * G1_logdata1_ZH_y(k-4) ...
          + sys_tr_d_num(2) * G1_logdata1_ZH_u(k-1) + sys_tr_d_num(3) * G1_logdata1_ZH_u(k-2) + sys_tr_d_num(4) * G1_logdata1_ZH_u(k-3) + sys_tr_d_num(5) * G1_logdata1_ZH_u(k-4);
    G1_logdata1_ZH_y_true(k) = G1_logdata1_ZH_y(k) + y_gpr_G1_logdata1_ZH(k-1);
end

subplot(312)
plot(G1_logdata1_ZH_y_true,'g','LineWidth',1.0); grid on; hold on;
% plot(y_true,'-bo','LineWidth',1.0, 'MarkerSize',3); grid on; hold on;
TIME = (1:N-1)';
% plot(TIME, G1_logdata1_ZH_y(2:N-1)+y_gpr_G1_logdata1_ZH,'k--','LineWidth',1.2); grid on; hold on;
patch([TIME;flipud(TIME)],[G1_logdata1_ZH_y_true(1:N-1)+3*y_gpr_G1_logdata1_ZH_sd;flipud(G1_logdata1_ZH_y_true(1:N-1)-3*y_gpr_G1_logdata1_ZH_sd)],'b','FaceAlpha',0.1); % Prediction intervals
plot(logdata1_ZH(:,8),'m','LineWidth',1.0); grid on;
legend(["arx + gpr", "3\sigma", "human vel"],'Location','northeast');
ylabel('Human Velocity (m/s)','FontSize',14)
% xlabel('Time Steps (10HZ)','FontSize',14)
axis([0 N -10 40])
title("arx",'FontSize',16);

% arx with gpr_G1_logdata1_RZ

N= length(logdata1_RZ(:,8))-1;
G1_logdata1_RZ_u = [logdata1_RZ(:,5)]';
G1_logdata1_RZ_u = G1_logdata1_RZ_u(1:(N-1));

G1_logdata1_RZ_y = zeros(N-1, 1); 
G1_logdata1_RZ_y_true = zeros(N-1, 1); 

y_gpr_G1_logdata1_RZ = zeros(N-1, 1);
y_gpr_G1_logdata1_RZ_sd = zeros(N-1, 1);

for k = 5:N-1
    [m,v,~] = predict(gpr_sparse, X_G1_logdata1_RZ(k-1, :));
    y_gpr_G1_logdata1_RZ(k-1,:) = m;
    y_gpr_G1_logdata1_RZ_sd(k-1,:) = v;

    G1_logdata1_RZ_y(k) = -sys_tr_d_den(2) * G1_logdata1_RZ_y(k-1) - sys_tr_d_den(3) * G1_logdata1_RZ_y(k-2) - sys_tr_d_den(4) * G1_logdata1_RZ_y(k-3) - sys_tr_d_den(5) * G1_logdata1_RZ_y(k-4) ...
          + sys_tr_d_num(2) * G1_logdata1_RZ_u(k-1) + sys_tr_d_num(3) * G1_logdata1_RZ_u(k-2) + sys_tr_d_num(4) * G1_logdata1_RZ_u(k-3) + sys_tr_d_num(5) * G1_logdata1_RZ_u(k-4);
    G1_logdata1_RZ_y_true(k) = G1_logdata1_RZ_y(k) + y_gpr_G1_logdata1_RZ(k-1);
end

subplot(313)
plot(G1_logdata1_RZ_y_true,'g','LineWidth',1.0); grid on; hold on;
% plot(y_true,'-bo','LineWidth',1.0, 'MarkerSize',3); grid on; hold on;
TIME = (1:N-1)';
% plot(TIME, G1_logdata1_RZ_y(2:N-1)+y_gpr_G1_logdata1_RZ,'k--','LineWidth',1.2); grid on; hold on;
patch([TIME;flipud(TIME)],[G1_logdata1_RZ_y_true(1:N-1)+3*y_gpr_G1_logdata1_RZ_sd;flipud(G1_logdata1_RZ_y_true(1:N-1)-3*y_gpr_G1_logdata1_RZ_sd)],'b','FaceAlpha',0.1); % Prediction intervals
plot(logdata1_RZ(:,8),'m','LineWidth',1.0); grid on;
legend(["arx + gpr", "3\sigma", "human vel"],'Location','northeast');
ylabel('Human Velocity (m/s)','FontSize',14)
% xlabel('Time Steps (10HZ)','FontSize',14)
axis([0 N -10 40])
title("arx",'FontSize',16);

%% Test with G1_logdata3

load gpr_sparse;

% arx with gpr_G1_logdata3_RZ

N= length(logdata3_RZ(:,8))-1;
G1_logdata3_RZ_u = [logdata3_RZ(:,5)]';
G1_logdata3_RZ_u = G1_logdata3_RZ_u(1:(N-1));

G1_logdata3_RZ_y = zeros(N-1, 1); 
G1_logdata3_RZ_y_true = zeros(N-1, 1); 

y_gpr_G1_logdata3_RZ = zeros(N-1, 1);
y_gpr_G1_logdata3_RZ_sd = zeros(N-1, 1);

for k = 5:N-1
    [m,v,~] = predict(gpr_sparse, X_G1_logdata3_RZ(k-1, :));
    y_gpr_G1_logdata3_RZ(k-1,:) = m;
    y_gpr_G1_logdata3_RZ_sd(k-1,:) = v;

    G1_logdata3_RZ_y(k) = -sys_tr_d_den(2) * G1_logdata3_RZ_y(k-1) - sys_tr_d_den(3) * G1_logdata3_RZ_y(k-2) - sys_tr_d_den(4) * G1_logdata3_RZ_y(k-3) - sys_tr_d_den(5) * G1_logdata3_RZ_y(k-4) ...
          + sys_tr_d_num(2) * G1_logdata3_RZ_u(k-1) + sys_tr_d_num(3) * G1_logdata3_RZ_u(k-2) + sys_tr_d_num(4) * G1_logdata3_RZ_u(k-3) + sys_tr_d_num(5) * G1_logdata3_RZ_u(k-4);
    G1_logdata3_RZ_y_true(k) = G1_logdata3_RZ_y(k) + y_gpr_G1_logdata3_RZ(k-1);
end

load gpr_sparse;

% arx with gpr_G1_logdata3_YN

N= length(logdata3_YN(:,8))-1;
G1_logdata3_YN_u = [logdata3_YN(:,5)]';
G1_logdata3_YN_u = G1_logdata3_YN_u(1:(N-1));

G1_logdata3_YN_y = zeros(N-1, 1); 
G1_logdata3_YN_y_true = zeros(N-1, 1); 

y_gpr_G1_logdata3_YN = zeros(N-1, 1);
y_gpr_G1_logdata3_YN_sd = zeros(N-1, 1);

for k = 5:N-1
    [m,v,~] = predict(gpr_sparse, X_G1_logdata3_YN(k-1, :));
    y_gpr_G1_logdata3_YN(k-1,:) = m;
    y_gpr_G1_logdata3_YN_sd(k-1,:) = v;

    G1_logdata3_YN_y(k) = -sys_tr_d_den(2) * G1_logdata3_YN_y(k-1) - sys_tr_d_den(3) * G1_logdata3_YN_y(k-2) - sys_tr_d_den(4) * G1_logdata3_YN_y(k-3) - sys_tr_d_den(5) * G1_logdata3_YN_y(k-4) ...
          + sys_tr_d_num(2) * G1_logdata3_YN_u(k-1) + sys_tr_d_num(3) * G1_logdata3_YN_u(k-2) + sys_tr_d_num(4) * G1_logdata3_YN_u(k-3) + sys_tr_d_num(5) * G1_logdata3_YN_u(k-4);
    G1_logdata3_YN_y_true(k) = G1_logdata3_YN_y(k) + y_gpr_G1_logdata3_YN(k-1);
end

figure()
subplot(311)
plot(G1_logdata3_YN_y_true,'g','LineWidth',1.0); grid on; hold on;
% plot(y_true,'-bo','LineWidth',1.0, 'MarkerSize',3); grid on; hold on;
TIME = (1:N-1)';
% plot(TIME, G1_logdata3_YN_y(2:N-1)+y_gpr_G1_logdata3_YN,'k--','LineWidth',1.2); grid on; hold on;
patch([TIME;flipud(TIME)],[G1_logdata3_YN_y_true(1:N-1)+3*y_gpr_G1_logdata3_YN_sd;flipud(G1_logdata3_YN_y_true(1:N-1)-3*y_gpr_G1_logdata3_YN_sd)],'b','FaceAlpha',0.1); % Prediction intervals
plot(logdata3_YN(:,8),'m','LineWidth',1.0); grid on;
legend(["arx + gpr", "3\sigma", "human vel"],'Location','northeast');
ylabel('Human Velocity (m/s)','FontSize',14)
% xlabel('Time Steps (10HZ)','FontSize',14)
axis([0 N -10 40])
title("arx",'FontSize',16);

% arx with gpr_G1_logdata3_ZH

N= length(logdata3_ZH(:,8))-1;
G1_logdata3_ZH_u = [logdata3_ZH(:,5)]';
G1_logdata3_ZH_u = G1_logdata3_ZH_u(1:(N-1));

G1_logdata3_ZH_y = zeros(N-1, 1); 
G1_logdata3_ZH_y_true = zeros(N-1, 1); 

y_gpr_G1_logdata3_ZH = zeros(N-1, 1);
y_gpr_G1_logdata3_ZH_sd = zeros(N-1, 1);

for k = 5:N-1
    [m,v,~] = predict(gpr_sparse, X_G1_logdata3_ZH(k-1, :));
    y_gpr_G1_logdata3_ZH(k-1,:) = m;
    y_gpr_G1_logdata3_ZH_sd(k-1,:) = v;

    G1_logdata3_ZH_y(k) = -sys_tr_d_den(2) * G1_logdata3_ZH_y(k-1) - sys_tr_d_den(3) * G1_logdata3_ZH_y(k-2) - sys_tr_d_den(4) * G1_logdata3_ZH_y(k-3) - sys_tr_d_den(5) * G1_logdata3_ZH_y(k-4) ...
          + sys_tr_d_num(2) * G1_logdata3_ZH_u(k-1) + sys_tr_d_num(3) * G1_logdata3_ZH_u(k-2) + sys_tr_d_num(4) * G1_logdata3_ZH_u(k-3) + sys_tr_d_num(5) * G1_logdata3_ZH_u(k-4);
    G1_logdata3_ZH_y_true(k) = G1_logdata3_ZH_y(k) + y_gpr_G1_logdata3_ZH(k-1);
end

subplot(312)
plot(G1_logdata3_ZH_y_true,'g','LineWidth',1.0); grid on; hold on;
% plot(y_true,'-bo','LineWidth',1.0, 'MarkerSize',3); grid on; hold on;
TIME = (1:N-1)';
% plot(TIME, G1_logdata3_ZH_y(2:N-1)+y_gpr_G1_logdata3_ZH,'k--','LineWidth',1.2); grid on; hold on;
patch([TIME;flipud(TIME)],[G1_logdata3_ZH_y_true(1:N-1)+3*y_gpr_G1_logdata3_ZH_sd;flipud(G1_logdata3_ZH_y_true(1:N-1)-3*y_gpr_G1_logdata3_ZH_sd)],'b','FaceAlpha',0.1); % Prediction intervals
plot(logdata3_ZH(:,8),'m','LineWidth',1.0); grid on;
legend(["arx + gpr", "3\sigma", "human vel"],'Location','northeast');
ylabel('Human Velocity (m/s)','FontSize',14)
% xlabel('Time Steps (10HZ)','FontSize',14)
axis([0 N -10 40])
title("arx",'FontSize',16);

% arx with gpr_G1_logdata3_RZ

N= length(logdata3_RZ(:,8))-1;
G1_logdata3_RZ_u = [logdata3_RZ(:,5)]';
G1_logdata3_RZ_u = G1_logdata3_RZ_u(1:(N-1));

G1_logdata3_RZ_y = zeros(N-1, 1); 
G1_logdata3_RZ_y_true = zeros(N-1, 1); 

y_gpr_G1_logdata3_RZ = zeros(N-1, 1);
y_gpr_G1_logdata3_RZ_sd = zeros(N-1, 1);

for k = 5:N-1
    [m,v,~] = predict(gpr_sparse, X_G1_logdata3_RZ(k-1, :));
    y_gpr_G1_logdata3_RZ(k-1,:) = m;
    y_gpr_G1_logdata3_RZ_sd(k-1,:) = v;

    G1_logdata3_RZ_y(k) = -sys_tr_d_den(2) * G1_logdata3_RZ_y(k-1) - sys_tr_d_den(3) * G1_logdata3_RZ_y(k-2) - sys_tr_d_den(4) * G1_logdata3_RZ_y(k-3) - sys_tr_d_den(5) * G1_logdata3_RZ_y(k-4) ...
          + sys_tr_d_num(2) * G1_logdata3_RZ_u(k-1) + sys_tr_d_num(3) * G1_logdata3_RZ_u(k-2) + sys_tr_d_num(4) * G1_logdata3_RZ_u(k-3) + sys_tr_d_num(5) * G1_logdata3_RZ_u(k-4);
    G1_logdata3_RZ_y_true(k) = G1_logdata3_RZ_y(k) + y_gpr_G1_logdata3_RZ(k-1);
end

subplot(313)
plot(G1_logdata3_RZ_y_true,'g','LineWidth',1.0); grid on; hold on;
% plot(y_true,'-bo','LineWidth',1.0, 'MarkerSize',3); grid on; hold on;
TIME = (1:N-1)';
% plot(TIME, G1_logdata3_RZ_y(2:N-1)+y_gpr_G1_logdata3_RZ,'k--','LineWidth',1.2); grid on; hold on;
patch([TIME;flipud(TIME)],[G1_logdata3_RZ_y_true(1:N-1)+3*y_gpr_G1_logdata3_RZ_sd;flipud(G1_logdata3_RZ_y_true(1:N-1)-3*y_gpr_G1_logdata3_RZ_sd)],'b','FaceAlpha',0.1); % Prediction intervals
plot(logdata3_RZ(:,8),'m','LineWidth',1.0); grid on;
legend(["arx + gpr", "3\sigma", "human vel"],'Location','northeast');
ylabel('Human Velocity (m/s)','FontSize',14)
% xlabel('Time Steps (10HZ)','FontSize',14)
axis([0 N -10 40])
title("arx",'FontSize',16);
%% Test with G1_logdata2 

load gpr_sparse;

% arx with gpr_G1_logdata2_RZ

N= length(logdata2_RZ(:,8))-1;
G1_logdata2_RZ_u = [logdata2_RZ(:,5)]';
G1_logdata2_RZ_u = G1_logdata2_RZ_u(1:(N-1));

G1_logdata2_RZ_y = zeros(N-1, 1); 
G1_logdata2_RZ_y_true = zeros(N-1, 1); 

y_gpr_G1_logdata2_RZ = zeros(N-1, 1);
y_gpr_G1_logdata2_RZ_sd = zeros(N-1, 1);

for k = 5:N-1
    [m,v,~] = predict(gpr_sparse, X_G1_logdata2_RZ(k-1, :));
    y_gpr_G1_logdata2_RZ(k-1,:) = m;
    y_gpr_G1_logdata2_RZ_sd(k-1,:) = v;

    G1_logdata2_RZ_y(k) = -sys_tr_d_den(2) * G1_logdata2_RZ_y(k-1) - sys_tr_d_den(3) * G1_logdata2_RZ_y(k-2) - sys_tr_d_den(4) * G1_logdata2_RZ_y(k-3) - sys_tr_d_den(5) * G1_logdata2_RZ_y(k-4) ...
          + sys_tr_d_num(2) * G1_logdata2_RZ_u(k-1) + sys_tr_d_num(3) * G1_logdata2_RZ_u(k-2) + sys_tr_d_num(4) * G1_logdata2_RZ_u(k-3) + sys_tr_d_num(5) * G1_logdata2_RZ_u(k-4);
    G1_logdata2_RZ_y_true(k) = G1_logdata2_RZ_y(k) + y_gpr_G1_logdata2_RZ(k-1);
end

load gpr_sparse;

% arx with gpr_G1_logdata2_YN

N= length(logdata2_YN(:,8))-1;
G1_logdata2_YN_u = [logdata2_YN(:,5)]';
G1_logdata2_YN_u = G1_logdata2_YN_u(1:(N-1));

G1_logdata2_YN_y = zeros(N-1, 1); 
G1_logdata2_YN_y_true = zeros(N-1, 1); 

y_gpr_G1_logdata2_YN = zeros(N-1, 1);
y_gpr_G1_logdata2_YN_sd = zeros(N-1, 1);

for k = 5:N-1
    [m,v,~] = predict(gpr_sparse, X_G1_logdata2_YN(k-1, :));
    y_gpr_G1_logdata2_YN(k-1,:) = m;
    y_gpr_G1_logdata2_YN_sd(k-1,:) = v;

    G1_logdata2_YN_y(k) = -sys_tr_d_den(2) * G1_logdata2_YN_y(k-1) - sys_tr_d_den(3) * G1_logdata2_YN_y(k-2) - sys_tr_d_den(4) * G1_logdata2_YN_y(k-3) - sys_tr_d_den(5) * G1_logdata2_YN_y(k-4) ...
          + sys_tr_d_num(2) * G1_logdata2_YN_u(k-1) + sys_tr_d_num(3) * G1_logdata2_YN_u(k-2) + sys_tr_d_num(4) * G1_logdata2_YN_u(k-3) + sys_tr_d_num(5) * G1_logdata2_YN_u(k-4);
    G1_logdata2_YN_y_true(k) = G1_logdata2_YN_y(k) + y_gpr_G1_logdata2_YN(k-1);
end

figure()
subplot(311)
plot(G1_logdata2_YN_y_true,'g','LineWidth',1.0); grid on; hold on;
% plot(y_true,'-bo','LineWidth',1.0, 'MarkerSize',3); grid on; hold on;
TIME = (1:N-1)';
% plot(TIME, G1_logdata2_YN_y(2:N-1)+y_gpr_G1_logdata2_YN,'k--','LineWidth',1.2); grid on; hold on;
patch([TIME;flipud(TIME)],[G1_logdata2_YN_y_true(1:N-1)+3*y_gpr_G1_logdata2_YN_sd;flipud(G1_logdata2_YN_y_true(1:N-1)-3*y_gpr_G1_logdata2_YN_sd)],'b','FaceAlpha',0.1); % Prediction intervals
plot(logdata2_YN(:,8),'m','LineWidth',1.0); grid on;
legend(["arx + gpr", "3\sigma", "human vel"],'Location','northeast');
ylabel('Human Velocity (m/s)','FontSize',14)
% xlabel('Time Steps (10HZ)','FontSize',14)
axis([0 N -10 40])
title("arx",'FontSize',16);

% arx with gpr_G1_logdata2_ZH

N= length(logdata2_ZH(:,8))-1;
G1_logdata2_ZH_u = [logdata2_ZH(:,5)]';
G1_logdata2_ZH_u = G1_logdata2_ZH_u(1:(N-1));

G1_logdata2_ZH_y = zeros(N-1, 1); 
G1_logdata2_ZH_y_true = zeros(N-1, 1); 

y_gpr_G1_logdata2_ZH = zeros(N-1, 1);
y_gpr_G1_logdata2_ZH_sd = zeros(N-1, 1);

for k = 5:N-1
    [m,v,~] = predict(gpr_sparse, X_G1_logdata2_ZH(k-1, :));
    y_gpr_G1_logdata2_ZH(k-1,:) = m;
    y_gpr_G1_logdata2_ZH_sd(k-1,:) = v;

    G1_logdata2_ZH_y(k) = -sys_tr_d_den(2) * G1_logdata2_ZH_y(k-1) - sys_tr_d_den(3) * G1_logdata2_ZH_y(k-2) - sys_tr_d_den(4) * G1_logdata2_ZH_y(k-3) - sys_tr_d_den(5) * G1_logdata2_ZH_y(k-4) ...
          + sys_tr_d_num(2) * G1_logdata2_ZH_u(k-1) + sys_tr_d_num(3) * G1_logdata2_ZH_u(k-2) + sys_tr_d_num(4) * G1_logdata2_ZH_u(k-3) + sys_tr_d_num(5) * G1_logdata2_ZH_u(k-4);
    G1_logdata2_ZH_y_true(k) = G1_logdata2_ZH_y(k) + y_gpr_G1_logdata2_ZH(k-1);
end

subplot(312)
plot(G1_logdata2_ZH_y_true,'g','LineWidth',1.0); grid on; hold on;
% plot(y_true,'-bo','LineWidth',1.0, 'MarkerSize',3); grid on; hold on;
TIME = (1:N-1)';
% plot(TIME, G1_logdata2_ZH_y(2:N-1)+y_gpr_G1_logdata2_ZH,'k--','LineWidth',1.2); grid on; hold on;
patch([TIME;flipud(TIME)],[G1_logdata2_ZH_y_true(1:N-1)+3*y_gpr_G1_logdata2_ZH_sd;flipud(G1_logdata2_ZH_y_true(1:N-1)-3*y_gpr_G1_logdata2_ZH_sd)],'b','FaceAlpha',0.1); % Prediction intervals
plot(logdata2_ZH(:,8),'m','LineWidth',1.0); grid on;
legend(["arx + gpr", "3\sigma", "human vel"],'Location','northeast');
ylabel('Human Velocity (m/s)','FontSize',14)
% xlabel('Time Steps (10HZ)','FontSize',14)
axis([0 N -10 40])
title("arx",'FontSize',16);

% arx with gpr_G1_logdata2_RZ

N= length(logdata2_RZ(:,8))-1;
G1_logdata2_RZ_u = [logdata2_RZ(:,5)]';
G1_logdata2_RZ_u = G1_logdata2_RZ_u(1:(N-1));

G1_logdata2_RZ_y = zeros(N-1, 1); 
G1_logdata2_RZ_y_true = zeros(N-1, 1); 

y_gpr_G1_logdata2_RZ = zeros(N-1, 1);
y_gpr_G1_logdata2_RZ_sd = zeros(N-1, 1);

for k = 5:N-1
    [m,v,~] = predict(gpr_sparse, X_G1_logdata2_RZ(k-1, :));
    y_gpr_G1_logdata2_RZ(k-1,:) = m;
    y_gpr_G1_logdata2_RZ_sd(k-1,:) = v;

    G1_logdata2_RZ_y(k) = -sys_tr_d_den(2) * G1_logdata2_RZ_y(k-1) - sys_tr_d_den(3) * G1_logdata2_RZ_y(k-2) - sys_tr_d_den(4) * G1_logdata2_RZ_y(k-3) - sys_tr_d_den(5) * G1_logdata2_RZ_y(k-4) ...
          + sys_tr_d_num(2) * G1_logdata2_RZ_u(k-1) + sys_tr_d_num(3) * G1_logdata2_RZ_u(k-2) + sys_tr_d_num(4) * G1_logdata2_RZ_u(k-3) + sys_tr_d_num(5) * G1_logdata2_RZ_u(k-4);
    G1_logdata2_RZ_y_true(k) = G1_logdata2_RZ_y(k) + y_gpr_G1_logdata2_RZ(k-1);
end

subplot(313)
plot(G1_logdata2_RZ_y_true,'g','LineWidth',1.0); grid on; hold on;
% plot(y_true,'-bo','LineWidth',1.0, 'MarkerSize',3); grid on; hold on;
TIME = (1:N-1)';
% plot(TIME, G1_logdata2_RZ_y(2:N-1)+y_gpr_G1_logdata2_RZ,'k--','LineWidth',1.2); grid on; hold on;
patch([TIME;flipud(TIME)],[G1_logdata2_RZ_y_true(1:N-1)+3*y_gpr_G1_logdata2_RZ_sd;flipud(G1_logdata2_RZ_y_true(1:N-1)-3*y_gpr_G1_logdata2_RZ_sd)],'b','FaceAlpha',0.1); % Prediction intervals
plot(logdata2_RZ(:,8),'m','LineWidth',1.0); grid on;
legend(["arx + gpr", "3\sigma", "human vel"],'Location','northeast');
ylabel('Human Velocity (m/s)','FontSize',14)
% xlabel('Time Steps (10HZ)','FontSize',14)
axis([0 N -10 40])
title("arx",'FontSize',16);
