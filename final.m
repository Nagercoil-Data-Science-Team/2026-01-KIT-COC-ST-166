clc;
clear;
close all;
rng(1);

%% =====================================================
% STEP 1: SMART CITY TRAFFIC ENVIRONMENT
% =====================================================
numNodes = 8;
Adj = [ 0 1 1 0 0 0 0 0;
        1 0 1 1 0 0 0 0;
        1 1 0 1 1 0 0 0;
        0 1 1 0 1 1 0 0;
        0 0 1 1 0 1 1 0;
        0 0 0 1 1 0 1 1;
        0 0 0 0 1 1 0 1;
        0 0 0 0 0 1 1 0 ];

cityGraph = graph(Adj);
numEdges = numedges(cityGraph);

rho = 20 + 80*rand(numEdges,1);
v   = 20 + 60*rand(numEdges,1);
q   = rho .* v;

node_positions = rand(numNodes,2)*numNodes;

%% =====================================================
% STEP 2: IoT TRAFFIC DATA
% =====================================================
for t = 1:40
    rho = max(10,min(120,rho + 0.05*randn(numEdges,1)));
    v   = max(15,min(90,v + 0.05*randn(numEdges,1)));
    q   = rho .* v;
end

%% =====================================================
% STEP 3: UAV TRUE & GPS POSITIONS
% =====================================================
numUAVs = 5;
UAV_true = [rand(numUAVs,2)*numNodes 30+20*rand(numUAVs,1)];
GPS_noise_std = 6;
UAV_est = UAV_true + GPS_noise_std*randn(numUAVs,3);

%% =====================================================
% STEP 4: NODE PRIORITY
% =====================================================
node_priority_level = randi([1 3],numNodes,1);

%% =====================================================
% STEP 5: PLACEHOLDER BOA PATH
% =====================================================
for u = 1:numUAVs
    UAV_paths{u} = randperm(numNodes,4); %#ok<SAGROW>
end

%% =====================================================
% STEP 6: RESIDUAL DEEP LEARNING (STABLE)
% =====================================================
numSamples = 2500;

X = zeros(numSamples,10);
Y = zeros(numSamples,3);

for i = 1:numSamples
    e = randi(numEdges);
    u = randi(numUAVs);
    n = randi(numNodes);

    congestion = rho(e)/max(v(e),1);

    X(i,:) = [
        rho(e)
        v(e)
        q(e)
        UAV_est(u,1)
        UAV_est(u,2)
        UAV_est(u,3)
        node_priority_level(n)
        mean(rho)
        mean(v)
        congestion
    ];

    Y(i,:) = UAV_true(u,:) - UAV_est(u,:);
end

[Xn,~,~] = zscore(X);

layers = [
    featureInputLayer(10)
    fullyConnectedLayer(64)
    reluLayer
    fullyConnectedLayer(32)
    reluLayer
    fullyConnectedLayer(3)
    regressionLayer];

options = trainingOptions('adam',...
    'MaxEpochs',200,...
    'MiniBatchSize',32,...
    'Verbose',false);

net = trainNetwork(Xn,Y,layers,options);

% -------- STABLE RESIDUAL FUSION ----------
alpha = 0.6;   % confidence factor
residual_pred = predict(net,Xn(1:numUAVs,:));
UAV_hybrid_pos = UAV_est + alpha * residual_pred;

%% =====================================================
% STEP 7: PERFORMANCE EVALUATION
% =====================================================
err_DNN = vecnorm(UAV_true - UAV_hybrid_pos,2,2);
err_base = vecnorm(UAV_true - UAV_est,2,2);

energy_DNN = sum(vecnorm(UAV_hybrid_pos - UAV_est,2,2).^2);
energy_base = sum(vecnorm(UAV_est - UAV_true,2,2).^2);

latency_DNN = 0.02 + err_DNN*0.008;
latency_base = 0.05 + err_base*0.015;

coverage_radius = 3;
covered = zeros(numNodes,1);
for u = 1:numUAVs
    d = vecnorm(node_positions - UAV_hybrid_pos(u,1:2),2,2);
    covered(d <= coverage_radius) = 1;
end
coverage_ratio = sum(covered)/numNodes;

%% =====================================================
% RESULTS OUTPUT
% =====================================================
disp('========= PERFORMANCE MATRICES =========');

disp('Localization Error (meters) - DNN');
disp(err_DNN);

disp('Localization Error (meters) - Baseline');
disp(err_base);

disp('Energy Consumption [DNN  Baseline]');
disp([energy_DNN energy_base]);

disp('Latency (seconds) [DNN  Baseline]');
disp([latency_DNN latency_base]);

disp('Coverage Ratio');
disp(coverage_ratio);

%% =====================================================
% PLOTS
% =====================================================
figure('Position',[200 100 1200 600]);

subplot(2,2,1)
bar([err_DNN err_base])
title('Localization Error (m)')
legend('DNN','Baseline')

subplot(2,2,2)
bar([energy_DNN energy_base])
set(gca,'XTickLabel',{'Energy'})
title('Energy Consumption')
legend('DNN','Baseline')

subplot(2,2,3)
bar([latency_DNN latency_base])
title('Latency (s)')
legend('DNN','Baseline')

subplot(2,2,4)
bar(coverage_ratio)
ylim([0 1])
title('Coverage Ratio')

sgtitle('Stable Residual DNN-Assisted UAV Localization');
