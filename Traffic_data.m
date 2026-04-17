clc;
clear;
close all;

%% =====================================================
% STEP 1: SMART CITY TRAFFIC ENVIRONMENT MODELING
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

% Initial traffic parameters
rho = 20 + 80 * rand(numEdges,1);
v   = 20 + 60 * rand(numEdges,1);
q   = rho .* v;

cityGraph.Edges.Density = rho;
cityGraph.Edges.Speed   = v;
cityGraph.Edges.Flow    = q;

numRegions = 3;
regions = randi(numRegions, numNodes, 1);

% Plot city traffic graph
figure('Name', 'Step 1: City Network');
p1 = plot(cityGraph,'Layout','force');
title('Smart City Traffic Network');
p1.NodeCData = regions;
labelnode(p1,1:numNodes,string(1:numNodes));
colorbar;
colormap(jet);

%% =====================================================
% STEP 2: IoT-BASED REAL-TIME TRAFFIC DATA GENERATION
% =====================================================
T = 40;
sensorNoise = 0.05;
numUAVs_step2 = 3;
UAV_positions = rand(numUAVs_step2,2) * numNodes;

rho_t = zeros(numEdges,T);
v_t   = zeros(numEdges,T);
q_t   = zeros(numEdges,T);

% Create figure for real-time monitoring
figure('Name', 'Step 2: Real-Time Monitoring');
colormap(hot);

for t = 1:T
    rho = rho + sensorNoise * randn(numEdges,1);
    v   = v   + sensorNoise * randn(numEdges,1);
    rho = max(5, min(rho, 120));
    v   = max(10, min(v, 90));
    q = rho .* v;
    
    rho_t(:,t) = rho;
    v_t(:,t)   = v;
    q_t(:,t)   = q;
    
    UAV_Data{t}.Density = rho;
    UAV_Data{t}.Speed   = v;
    UAV_Data{t}.Flow    = q;
    
    % Update graph with current traffic data
    cityGraph.Edges.Density = rho;
    
    % Recreate plot with updated data - FIXED VERSION
    clf;
    p_rt = plot(cityGraph,'Layout','force');
    title(['Real-Time IoT-Assisted Traffic Monitoring - Time Step: ' num2str(t)]);
    labelnode(p_rt,1:numNodes,string(1:numNodes));
    
    % Set default edge color first (this clears any EdgeCData)
    p_rt.EdgeColor = [0.5 0.5 0.5];
    
    % Get colormap for edge coloring
    cmap = colormap(hot);
    rho_norm = rescale(rho, 1, size(cmap,1));
    
    % Set line widths based on density
    lineWidths = rescale(rho, 1, 8);
    
    % Now highlight each edge with its specific color and width
    for edge = 1:numEdges
        % Get color from colormap based on density
        color_idx = round(rho_norm(edge));
        edge_color = cmap(color_idx, :);
        
        highlight(p_rt, cityGraph.Edges.EndNodes(edge,1), ...
                  cityGraph.Edges.EndNodes(edge,2), ...
                  'LineWidth', lineWidths(edge), ...
                  'EdgeColor', edge_color);
    end
    
    colorbar;
    drawnow;
    pause(0.05);
end

figure('Name', 'Step 2: Traffic Evolution');
plot(mean(rho_t,1),'LineWidth',2); hold on;
plot(mean(v_t,1),'LineWidth',2);
plot(mean(q_t,1),'LineWidth',2);
grid on;
xlabel('Time Step');
ylabel('Average Value');
legend('Density \rho','Speed v','Flow q');
title('IoT-Based Traffic Evolution Over Time');

%% =====================================================
% STEP 3: UAV SWARM DEPLOYMENT AND COOPERATIVE LOCALIZATION
% =====================================================
numUAVs = 5;
citySize = [numNodes, numNodes, 100];

UAV_true_pos = [rand(numUAVs,2) * numNodes, 20 + 60*rand(numUAVs,1)];
GPS_noise_std = 2.0;
UAV_est_pos = UAV_true_pos + GPS_noise_std * randn(numUAVs, 3);

numAnchors = 6;
anchor_pos = [rand(numAnchors,2) * numNodes, zeros(numAnchors,1)];

U2U_range = 150;
U2I_range = 200;

dt = 1.0;
filters = cell(numUAVs, 1);
for i = 1:numUAVs
    initialState = [UAV_est_pos(i,:), zeros(1,3)]';
    A = [1 0 0 dt 0  0;
         0 1 0 0  dt 0;
         0 0 1 0  0  dt;
         0 0 0 1  0  0;
         0 0 0 0  1  0;
         0 0 0 0  0  1];
    H = [1 0 0 0 0 0;
         0 1 0 0 0 0;
         0 0 1 0 0 0];
    Q = 0.1 * eye(6);
    R = 1.0 * eye(3);
    P0 = 10 * eye(6);
    filters{i} = struct('x', initialState, 'P', P0, 'A', A, 'H', H, 'Q', Q, 'R', R);
end

numIterations = 50;
UAV_est_history = zeros(numUAVs, 3, numIterations);
position_error = zeros(numUAVs, numIterations);
gps_only_error = zeros(numUAVs, 1);

for iter = 1:numIterations
    if iter > 1
        UAV_true_pos = UAV_true_pos + 0.2 * randn(numUAVs, 3);
        UAV_true_pos(:,1:2) = max(0, min(UAV_true_pos(:,1:2), numNodes));
        UAV_true_pos(:,3) = max(20, min(UAV_true_pos(:,3), 80));
    end
    
    for i = 1:numUAVs
        filters{i}.x = filters{i}.A * filters{i}.x;
        filters{i}.P = filters{i}.A * filters{i}.P * filters{i}.A' + filters{i}.Q;
        
        measurements = [];
        measurement_count = 0;
        
        gps_meas = UAV_true_pos(i,:)' + GPS_noise_std * randn(3,1);
        measurements = [measurements; gps_meas];
        measurement_count = measurement_count + 1;
        
        if iter == 1
            gps_only_error(i) = norm(gps_meas - UAV_true_pos(i,:)');
        end
        
        for a = 1:numAnchors
            dist = norm(UAV_true_pos(i,1:2) - anchor_pos(a,1:2));
            if dist <= U2I_range
                range_noise = 0.5;
                measured_dist = dist + range_noise * randn();
                direction = (anchor_pos(a,1:2) - filters{i}.x(1:2)');
                if norm(direction) > 0
                    direction = direction / norm(direction);
                    correction = direction * (measured_dist - norm(filters{i}.x(1:2)' - anchor_pos(a,1:2)));
                    filters{i}.x(1:2) = filters{i}.x(1:2) + 0.1 * correction';
                end
            end
        end
        
        for j = 1:numUAVs
            if i ~= j
                dist = norm(UAV_true_pos(i,:) - UAV_true_pos(j,:));
                if dist <= U2U_range
                    range_noise = 0.3;
                    measured_dist = dist + range_noise * randn();
                    direction = (filters{j}.x(1:3) - filters{i}.x(1:3));
                    if norm(direction) > 0
                        direction = direction / norm(direction);
                        correction = direction * (measured_dist - norm(filters{i}.x(1:3) - filters{j}.x(1:3)));
                        filters{i}.x(1:3) = filters{i}.x(1:3) + 0.05 * correction;
                    end
                end
            end
        end
        
        z = gps_meas;
        y = z - filters{i}.H * filters{i}.x;
        S = filters{i}.H * filters{i}.P * filters{i}.H' + filters{i}.R;
        K = filters{i}.P * filters{i}.H' / S;
        filters{i}.x = filters{i}.x + K * y;
        filters{i}.P = (eye(6) - K * filters{i}.H) * filters{i}.P;
        
        UAV_est_history(i,:,iter) = filters{i}.x(1:3)';
        position_error(i, iter) = norm(filters{i}.x(1:3) - UAV_true_pos(i,:)');
    end
end

figure('Name', 'Step 3: UAV Cooperative Localization', 'Position', [100, 100, 1200, 800]);

subplot(2,2,1);
hold on; grid on;
scatter3(anchor_pos(:,1), anchor_pos(:,2), anchor_pos(:,3), 200, 'k', 'filled', '^', ...
    'DisplayName', 'IoT Anchors');
scatter3(UAV_true_pos(:,1), UAV_true_pos(:,2), UAV_true_pos(:,3), 150, 'g', 'filled', ...
    'DisplayName', 'True UAV Pos');
uav_est_final = squeeze(UAV_est_history(:,:,end));
scatter3(uav_est_final(:,1), uav_est_final(:,2), uav_est_final(:,3), 150, 'r', 'o', ...
    'LineWidth', 2, 'DisplayName', 'Estimated UAV Pos');
for i = 1:numUAVs
    for j = i+1:numUAVs
        dist = norm(UAV_true_pos(i,:) - UAV_true_pos(j,:));
        if dist <= U2U_range
            plot3([UAV_true_pos(i,1), UAV_true_pos(j,1)], ...
                  [UAV_true_pos(i,2), UAV_true_pos(j,2)], ...
                  [UAV_true_pos(i,3), UAV_true_pos(j,3)], 'b--', 'LineWidth', 0.5);
        end
    end
end
xlabel('X (km)'); ylabel('Y (km)'); zlabel('Altitude (m)');
title('UAV Swarm 3D Deployment');
legend('Location', 'best');
view(45, 30);

subplot(2,2,2);
plot(1:numIterations, position_error', 'LineWidth', 1.5);
hold on;
plot(1:numIterations, mean(position_error,1), 'k', 'LineWidth', 3, 'DisplayName', 'Average');
grid on;
xlabel('Iteration');
ylabel('Position Error (m)');
title('Cooperative Localization Convergence');
legend(['UAV ' + string(1:numUAVs), 'Average'], 'Location', 'best');

subplot(2,2,3);
hold on; grid on;
node_positions = zeros(numNodes, 2);
cols = ceil(sqrt(numNodes));
for n = 1:numNodes
    row = floor((n-1)/cols);
    col = mod(n-1, cols);
    node_positions(n,:) = [col*numNodes/cols + 1, row*numNodes/cols + 1];
end
p_city = plot(cityGraph, 'XData', node_positions(:,1), 'YData', node_positions(:,2), ...
     'EdgeColor', [0.7 0.7 0.7], 'NodeColor', [0.8 0.8 0.8], 'LineWidth', 0.5, ...
     'MarkerSize', 6);
scatter(anchor_pos(:,1), anchor_pos(:,2), 200, 'k', 'filled', '^', ...
    'DisplayName', 'Anchors');
scatter(UAV_true_pos(:,1), UAV_true_pos(:,2), 150, 'g', 'filled', ...
    'DisplayName', 'True Pos');
scatter(uav_est_final(:,1), uav_est_final(:,2), 150, 'r', 'o', 'LineWidth', 2, ...
    'DisplayName', 'Estimated Pos');
for i = 1:numUAVs
    quiver(UAV_true_pos(i,1), UAV_true_pos(i,2), ...
           uav_est_final(i,1)-UAV_true_pos(i,1), ...
           uav_est_final(i,2)-UAV_true_pos(i,2), ...
           0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
end
xlabel('X (km)'); ylabel('Y (km)');
title('Top View: UAV Positions over City Network');
legend('Location', 'best');
xlim([0 numNodes]); ylim([0 numNodes]);
axis equal;

subplot(2,2,4);
comm_adj = zeros(numUAVs);
for i = 1:numUAVs
    for j = i+1:numUAVs
        dist = norm(UAV_true_pos(i,:) - UAV_true_pos(j,:));
        if dist <= U2U_range
            comm_adj(i,j) = 1;
            comm_adj(j,i) = 1;
        end
    end
end
comm_graph = graph(comm_adj);
p_comm = plot(comm_graph, 'Layout', 'force', 'NodeColor', 'b', 'MarkerSize', 8);
title('UAV-to-UAV Communication Topology');
labelnode(p_comm, 1:numUAVs, "UAV" + string(1:numUAVs));

%% =====================================================
% STEP 4: TRAFFIC-AWARE REGION CLUSTERING
% =====================================================
fprintf('\n\n========================================\n');
fprintf('STEP 4: TRAFFIC-AWARE REGION CLUSTERING\n');
fprintf('========================================\n\n');

% Extract traffic features for each road (edge)
fprintf('--- Extracting Traffic Features ---\n');

% Use final time step traffic data
final_rho = rho_t(:,end);
final_v = v_t(:,end);
final_q = q_t(:,end);

% Compute congestion level (inverse of speed, normalized by density)
congestion_level = final_rho ./ max(final_v, 1);

% Compute road importance (based on flow)
road_importance = final_q / max(final_q);

% Create feature matrix for edges
edge_features = [final_rho, final_v, final_q, congestion_level, road_importance];

% Normalize features for clustering
edge_features_norm = normalize(edge_features, 'range');

fprintf('Feature extraction complete.\n');
fprintf('Features: Density, Speed, Flow, Congestion Level, Road Importance\n\n');

% Display edge features
edge_table = table((1:numEdges)', final_rho, final_v, final_q, congestion_level, road_importance, ...
    'VariableNames', {'Road_ID', 'Density', 'Speed', 'Flow', 'Congestion', 'Importance'});
disp(edge_table);

% Apply K-Means clustering to roads
fprintf('\n--- Applying K-Means Clustering ---\n');
numClusters = 3; % High, Medium, Low traffic priority
[road_clusters, road_centroids] = kmeans(edge_features_norm, numClusters, ...
    'Distance', 'sqeuclidean', 'Replicates', 10);

fprintf('K-Means clustering complete with %d clusters.\n\n', numClusters);

% Assign priority levels based on average congestion in each cluster
cluster_priorities = zeros(numClusters, 1);
for c = 1:numClusters
    cluster_mask = (road_clusters == c);
    cluster_priorities(c) = mean(congestion_level(cluster_mask));
end
[~, priority_order] = sort(cluster_priorities, 'descend');

% Map clusters to priority levels (1=High, 2=Medium, 3=Low)
priority_map = zeros(numClusters, 1);
for c = 1:numClusters
    priority_map(priority_order(c)) = c;
end
road_priority = priority_map(road_clusters);

fprintf('--- Road Cluster Assignment ---\n');
cluster_table = table((1:numEdges)', road_clusters, road_priority, ...
    'VariableNames', {'Road_ID', 'Cluster', 'Priority_Level'});
disp(cluster_table);

% Aggregate traffic features to nodes (intersections)
fprintf('\n--- Aggregating Features to Nodes ---\n');
node_density = zeros(numNodes, 1);
node_flow = zeros(numNodes, 1);
node_congestion = zeros(numNodes, 1);
node_edge_count = zeros(numNodes, 1);

for e = 1:numEdges
    endpoints = cityGraph.Edges.EndNodes(e,:);
    for node = endpoints
        node_density(node) = node_density(node) + final_rho(e);
        node_flow(node) = node_flow(node) + final_q(e);
        node_congestion(node) = node_congestion(node) + congestion_level(e);
        node_edge_count(node) = node_edge_count(node) + 1;
    end
end

% Average values
node_density = node_density ./ max(node_edge_count, 1);
node_flow = node_flow ./ max(node_edge_count, 1);
node_congestion = node_congestion ./ max(node_edge_count, 1);

% Node importance (betweenness centrality)
node_importance = centrality(cityGraph, 'betweenness');

% Create node feature matrix
node_features = [node_density, node_flow, node_congestion, node_importance];
node_features_norm = normalize(node_features, 'range');

fprintf('Node feature aggregation complete.\n\n');

% Display node features
node_table = table((1:numNodes)', node_density, node_flow, node_congestion, node_importance, ...
    'VariableNames', {'Node_ID', 'Avg_Density', 'Avg_Flow', 'Avg_Congestion', 'Importance'});
disp(node_table);

% Apply K-Means clustering to nodes
fprintf('\n--- Clustering Nodes into Surveillance Regions ---\n');
numNodeClusters = 3;
[node_clusters, node_centroids] = kmeans(node_features_norm, numNodeClusters, ...
    'Distance', 'sqeuclidean', 'Replicates', 10);

% Assign priority to node clusters
node_cluster_priorities = zeros(numNodeClusters, 1);
for c = 1:numNodeClusters
    cluster_mask = (node_clusters == c);
    node_cluster_priorities(c) = mean(node_congestion(cluster_mask));
end
[~, node_priority_order] = sort(node_cluster_priorities, 'descend');

node_priority_map = zeros(numNodeClusters, 1);
for c = 1:numNodeClusters
    node_priority_map(node_priority_order(c)) = c;
end
node_priority_level = node_priority_map(node_clusters);

fprintf('Node clustering complete.\n\n');

fprintf('--- Node Cluster Assignment ---\n');
node_cluster_table = table((1:numNodes)', node_clusters, node_priority_level, ...
    'VariableNames', {'Node_ID', 'Cluster', 'Priority_Level'});
disp(node_cluster_table);

% UAV assignment based on cluster priority
fprintf('\n--- UAV Assignment to Clusters ---\n');
UAVs_per_priority = [3, 2, 1]; % High, Medium, Low priority
uav_assignment = cell(numNodeClusters, 1);

uav_id = 1;
for priority = 1:3
    nodes_in_priority = find(node_priority_level == priority);
    num_uavs_assigned = min(UAVs_per_priority(priority), numUAVs - uav_id + 1);
    
    if num_uavs_assigned > 0
        uav_assignment{priority} = uav_id:(uav_id + num_uavs_assigned - 1);
        uav_id = uav_id + num_uavs_assigned;
    else
        uav_assignment{priority} = [];
    end
end

fprintf('UAV-to-Cluster Assignment:\n');
for priority = 1:3
    fprintf('  Priority %d (Cluster %d): UAVs [%s]\n', ...
        priority, priority, num2str(uav_assignment{priority}));
end

% Create UAV assignment map
uav_cluster_assignment = zeros(numUAVs, 1);
for priority = 1:3
    if ~isempty(uav_assignment{priority})
        uav_cluster_assignment(uav_assignment{priority}) = priority;
    end
end

fprintf('\n--- Individual UAV Assignments ---\n');
uav_assign_table = table((1:numUAVs)', uav_cluster_assignment, ...
    'VariableNames', {'UAV_ID', 'Assigned_Cluster_Priority'});
disp(uav_assign_table);

% Summary statistics
fprintf('\n--- Clustering Summary Statistics ---\n');
fprintf('Total Nodes: %d\n', numNodes);
fprintf('Total Roads: %d\n', numEdges);
fprintf('Number of Node Clusters: %d\n', numNodeClusters);
fprintf('Number of Road Clusters: %d\n', numClusters);
fprintf('\nCluster Distribution (Nodes):\n');
for c = 1:numNodeClusters
    priority = node_priority_map(c);
    num_nodes = sum(node_clusters == c);
    fprintf('  Cluster %d (Priority %d): %d nodes\n', c, priority, num_nodes);
end
fprintf('\nCluster Distribution (Roads):\n');
for c = 1:numClusters
    priority = priority_map(c);
    num_roads = sum(road_clusters == c);
    fprintf('  Cluster %d (Priority %d): %d roads\n', c, priority, num_roads);
end

%% =====================================================
% VISUALIZATION: TRAFFIC-AWARE CLUSTERING
% =====================================================

figure('Name', 'Step 4: Traffic-Aware Clustering', 'Position', [100, 100, 1400, 900]);

% Subplot 1: Road clusters
subplot(2,3,1);
p_roads = plot(cityGraph, 'Layout', 'force', 'LineWidth', 2);
p_roads.EdgeCData = road_priority;
p_roads.NodeColor = [0.7 0.7 0.7];
colormap(gca, jet);
colorbar;
caxis([1 3]);
title('Road Clustering by Traffic Priority');
labelnode(p_roads, 1:numNodes, string(1:numNodes));

% Subplot 2: Node clusters
subplot(2,3,2);
p_nodes = plot(cityGraph, 'Layout', 'force');
p_nodes.NodeCData = node_priority_level;
p_nodes.MarkerSize = 10;
p_nodes.EdgeColor = [0.5 0.5 0.5];
colormap(gca, jet);
colorbar;
caxis([1 3]);
title('Node Clustering by Traffic Priority');
labelnode(p_nodes, 1:numNodes, string(1:numNodes));

% Subplot 3: Combined view with UAV assignment
subplot(2,3,3);
p_combined = plot(cityGraph, 'Layout', 'force', 'LineWidth', 2);
p_combined.NodeCData = node_priority_level;
p_combined.EdgeCData = road_priority;
p_combined.MarkerSize = 10;
colormap(gca, jet);
colorbar;
caxis([1 3]);
title('Combined Traffic-Aware Clustering');
labelnode(p_combined, 1:numNodes, string(1:numNodes));

% Subplot 4: Feature space visualization (PCA)
subplot(2,3,4);
[coeff, score, ~] = pca(edge_features_norm);
gscatter(score(:,1), score(:,2), road_priority, jet(3), 'o', 8, 'filled');
grid on;
xlabel('First Principal Component');
ylabel('Second Principal Component');
title('Road Clustering in Feature Space (PCA)');
legend('Priority 1 (High)', 'Priority 2 (Med)', 'Priority 3 (Low)', 'Location', 'best');

% Subplot 5: Node feature space (PCA)
subplot(2,3,5);
[coeff_node, score_node, ~] = pca(node_features_norm);
gscatter(score_node(:,1), score_node(:,2), node_priority_level, jet(3), 'o', 10, 'filled');
grid on;
xlabel('First Principal Component');
ylabel('Second Principal Component');
title('Node Clustering in Feature Space (PCA)');
legend('Priority 1 (High)', 'Priority 2 (Med)', 'Priority 3 (Low)', 'Location', 'best');

% Subplot 6: UAV deployment map
subplot(2,3,6);
hold on; grid on;

% Plot city network
p_uav_deploy = plot(cityGraph, 'XData', node_positions(:,1), 'YData', node_positions(:,2), ...
     'EdgeColor', [0.7 0.7 0.7], 'LineWidth', 1.5);

% Color nodes by priority
p_uav_deploy.NodeCData = node_priority_level;
p_uav_deploy.MarkerSize = 12;

% Overlay UAVs on high-priority regions
high_priority_nodes = find(node_priority_level == 1);
med_priority_nodes = find(node_priority_level == 2);
low_priority_nodes = find(node_priority_level == 3);

if ~isempty(high_priority_nodes)
    scatter(node_positions(high_priority_nodes,1), node_positions(high_priority_nodes,2), ...
        250, 'r', 'filled', '^', 'DisplayName', 'High Priority + UAVs');
end
if ~isempty(med_priority_nodes)
    scatter(node_positions(med_priority_nodes,1), node_positions(med_priority_nodes,2), ...
        200, 'y', 'filled', 's', 'DisplayName', 'Medium Priority');
end
if ~isempty(low_priority_nodes)
    scatter(node_positions(low_priority_nodes,1), node_positions(low_priority_nodes,2), ...
        150, 'b', 'filled', 'o', 'DisplayName', 'Low Priority');
end

colormap(gca, jet);
colorbar;
title('UAV Deployment by Traffic Priority');
xlabel('X (km)'); ylabel('Y (km)');
legend('Location', 'best');
xlim([0 numNodes]); ylim([0 numNodes]);
axis equal;

fprintf('\n========================================\n');
fprintf('STEP 4 COMPLETED SUCCESSFULLY\n');
fprintf('========================================\n\n');