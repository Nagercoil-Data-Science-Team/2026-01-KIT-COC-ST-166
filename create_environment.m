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
end

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

uav_est_final = squeeze(UAV_est_history(:,:,end));

%% =====================================================
% STEP 4: TRAFFIC-AWARE REGION CLUSTERING
% =====================================================
fprintf('\n========================================\n');
fprintf('STEP 4: TRAFFIC-AWARE REGION CLUSTERING\n');
fprintf('========================================\n');

% Use final time step traffic data
final_rho = rho_t(:,end);
final_v = v_t(:,end);
final_q = q_t(:,end);

congestion_level = final_rho ./ max(final_v, 1);
road_importance = final_q / max(final_q);

edge_features = [final_rho, final_v, final_q, congestion_level, road_importance];
edge_features_norm = normalize(edge_features, 'range');

numClusters = 3;
[road_clusters, ~] = kmeans(edge_features_norm, numClusters, ...
    'Distance', 'sqeuclidean', 'Replicates', 10);

cluster_priorities = zeros(numClusters, 1);
for c = 1:numClusters
    cluster_mask = (road_clusters == c);
    cluster_priorities(c) = mean(congestion_level(cluster_mask));
end
[~, priority_order] = sort(cluster_priorities, 'descend');

priority_map = zeros(numClusters, 1);
for c = 1:numClusters
    priority_map(priority_order(c)) = c;
end
road_priority = priority_map(road_clusters);

% Aggregate to nodes
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

node_density = node_density ./ max(node_edge_count, 1);
node_flow = node_flow ./ max(node_edge_count, 1);
node_congestion = node_congestion ./ max(node_edge_count, 1);

node_importance = centrality(cityGraph, 'betweenness');
node_features = [node_density, node_flow, node_congestion, node_importance];
node_features_norm = normalize(node_features, 'range');

numNodeClusters = 3;
[node_clusters, ~] = kmeans(node_features_norm, numNodeClusters, ...
    'Distance', 'sqeuclidean', 'Replicates', 10);

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

fprintf('Total Nodes: %d, Total Roads: %d\n', numNodes, numEdges);
fprintf('Clustering complete with %d node clusters\n', numNodeClusters);

%% =====================================================
% STEP 5: COOPERATIVE PATH PLANNING USING BOA
% =====================================================
fprintf('\n========================================\n');
fprintf('STEP 5: BUTTERFLY OPTIMIZATION ALGORITHM\n');
fprintf('========================================\n\n');

% Define node positions for path planning
node_positions = zeros(numNodes, 2);
cols = ceil(sqrt(numNodes));
for n = 1:numNodes
    row = floor((n-1)/cols);
    col = mod(n-1, cols);
    node_positions(n,:) = [col*numNodes/cols + 1, row*numNodes/cols + 1];
end

% Identify high-priority surveillance points
high_priority_nodes = find(node_priority_level == 1);
med_priority_nodes = find(node_priority_level == 2);
surveillance_points = [high_priority_nodes; med_priority_nodes];

fprintf('--- Path Planning Setup ---\n');
fprintf('Surveillance Points: %d high-priority + %d medium-priority = %d total\n', ...
    length(high_priority_nodes), length(med_priority_nodes), length(surveillance_points));

% BOA Parameters
n_butterflies = 30;
max_iter = 100;
p = 0.8;          % Switch probability
c = 0.01;         % Sensory modality
a = 0.1;          % Power exponent

% Path representation: sequence of waypoints
n_waypoints = min(8, length(surveillance_points));
fprintf('Planning paths with %d waypoints per UAV\n', n_waypoints);

% Initialize butterfly population for each UAV
UAV_paths = cell(numUAVs, 1);
UAV_best_fitness = zeros(numUAVs, 1);
UAV_fitness_history = zeros(numUAVs, max_iter);

fprintf('\n--- Starting BOA Optimization ---\n');

for uav = 1:numUAVs
    fprintf('Optimizing path for UAV %d...\n', uav);
    
    % Start and end positions for this UAV
    start_pos = uav_est_final(uav, 1:2);
    
    % Initialize butterflies (candidate paths)
    butterflies = zeros(n_butterflies, n_waypoints);
    fitness = zeros(n_butterflies, 1);
    
    % Random initialization
    for i = 1:n_butterflies
        butterflies(i,:) = surveillance_points(randperm(length(surveillance_points), n_waypoints));
    end
    
    % Evaluate initial fitness
    for i = 1:n_butterflies
        fitness(i) = evaluate_path_fitness(butterflies(i,:), start_pos, ...
            node_positions, node_priority_level, uav_est_final, uav);
    end
    
    [best_fitness, best_idx] = min(fitness);
    best_butterfly = butterflies(best_idx, :);
    
    % BOA main loop
    for iter = 1:max_iter
        for i = 1:n_butterflies
            % Calculate fragrance
            f_i = c * fitness(i)^a;
            
            if rand < p
                % Global search phase
                r = randi(n_butterflies);
                for j = 1:n_waypoints
                    if rand < 0.3
                        % Move towards best butterfly
                        candidates = surveillance_points;
                        candidates(candidates == butterflies(i,j)) = [];
                        if ~isempty(candidates)
                            butterflies(i,j) = candidates(randi(length(candidates)));
                        end
                    end
                end
            else
                % Local search phase
                r1 = randi(n_butterflies);
                r2 = randi(n_butterflies);
                for j = 1:n_waypoints
                    if rand < 0.3
                        % Random walk in solution space
                        candidates = surveillance_points;
                        butterflies(i,j) = candidates(randi(length(candidates)));
                    end
                end
            end
            
            % Ensure no duplicate waypoints
            butterflies(i,:) = unique_waypoints(butterflies(i,:), surveillance_points, n_waypoints);
            
            % Evaluate new fitness
            new_fitness = evaluate_path_fitness(butterflies(i,:), start_pos, ...
                node_positions, node_priority_level, uav_est_final, uav);
            
            if new_fitness < fitness(i)
                fitness(i) = new_fitness;
                
                if new_fitness < best_fitness
                    best_fitness = new_fitness;
                    best_butterfly = butterflies(i,:);
                end
            end
        end
        
        UAV_fitness_history(uav, iter) = best_fitness;
    end
    
    % Store optimized path
    UAV_paths{uav} = best_butterfly;
    UAV_best_fitness(uav) = best_fitness;
    
    fprintf('  Best fitness: %.4f\n', best_fitness);
end

fprintf('\n--- BOA Optimization Complete ---\n');
fprintf('All UAV paths optimized successfully\n\n');

% Display path summary
fprintf('--- Path Planning Summary ---\n');
for uav = 1:numUAVs
    waypoint_nodes = UAV_paths{uav};
    fprintf('UAV %d: %d waypoints, Fitness: %.4f\n', uav, length(waypoint_nodes), UAV_best_fitness(uav));
    fprintf('  Route: ');
    for w = 1:length(waypoint_nodes)
        fprintf('%d ', waypoint_nodes(w));
    end
    fprintf('\n');
end

%% =====================================================
% VISUALIZATION: STEP 5 - BOA PATH PLANNING
% =====================================================

figure('Name', 'Step 5: BOA Path Planning Results', 'Position', [100, 100, 1400, 900]);

% Subplot 1: Optimized UAV Paths on City Network
subplot(2,3,1);
hold on; grid on;
plot(cityGraph, 'XData', node_positions(:,1), 'YData', node_positions(:,2), ...
     'EdgeColor', [0.8 0.8 0.8], 'LineWidth', 1, 'NodeColor', [0.9 0.9 0.9]);

% Color nodes by priority
for i = 1:numNodes
    if node_priority_level(i) == 1
        plot(node_positions(i,1), node_positions(i,2), 'ro', 'MarkerSize', 12, ...
            'MarkerFaceColor', 'r', 'LineWidth', 2);
    elseif node_priority_level(i) == 2
        plot(node_positions(i,1), node_positions(i,2), 'ys', 'MarkerSize', 10, ...
            'MarkerFaceColor', 'y', 'LineWidth', 1.5);
    else
        plot(node_positions(i,1), node_positions(i,2), 'bo', 'MarkerSize', 8, ...
            'MarkerFaceColor', 'b', 'LineWidth', 1);
    end
end

% Plot UAV paths
colors = lines(numUAVs);
for uav = 1:numUAVs
    path = UAV_paths{uav};
    full_path = [uav_est_final(uav,1:2); node_positions(path,:)];
    plot(full_path(:,1), full_path(:,2), '-o', 'Color', colors(uav,:), ...
        'LineWidth', 2.5, 'MarkerSize', 8, 'MarkerFaceColor', colors(uav,:), ...
        'DisplayName', sprintf('UAV %d', uav));
end

xlabel('X (km)'); ylabel('Y (km)');
title('Optimized UAV Surveillance Paths');
legend('Location', 'best');
xlim([0 numNodes]); ylim([0 numNodes]);
axis equal;

% Subplot 2: Fitness Convergence
subplot(2,3,2);
hold on; grid on;
for uav = 1:numUAVs
    plot(1:max_iter, UAV_fitness_history(uav,:), 'LineWidth', 2, ...
        'DisplayName', sprintf('UAV %d', uav));
end
xlabel('Iteration');
ylabel('Fitness Value');
title('BOA Convergence - All UAVs');
legend('Location', 'best');

% Subplot 3: 3D Visualization of UAV Paths
subplot(2,3,3);
hold on; grid on;
% Plot city network at ground level
for e = 1:numEdges
    endpoints = cityGraph.Edges.EndNodes(e,:);
    plot3([node_positions(endpoints(1),1), node_positions(endpoints(2),1)], ...
          [node_positions(endpoints(1),2), node_positions(endpoints(2),2)], ...
          [0, 0], 'Color', [0.8 0.8 0.8], 'LineWidth', 0.5);
end

% Plot surveillance points
scatter3(node_positions(high_priority_nodes,1), node_positions(high_priority_nodes,2), ...
    zeros(length(high_priority_nodes),1), 100, 'r', 'filled', '^');
scatter3(node_positions(med_priority_nodes,1), node_positions(med_priority_nodes,2), ...
    zeros(length(med_priority_nodes),1), 80, 'y', 'filled', 's');

% Plot 3D UAV paths
for uav = 1:numUAVs
    path = UAV_paths{uav};
    full_path_3d = [uav_est_final(uav,:); [node_positions(path,:), ...
        30*ones(length(path),1)]];
    plot3(full_path_3d(:,1), full_path_3d(:,2), full_path_3d(:,3), '-o', ...
        'Color', colors(uav,:), 'LineWidth', 2, 'MarkerSize', 6, ...
        'MarkerFaceColor', colors(uav,:));
end

xlabel('X (km)'); ylabel('Y (km)'); zlabel('Altitude (m)');
title('3D UAV Surveillance Trajectories');
view(45, 30);

% Subplot 4: Path Length Distribution
subplot(2,3,4);
path_lengths = zeros(numUAVs, 1);
for uav = 1:numUAVs
    path = UAV_paths{uav};
    full_path = [uav_est_final(uav,1:2); node_positions(path,:)];
    total_dist = 0;
    for i = 1:size(full_path,1)-1
        total_dist = total_dist + norm(full_path(i+1,:) - full_path(i,:));
    end
    path_lengths(uav) = total_dist;
end

bar(1:numUAVs, path_lengths, 'FaceColor', [0.2 0.6 0.8]);
xlabel('UAV ID');
ylabel('Path Length (km)');
title('Total Path Length per UAV');
grid on;

% Subplot 5: Coverage Map
subplot(2,3,5);
coverage_map = zeros(numNodes, 1);
for uav = 1:numUAVs
    path = UAV_paths{uav};
    for node = path
        coverage_map(node) = coverage_map(node) + 1;
    end
end

bar(1:numNodes, coverage_map, 'FaceColor', [0.4 0.7 0.3]);
xlabel('Node ID');
ylabel('Visit Count');
title('Surveillance Coverage per Node');
grid on;

% Subplot 6: Priority vs Coverage Analysis
subplot(2,3,6);
priority_coverage = zeros(3, 1);
priority_counts = zeros(3, 1);
for i = 1:numNodes
    priority_coverage(node_priority_level(i)) = priority_coverage(node_priority_level(i)) + coverage_map(i);
    priority_counts(node_priority_level(i)) = priority_counts(node_priority_level(i)) + 1;
end

avg_coverage = priority_coverage ./ max(priority_counts, 1);
bar(1:3, avg_coverage, 'FaceColor', [0.8 0.4 0.2]);
set(gca, 'XTickLabel', {'High', 'Medium', 'Low'});
xlabel('Priority Level');
ylabel('Average Coverage');
title('Coverage by Priority Level');
grid on;

% Print optimization metrics
fprintf('\n--- Path Planning Metrics ---\n');
fprintf('Average path length: %.2f km\n', mean(path_lengths));
fprintf('Total coverage points: %d\n', sum(coverage_map > 0));
fprintf('High-priority coverage: %.2f%%\n', ...
    100*sum(coverage_map(high_priority_nodes) > 0)/length(high_priority_nodes));
fprintf('Medium-priority coverage: %.2f%%\n', ...
    100*sum(coverage_map(med_priority_nodes) > 0)/length(med_priority_nodes));

fprintf('\n========================================\n');
fprintf('STEP 5 COMPLETED SUCCESSFULLY\n');
fprintf('========================================\n\n');

%% =====================================================
% HELPER FUNCTIONS
% =====================================================

function fitness = evaluate_path_fitness(path, start_pos, node_pos, ...
    node_priority, all_uav_pos, current_uav)
    % Fitness components:
    % 1. Total path distance (minimize)
    % 2. Priority coverage (maximize high priority)
    % 3. Collision avoidance (penalize close proximity to other UAVs)
    % 4. Energy efficiency (penalize sharp turns)
    
    % Distance cost
    full_path = [start_pos; node_pos(path,:)];
    total_dist = 0;
    for i = 1:size(full_path,1)-1
        total_dist = total_dist + norm(full_path(i+1,:) - full_path(i,:));
    end
    
    % Priority cost (negative because we want high priority)
    priority_score = 0;
    for node = path
        if node_priority(node) == 1
            priority_score = priority_score - 10;
        elseif node_priority(node) == 2
            priority_score = priority_score - 5;
        else
            priority_score = priority_score - 1;
        end
    end
    
    % Collision avoidance cost
    collision_cost = 0;
    min_separation = 2.0; % km
    for i = 1:size(all_uav_pos,1)
        if i ~= current_uav
            for waypoint = path
                dist = norm(all_uav_pos(i,1:2) - node_pos(waypoint,:));
                if dist < min_separation
                    collision_cost = collision_cost + 50 * (min_separation - dist);
                end
            end
        end
    end
    
    % Turn penalty (energy efficiency)
    turn_cost = 0;
    if length(path) > 2
        for i = 2:length(path)-1
            v1 = node_pos(path(i),:) - node_pos(path(i-1),:);
            v2 = node_pos(path(i+1),:) - node_pos(path(i),:);
            angle = acos(dot(v1,v2)/(norm(v1)*norm(v2)+1e-6));
            turn_cost = turn_cost + abs(angle);
        end
    end
    
    % Combined fitness (lower is better)
    fitness = 2*total_dist - priority_score + collision_cost + 5*turn_cost;
end

function unique_path = unique_waypoints(path, available_points, n_waypoints)
    % Ensure path has unique waypoints
    unique_path = unique(path, 'stable');
    
    % If we lost waypoints due to duplicates, add random ones
    while length(unique_path) < n_waypoints
        candidates = available_points;
        candidates(ismember(candidates, unique_path)) = [];
        if ~isempty(candidates)
            unique_path = [unique_path, candidates(randi(length(candidates)))];
        else
            break;
        end
    end
    
    % Trim if too many
    unique_path = unique_path(1:min(n_waypoints, length(unique_path)));
end