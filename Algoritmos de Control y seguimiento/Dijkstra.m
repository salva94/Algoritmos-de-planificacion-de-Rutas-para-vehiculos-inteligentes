% Algoritmo Dijkstra
% Autor：Salva
% Fecha：2021/12/1

clc
clear
close all

%% Definición de gráfico
% Construya una matriz de celdas de nodos de acuerdo con la tabla de nodos vecinos del nodo y la tabla de correspondencia de nodos de letras-valor de nodos
nodes_dist = cell(0);
nodes_dist(1,:) = {1, [2, 6, 7], [12, 16, 14]};
nodes_dist(2,:) = {2, [1, 3, 6], [12, 10, 7]};
nodes_dist(3,:) = {3, [2, 4, 5, 6], [10, 3, 5, 6]};
nodes_dist(4,:) = {4, [3, 5], [3, 4]};
nodes_dist(5,:) = {5, [3, 4, 6, 7], [5, 4, 2, 8]};
nodes_dist(6,:) = {6, [1, 2, 3, 5, 7], [16, 7, 6, 2, 9]};
nodes_dist(7,:) = {7, [1, 5, 6], [14, 8, 9]};

%% Inicialización del algoritmo
% La primera columna representa el número de nodo S/U
% Para S, la segunda columna representa la distancia mínima obtenida desde el nodo fuente a este nodo, y no se cambiará;
% Para U, la segunda columna representa la distancia mínima obtenida temporalmente desde el nodo fuente a este nodo, que puede cambiarse
S = [4, 0];
U(:,1) = [1, 2, 3, 5, 6, 7];
U(:,2) = [inf, inf, 3, 4, inf, inf];

% Inicialización de la ruta óptima y la ruta óptima temporal
path_opt = cell(7,2);
path_opt(4,:) = {4, 4};

path_temp = cell(7,2);
path_temp(3,:) = {3, [4, 3]};
path_temp(4,:) = {4, 4};
path_temp(5,:) = {5, [4, 5]};

%% Recorre todos los nodos
while ~isempty(U)
    
    % Encuentre el valor de distancia mínima actual y el nodo correspondiente en el conjunto U, y elimine el nodo al conjunto S
    [dist_min, idx] = min(U(:,2));
    node_min = U(idx, 1);
    S(end+1,:) = [node_min, dist_min];
    U(idx,:) = [];
    
    % Agregue el nodo con el valor de distancia más pequeño al conjunto de ruta óptimo
    path_opt(node_min,:) = path_temp(node_min,:);
    
    %% Recorra los nodos vecinos del nodo de distancia mínima a su vez para determinar si actualizar el valor de distancia del nodo vecino en el conjunto U
    for i = 1:length(nodes_dist{node_min, 2})
        
        % Nodos que necesitan ser juzgados
        node_temp = nodes_dist{node_min, 2}(i);
        
        % Encuentre el valor de índice del nodo node_temp en el conjunto U
        idx_temp = find(node_temp == U(:,1));
        
        % Determinar si actualizar
        if ~isempty(idx_temp)
            if dist_min + nodes_dist{node_min, 3}(i) < U(idx_temp, 2)
                U(idx_temp, 2) = dist_min + nodes_dist{node_min, 3}(i);
                
                % Actualizar la ruta óptima temporal
                path_temp{node_temp, 1} = node_temp;
                path_temp{node_temp, 2} = [path_opt{node_min, 2}, node_temp];                
            end
        end
    end
end
        
        
        
    
    





