% Algoritmo ACA
% AUTOR:SALVA
% FECHA: 2021/12/1

clc
clear
close all


%% Parametros de inicializacion
% Construya una matriz de celdas de nodos de acuerdo con la tabla de nodos adyacentes del nodo y la tabla de correspondencia de nodos de letras-n迆mero de nodos
nodes_data = cell(0);
nodes_data(1,:) = {1, [2, 6, 7], [12, 16, 14]};
nodes_data(2,:) = {2, [1, 3, 6], [12, 10, 7]};
nodes_data(3,:) = {3, [2, 4, 5, 6], [10, 3, 5, 6]};
nodes_data(4,:) = {4, [3, 5], [3, 4]};
nodes_data(5,:) = {5, [3, 4, 6, 7], [5, 4, 2, 8]};
nodes_data(6,:) = {6, [1, 2, 3, 5, 7], [16, 7, 6, 2, 9]};
nodes_data(7,:) = {7, [1, 5, 6], [14, 8, 9]};

% Nodos iniciales y finales
node_start = 4;                       % Nodo de origen inicial
node_end = 1;                         % Nodo final

% Definir los parametros  de la colonia de hormigas
m = 50;                              % Numero de hormigas
n = size(nodes_data,1);              % Numero de nodos
alpha = 1;                           % Factor de importancia de las feromonas
beta = 5;                            % Factor de importancia de la funcion heuristica
rho = 0.1;                           % Factor de volatilizacion de feromonas
Q = 1;                               % constante

% En el proceso iterativo, los par芍metros relevantes se inicializan y definen
iter = 1;                            % Valor inicial del numero de iteracion
iter_max = 100;                      % El n迆mero maximo de iteraciones 
Route_best = cell(iter_max,1);       % El mejor camino para cada generacion       
Length_best = zeros(iter_max,1);     % La longitud del mejor camino para cada generacion.  
Length_ave = zeros(iter_max,1);      % La longitud promedio de cada ruta de generacion.

% Ponga feromona y factor volatil en nodes_data juntos
Delta_Tau_initial = nodes_data(:,1:2);
for i = 1:size(nodes_data,1)
    nodes_data{i,4} = ones(1, length(nodes_data{i,3}));   % Feromonas
    nodes_data{i,5} = 1./nodes_data{i,3};                 % Factor volatiidad
    Delta_Tau_initial{i,3} = zeros(1, length(nodes_data{i,3}));
end


%% Encuentra iterativamente el mejor camino
while iter <= iter_max  
  
    route = cell(0);
    
    %%  Selecci車n de ruta por hormiga
    for i = 1:m
        % Selecci車n de ruta nodo por nodo
        neighbor = cell(0);
        node_step = node_start;
        path = node_step;
        dist = 0;
        while ~ismember(node_end, path) %Cuando el nodo final se incluye en la tabla de ruta, la hormiga completa la optimizaci車n de la ruta y salta fuera del ciclo.
           
            % Encuentra nodos vecinos           
            neighbor = nodes_data{node_step,2};
            
            % Eliminar los nodos vecinos que se han visitado
            idx = [];
            for k = 1:length(neighbor)
                if ismember(neighbor(k), path)
                    idx(end+1) =  k;
                end
            end
            neighbor(idx) = [];
            
            % Eliminar los nodos vecinos que se han visitado
            if isempty(neighbor)
                neighbor = cell(0);
                node_step = node_start;
                path = node_step;
                dist = 0;
                continue
            end
                
            
            %Calcule la probabilidad de acceso del siguiente nodo
            P = neighbor;
            for k=1:length(P)
                P(2,k) = nodes_data{node_step, 4}(k)^alpha * ...
                    nodes_data{node_step, 5}(k)^beta;
            end
            P(2,:) = P(2,:)/sum(P(2,:));
            
            % Metodo de la ruleta para seleccionar el siguiente nodo visitante
            Pc = cumsum(P(2,:));
            Pc = [0, Pc];
            randnum = rand;
            for k = 1:length(Pc)-1
                if randnum > Pc(k) && randnum < Pc(k+1)
                    target_node = neighbor(k);
                end
            end
            
            
            % Calcular la distancia de un solo paso
            idx_temp = find(nodes_data{node_step, 2} == target_node);
            dist = dist + nodes_data{node_step, 3}(idx_temp);
            
            % Actualice el nodo de destino y la ruta establecida para el siguiente paso           
            node_step = target_node;
            path(end+1) = node_step;         
                       
        end
        
        % Almacene la distancia acumulada y la ruta correspondiente de la i-谷sima hormiga
        Length(i,1) = dist;
        route{i,1} = path;
    end
    
    %% Calcule la distancia mas corta y la ruta correspondiente entre los hombres de esta generaci車n.
    if iter == 1
        [min_Length,min_index] = min(Length);
        Length_best(iter) = min_Length;
        Length_ave(iter) = mean(Length);
        Route_best{iter,1} = route{min_index,1};
        
    else
        [min_Length,min_index] = min(Length);
        Length_best(iter) = min(Length_best(iter - 1),min_Length);
        Length_ave(iter) = mean(Length);
        if Length_best(iter) == min_Length
            Route_best{iter,1} = route{min_index,1};
        else
            Route_best{iter,1} = Route_best{iter-1,1};
        end
    end
    
    %% Actualizar feromona
    
    % Calcula la feromona que dejan las hormigas que pasan en cada camino
    Delta_Tau = Delta_Tau_initial;    
    
    % Calcular hormiga por hormiga
    for i = 1:m
       
        % Calcular entre nodos uno por uno
        for j = 1:length(route{i,1})-1
            node_start_temp = route{i,1}(j);
            node_end_temp = route{i,1}(j+1);
            idx =  find(Delta_Tau{node_start_temp, 2} == node_end_temp);
            Delta_Tau{node_start_temp,3}(idx) = Delta_Tau{node_start_temp,3}(idx) + Q/Length(i);
        end
        
    end
    
    % Considere el factor de volatilizaci車n, actualice la feromona
    for i = 1:size(nodes_data, 1)
        nodes_data{i, 4} = (1-rho) * nodes_data{i, 4} + Delta_Tau{i, 3};
    end
    
    % Actualizar iteraciones
    iter = iter + 1;
end


%%Grafica, resultado   

figure
plot(1:iter_max,Length_best,'b',1:iter_max,Length_ave,'r')
legend('Distancia mas corta','Distancia promedio')
xlabel('Numero de iteraciones')
ylabel('distancia')
title('Comparacion de la distancia mas corta y la distancia media de cada generacion')

% Ruta 車ptima
[dist_min, idx] = min(Length_best);
path_opt = Route_best{idx,1};
