% Método de campo de potencial artificial
% AUTOR:SALVA
% FECHA:2021/12/2
clc
clear
close all

%% Inicializar los parámetros del coche.
d = 3.5;               % Ancho estándar de la carretera
W = 1.8;               % Anchura del coche
L = 4.7;               % Longitud del coche

P0 = [0,-d/2,1,1];     % Información del punto de partida del vehículo, 1-2 columnas de posición, 3-4 columnas de velocidad
Pg = [99,d/2,0,0];     % ubicación del objetivo
Pobs = [15,7/4,0,0;
    30,-3/2,0,0;
    45,3/2,0,0;
    60,-3/4,0,0;
    80,7/4,0,0];       % Ubicación del obstáculo
P = [Pobs;Pg];         % Ponga la posición de destino y la posición del obstáculo juntas

Eta_att = 5;           % Calcule el factor de ganancia de la gravedad
Eta_rep_ob = 15;       % Calcule el factor de ganancia de repulsión
Eta_rep_edge = 50;     % Calcule el coeficiente de ganancia de la repulsión del límite.

d0 = 20;               % Distancia de influencia del obstáculo
n = size(P,1);         % Número total de obstáculos y goles
len_step = 0.5;        % Numero de pies
Num_iter = 200;        % Número máximo de iteraciones de bucle

%% ***************La inicialización ha terminado y comienza el ciclo principal.******************
Pi = P0;               %Asignar las coordenadas de partida del coche a Xi
i = 0;
while sqrt((Pi(1)-P(n,1))^2+(Pi(2)-P(n,2))^2) > 1
    i = i + 1;
    Path(i,:) = Pi;    %Guarda las coordenadas de cada punto por el que ha viajado el coche.
    
    %Calcule el vector de dirección unitaria y el vector de velocidad de la posición actual del vehículo y el obstáculo
    for j = 1:n-1    
        delta(j,:) = Pi(1,1:2) - P(j,1:2);                              % Repulsión expresa con puntos de vehículos-puntos de obstáculos.
        dist(j,1) = norm(delta(j,:));                                   % La distancia entre la posición actual del vehículo y el obstáculo.
        unitVector(j,:) = [delta(j,1)/dist(j,1), delta(j,2)/dist(j,1)]; % Vector de repulsión de dirección unitaria
    end
    
    %Calcule el vector de dirección unitaria y el vector de velocidad de la posición actual del vehículo y el objetivo
    delta(n,:) = P(n,1:2)-Pi(1,1:2);                                    %Utilice el punto de destino del vehículo para expresar la gravedad  
    dist(n,1) = norm(delta(n,:)); 
    unitVector(n,:)=[delta(n,1)/dist(n,1),delta(n,2)/dist(n,1)];

   %% Calcular la repulsión
    % Aumente el factor de ajuste del objetivo (es decir, la distancia desde el vehículo al objetivo) a la función de campo potencial de fuerza repulsiva original, de modo que la fuerza repulsiva también sea 0 después de que el vehículo alcance el punto objetivo
    for j = 1:n-1
        if dist(j,1) >= d0
            F_rep_ob(j,:) = [0,0];
        else
            % Repulsión de obstáculos 1, la dirección es del obstáculo al vehículo
            F_rep_ob1_abs = Eta_rep_ob * (1/dist(j,1)-1/d0) * dist(n,1) / dist(j,1)^2;         
            F_rep_ob1 = [F_rep_ob1_abs*unitVector(j,1), F_rep_ob1_abs*unitVector(j,2)];   
           
            % Repulsión de obstáculos 2, la dirección es desde el vehículo hasta el punto de destino
            F_rep_ob2_abs = 0.5 * Eta_rep_ob * (1/dist(j,1) - 1/d0)^2;                
            F_rep_ob2 = [F_rep_ob2_abs * unitVector(n,1), F_rep_ob2_abs * unitVector(n,2)];  
            
            % Cálculo mejorado de la fuerza de repulsión de obstáculos
            F_rep_ob(j,:) = F_rep_ob1+F_rep_ob2;                                   
        end
    end
    
    
    % Aumente el campo de potencial de repulsión de límite y seleccione la función de repulsión correspondiente de acuerdo con la posición actual del vehículo.
    if Pi(1,2) > -d+W/2 && Pi(1,2) <= -d/2             %Campo de fuerza en el área límite de la carretera inferior, la dirección apunta al eje y positivo
        F_rep_edge = [0,Eta_rep_edge * norm(Pi(:,3:4))*(exp(-d/2-Pi(1,2)))];
    elseif Pi(1,2) > -d/2 && Pi(1,2) <= -W/2           %El campo de fuerza del área límite inferior de la carretera, la dirección apunta a la dirección negativa del eje y
        F_rep_edge = [0,1/3 * Eta_rep_edge * Pi(1,2).^2];
    elseif Pi(1,2) > W/2  && Pi(1,2) < d/2             %El campo de fuerza del área límite superior de la carretera, la dirección apunta a la dirección positiva del eje y 
        F_rep_edge = [0, -1/3 * Eta_rep_edge * Pi(1,2).^2];
    elseif Pi(1,2) > d/2 && Pi(1,2)<=d-W/2             %El campo de fuerza del área límite superior de la carretera, la dirección apunta a la dirección negativa del eje y
        F_rep_edge = [0, Eta_rep_edge * norm(Pi(:,3:4)) * (exp(Pi(1,2)-d/2))];
    end
    
    %% Calcule la fuerza y ​​la dirección resultantes
    F_rep = [sum(F_rep_ob(:,1))  + F_rep_edge(1,1),...
           sum(F_rep_ob(:,2)) + F_rep_edge(1,2)];                                      % El vector de repulsión de todos los obstáculos.
    F_att = [Eta_att*dist(n,1)*unitVector(n,1), Eta_att*dist(n,1)*unitVector(n,2)];    % Vector de gravedad
    F_sum = [F_rep(1,1)+F_att(1,1),F_rep(1,2)+F_att(1,2)];                             % Vector de fuerza total
    UnitVec_Fsum(i,:) = 1/norm(F_sum) * F_sum;                                         % Vector unitario de fuerza total
    
    %Calcula la siguiente posición del coche.
    Pi(1,1:2)=Pi(1,1:2)+len_step*UnitVec_Fsum(i,:);                     

%     %Determina si llegar al final
%     if sqrt((Pi(1)-P(n,1))^2+(Pi(2)-P(n,2))^2) < 0.2 
%         break
%     end
end
Path(i,:)=P(n,:);            %Asignar el último punto del vector de ruta al objetivo

%% Graficar
figure
len_line = 100;



%Dibuja un mapa de carreteras gris
GreyZone = [-5,-d-0.5; -5,d+0.5; len_line,d+0.5; len_line,-d-0.5];
fill(GreyZone(:,1),GreyZone(:,2),[0.5 0.5 0.5]);
hold on
fill([P0(1),P0(1),P0(1)-L,P0(1)-L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'b')  %Coche 2

% Dibuja una línea divisoria
plot([-5, len_line],[0, 0], 'w--', 'linewidth',2);  %Línea divisoria
plot([-5,len_line],[d,d],'w','linewidth',2);     %Línea límite izquierda
plot([-5,len_line],[-d,-d],'w','linewidth',2);  %Línea límite izquierda

%Establecer el rango de visualización del eje de coordenadas
axis equal
set(gca, 'XLim',[-5 len_line]); 
set(gca, 'YLim',[-4 4]); 


% Dibujar camino
plot(P(1:n-1,1),P(1:n-1,2),'ro');   %Ubicación del obstáculo
plot(P(n,1),P(n,2),'gv');       %ubicación del objetivo
plot(P0(1,1),P0(1,2),'bs');    %Posición inicial
plot(Path(:,1),Path(:,2),'.b');%puntos de la ruta