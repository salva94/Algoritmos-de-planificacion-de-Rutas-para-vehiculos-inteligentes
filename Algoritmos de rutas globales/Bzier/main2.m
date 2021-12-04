% Método de la curva de Bezier: planificación de la trayectoria de cambio de carril basada en la curva de Bezier
% AUTOR:SALVA
% FECHA:2021/1/30
clc
clear
close all

%% 
% Definir puntos de control
d = 3.5;
P0 = [0, -d/2];
P1 = [25, -d/2];
P2 = [25, d/2];
P3 = [50, d/2];
P = [P0; P1; P2; P3];

% Obtenga puntos de ruta directamente de acuerdo con la definición de curva de Bezier
n = length(P)-1;
Path = [];
for t = 0:0.01:1
    if n == 1
        p_t = P;
    elseif n >= 2
        p_t = [0, 0];
        for i = 0:n
            k_C = factorial(n) / (factorial(i) * factorial(n-i));
            k_t = (1-t)^(n-i) * t^i;
            p_t = p_t + k_C * k_t * P(i+1,:);
        end
        Path(end+1,:) = p_t;
    
    else
        disp('La entrada del punto de control es incorrecta, vuelva a ingresar')
    end
end


%% Graficar
d = 3.5;               % Ancho estándar de la carretera
W = 1.8;               % Anchura del coche
L = 4.7;               % Longitud del coche
figure
len_line = 50;

% Dibuja un mapa de carreteras gris
GreyZone = [-5,-d-0.5; -5,d+0.5; len_line,d+0.5; len_line,-d-0.5];
fill(GreyZone(:,1),GreyZone(:,2),[0.5 0.5 0.5]);
hold on
fill([P0(1),P0(1),P0(1)-L,P0(1)-L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'b')  

% Dibuja una línea divisoria
plot([-5, len_line],[0, 0], 'w--', 'linewidth',2);  %Línea divisoria
plot([-5,len_line],[d,d],'w','linewidth',2);     %Línea límite izquierda
plot([-5,len_line],[-d,-d],'w','linewidth',2);  %Línea límite izquierda

% Establecer el rango de visualización del eje de coordenadas
axis equal
set(gca, 'XLim',[-5 len_line]); 
set(gca, 'YLim',[-4 4]); 

% Dibujar camino
scatter(P(:,1),P(:,2),'g')
plot(P(:,1),P(:,2),'r');%puntos de la ruta
scatter(Path(:,1),Path(:,2),200, '.b');%puntos de la ruta

