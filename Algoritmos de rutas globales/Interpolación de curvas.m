% Interpolación de curvas
% AUTOR: SALVA
% FECHA:2021/12/2
clc
clear
close all

%% Definición de escena
%%Definición de parámetros relacionados de la sección de la carretera y el vehículo en la escena de cambio de carril
d = 3.5;          % Ancho estándar de la carretera
len_line = 30;    % Longitud de la línea recta
W = 1.75;         % Anchura del coche
L = 4.7;          % Longitud del coche
x1 = 20;          %Coche x coordenada

%El estado inicial del cambio de carril del vehículo y el estado final deseado
t0 = 0;
t1 = 3;
state_t0 = [0, -d/2; 5, 0; 0, 0];  % x,y; vx,vy; ax,ay
state_t1 = [20, d/2; 5, 0; 0, 0];
x2 = state_t0(1);

%% Dibujar la escena
figure(1)
%Dibuja un mapa de carreteras gris
GreyZone = [-5,-d-0.5; -5,d+0.5; len_line,d+0.5; len_line,-d-0.5];
fill(GreyZone(:,1),GreyZone(:,2),[0.5 0.5 0.5]);
hold on

%Dibujar un coche
fill([x1,x1,x1+L,x1+L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'b')  %Coche 1
fill([x2,x2,x2-L,x2-L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'y')  %Coche 2

%Dibuja una línea divisoria
plot([-5, len_line],[0, 0], 'w--', 'linewidth',2);%Línea divisoria
plot([-5,len_line],[d,d],'w','linewidth',2);  %Línea límite izquierda
plot([-5,len_line],[-d,-d],'w','linewidth',2);  %Línea límite izquierda

%Establecer el rango de visualización del eje de coordenadas
axis equal
set(gca, 'XLim',[-5 len_line]); 
set(gca, 'YLim',[-4 4]); 

%% Generación de trayectoria polinomial de quinto grado

% Calcule las dos matrices de coeficientes de A y B
X = [state_t0(:,1); state_t1(:,1)];
Y = [state_t0(:,2); state_t1(:,2)];
T = [ t0^5      t0^4      t0^3     t0^2    t0   1;
      5*t0^4    4*t0^3    3*t0^2   2*t0    1    0;
      20*t0^3   12*t0^2   6*t0     1       0    0;
      t1^5      t1^4      t1^3     t1^2    t1   1;
      5*t1^4    4*t1^3    3*t1^2   2*t1    1    0;
      20*t1^3   12*t1^2   6*t1     1       0    0];
A = T \ X;
B = T \ Y;

%Discretizar el tiempo de t0 a t1 para obtener las coordenadas de la trayectoria en el momento discreto
t=(t0:0.05:t1)';
path=zeros(length(t),4);%1-4 Las columnas se almacenan por separado x,y,vx,vy 
for i = 1:length(t)
    %Coordenadas de posición longitudinal
    path(i,1) = [t(i)^5, t(i)^4, t(i)^3, t(i)^2, t(i), 1] * A;
    
    %Coordenadas de posición horizontal
    path(i,2) = [t(i)^5, t(i)^4, t(i)^3, t(i)^2, t(i), 1] * B;
    
    % Velocidad longitudinal
    path(i,3) = [5*t(i)^4,  4*t(i)^3,  3*t(i)^2,  2*t(i), 1, 0] * A;
    
    %Velocidad longitudinal
    path(i,4) = [5*t(i)^4,  4*t(i)^3,  3*t(i)^2,  2*t(i), 1, 0] * B;
end

% Dibujar trayectoria de cambio de carril
plot(path(:,1),path(:,2),'r--','linewidth',1.5); 

%%Velocidad longitudinal

% Velocidad longitudinal
figure 
plot(t, path(:,4), 'k'); 
xlabel('tiempo / s ');
ylabel('Velocidad lateral / m/s ');

% Velocidad longitudinal
figure 
plot(t, path(:,3), 'k'); 
xlabel('tiempo / s ');
ylabel('Velocidad longitudinal / m/s ');

