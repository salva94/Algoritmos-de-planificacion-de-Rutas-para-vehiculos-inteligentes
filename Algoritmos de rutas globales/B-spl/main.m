% M¨¦todo de curva B-spline
% AUTOR£ºSALVA
% FECHA£º2021/12/3
clc
clear
close all

%%Definici¨®n de datos
k = 4;                                    % B-spline de orden k, k-1 grado
flag = 2;                                  %1,2 Dibujar una curva B-spline uniforme y una curva B-spline casi uniforme respectivamente
d = 3.5;
P=[0, 10, 25, 25, 40, 50;
    -d/2,-d/2,-d/2+0.5,d/2-0.5,d/2,d/2 ];   %n = 5, 6 puntos de control, puede cumplir con la curvatura continua
n = size(P,2)-1;                          % n es el n¨²mero de puntos de control, contando desde 0

%% Generar curva B-spline

path=[];
Bik = zeros(n+1, 1);

if flag == 1     % B-spline uniforme
    NodeVector = linspace(0, 1, n+k+1); %Vector de nodo
    for u = (k-1)/(n+k+1) : 0.001 : (n+2)/(n+k+1)
        for i = 0 : 1 : n
            Bik(i+1, 1) = BaseFunction(i, k-1 , u, NodeVector);
        end
        p_u = P * Bik;
        path = [path; [p_u(1,1),p_u(2,1)]];
    end
    
elseif flag == 2  % B-spline casi uniforme
    NodeVector = U_quasi_uniform(n, k-1); % Vector de nodo de B-spline casi uniforme
    for u = 0 : 0.005 : 1-0.005
        for i = 0 : 1 : n
            Bik(i+1, 1) = BaseFunction(i, k-1 , u, NodeVector);
        end
        p_u = P * Bik;
        path=[path; [p_u(1),p_u(2)]];
    end
else
    fprintf('error!\n');
end

%% GRAFICAR
d = 3.5;               % Ancho est¨¢ndar de la carretera
W = 1.8;               % Anchura del coche
L = 4.7;               % Longitud del coche
figure
len_line = 50;
P0 = [0, -d/2];

% Dibuja un mapa de carreteras gris
GreyZone = [-5,-d-0.5; -5,d+0.5; len_line,d+0.5; len_line,-d-0.5];
fill(GreyZone(:,1),GreyZone(:,2),[0.5 0.5 0.5]);
hold on
fill([P0(1),P0(1),P0(1)-L,P0(1)-L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'b')  

% Dibuja una l¨ªnea divisoria
plot([-5, len_line],[0, 0], 'w--', 'linewidth',2);  %L¨ªnea divisoria
plot([-5,len_line],[d,d],'w','linewidth',2);     %L¨ªnea l¨ªmite izquierda
plot([-5,len_line],[-d,-d],'w','linewidth',2);  %L¨ªnea l¨ªmite izquierda

% Establecer el rango de visualizaci¨®n del eje de coordenadas
axis equal
set(gca, 'XLim',[-5 len_line]); 
set(gca, 'YLim',[-4 4]); 

% Dibujar camino
scatter(path(:,1),path(:,2),100, '.b');%Ptos de la ruta
scatter(P(1,:),P(2,:),'g')
plot(P(1,:),P(2,:),'r');%Ptos rutas