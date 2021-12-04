% Metodo de Stanley
% AutorㄩSalva
% Fechaㄩ2021/12/04
clc
clear
close all
load  path.mat

%% Definici車n de par芍metros relacionados
% Trayectoria de referencia
RefPos = path;            

% Calcule el 芍ngulo de rumbo de referencia de la trayectoria.
diff_x = diff(RefPos(:,1)) ;
diff_x(end+1) = diff_x(end);
diff_y = diff(RefPos(:,2)) ;
diff_y(end+1) = diff_y(end);
RefHeading = atan2(diff_y ,diff_x);

% Otros par芍metros constantes
targetSpeed = 10;           %Velocidad k queremos alcanzar, unidadㄩ m /s
InitialState = [RefPos(1,:)-0.5,RefHeading(1)+0.02,0];  % Posici車n longitudinal, posici車n horizontal, 芍ngulo de rumbo, velocidad
k = 5;                      % Par芍metro de ganancia
Kp = 1;                     % Coeficiente del controlador de velocidad P
dt = 0.1;                   % Intervalo de tiempo, unidadㄩs
L = 2;                      % Distancia entre ejes del veh赤culo, unidadㄩm

%% Programa principal
% Definici車n del estado inicial del veh赤culo
state = InitialState;
state_actual = state;
idx = 1;
latError_Stanley = [];

% Seguimiento
while idx < size(RefPos,1)-1
    % Encuentra el waypoint m芍s cercano dentro de la distancia de vista previa
    idx = findTargetIdx(state,RefPos);
       
    % Calcule el 芍ngulo de la rueda delantera
    [delta,latError] = stanley_control(idx,state,RefPos,RefHeading,k);
    
    % Si el error es demasiado grande, salga de la pista.
    if abs(latError) > 3
        disp('El error es demasiado grande, salga del programa.!\n')
        break
    end
    
    % Calcular aceleraci車n
    a = Kp* (targetSpeed-state(4));
    
    % Estado de actualizaci車n
    state_new = UpdateState(a,state,delta,dt,L);
    state = state_new;
    
    % Guarde la cantidad real de cada paso
    state_actual(end+1,:) = state_new;
    latError_Stanley(end+1,:) =  [idx,latError];
end

% Figuras
figure
plot(RefPos(:,1), RefPos(:,2), 'r');
xlabel('Coordenadas longitudinales / m');
ylabel('Coordenada horizontal / m');
hold on
for i = 1:size(state_actual,1)
    scatter(state_actual(i,1), state_actual(i,2),150, 'b.');
    pause(0.01)
end
legend('trayectoria planificada', 'Trayectoria real del vehiculo')

%  Guardar
path_stanley = state_actual(:,1:2);
save path_stanley.mat path_stanley;
save latError_Stanley.mat latError_Stanley
%% Primero busque el punto m芍s cercano a la posici車n actual en la trayectoria de referencia
function target_idx = findTargetIdx(state,RefPos)
for i = 1:size(RefPos,1)
    d(i,1) = norm(RefPos(i,:) - state(1:2));
end
[~,target_idx] = min(d);  % Encuentre el n迆mero de serie de un punto de seguimiento de referencia m芍s cercano a la posici車n actual
end

%% Ganancia de control
function [delta,latError] = stanley_control(idx,state,RefPos,RefHeading,k)

% De acuerdo con Baidu Apolo, calcula el error lateral
dx = state(1) - RefPos(idx,1);
dy = state(2) - RefPos(idx,2);
phi_r = RefHeading(idx);
latError = dy*cos(phi_r) - dx*sin(phi_r);

% Calcule theta que solo considere el error de rumbo y theta que solo considere el error lateral
theta_fai =  RefHeading(idx)- state(3);
theta_y = atan2(-k*latError,state(4));

%La combinaci車n de los dos 芍ngulos es el 芍ngulo de la rueda delantera.
delta = theta_fai + theta_y;
end

%% Estado de actualizaci車n
function state_new = UpdateState(a,state_old,delta,dt,L)
state_new(1) =  state_old(1) + state_old(4)*cos(state_old(3))*dt; %Coordenadas longitudinales
state_new(2) =  state_old(2) + state_old(4)*sin(state_old(3))*dt; %Coordenada horizontal
state_new(3) =  state_old(3) + state_old(4)*dt*tan(delta)/L;      %B車veda
state_new(4) =  state_old(4) + a*dt;                              %Velocidad longitudinal
end
