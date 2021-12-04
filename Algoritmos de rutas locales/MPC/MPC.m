% Seguimiento de trayectoria con MPC
% Autor£ºsalva
% Fecgha£º2021/12/4
clc
clear
close all
load path.mat

%% Par¨¢metros iniciales
dt = 0.1;   % tiempo 
L = 2.9;    % Distancia entre ejes
max_steer =60 * pi/180; % in rad
target_v =30.0 / 3.6;

%%Par¨¢metros relacionados de la trayectoria de referencia
% Definir trayectoria de referencia
refPos = path;
refPos_x = refPos(:,1);
refPos_y = refPos(:,2);

% Calcular el ¨¢ngulo y la curvatura del rumbo
diff_x = diff(refPos_x) ;
diff_x(end+1) = diff_x(end);
diff_y = diff(refPos_y);
diff_y(end+1) = diff_y(end);
derivative1 = gradient(refPos_y) ./ abs(diff_x);              % primera derivada
derivative2 = del2(refPos_y) ./ abs(diff_x);                  % segunda derivada
refHeading = atan2(diff_y , diff_x);                   % B¨®veda
refK = abs(derivative2) ./ (1+derivative1.^2).^(3/2);  % Calcular la curvatura

% Seg¨²n el principio de direcci¨®n de Ackerman, calcule el ¨¢ngulo de referencia de la rueda delantera
refDelta = atan(L*refK);

% Figura
figure
plot(refPos_x,refPos_y,'r-')
hold on

%% Programa principal
x = refPos_x(1)+0.5; 
y = refPos_y(1) + 0.5; 
yaw = refHeading(1)+0.02; 
v = 0.1;
U = [0.01;0.01];
idx =0;
pos_actual = [refPos_x,refPos_y];
latError_MPC = [];

% ciclo
while idx < length(refPos_x)-1
    
    % Controlador MPC
    [Delta,v,idx,latError,U] = mpc_control(x,y,yaw,refPos_x,refPos_y,refHeading,refDelta,dt,L,U,target_v) ;
    
    %Si el error es demasiado grande, salimos del ciclo.
    if abs(latError) > 3
        disp('Si el error es demasiado grande, salimos del ciclo!\n')
        break
    end
    
    % Estado de actualizaci¨®n
    [x,y,yaw] = updateState(x,y,yaw,v , Delta, dt,L, max_steer); 
    
    % Guarde la cantidad real de cada paso
    pos_actual(end+1,:) = [x,y];
    latError_MPC(end+1,:) = [idx,latError];
    
    % Dibujar diagrama de trayectoria de seguimiento
    scatter(x,y,150,'b.')
    pause(0.01);
end

%% Guardar
path_MPC = pos_actual;
save path_MPC.mat path_MPC
save latError_MPC.mat latError_MPC