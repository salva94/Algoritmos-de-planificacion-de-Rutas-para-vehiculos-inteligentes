function    [Delta_real,v_real,idx,latError,U ] = ...
    mpc_control(x,y,yaw,refPos_x,refPos_y,refPos_yaw,refDelta,dt,L,U,target_v)
%% Par芍metros preestablecidos de MPC
Nx = 3;         % N迆mero de cantidades estatales
Nu = 2;         % numero de controladores
Np = 60;        % horizonte de prediccion de predicci車n
Nc = 30;        % h.prediccion
row = 10;       % Factor de relajaci車n
Q = 100*eye(Np*Nx);      % (Np*Nx) ℅ (Np*Nx)
R = 1*eye(Nc*Nu);        % (Nc*Nu) ℅ (Nc*Nu)

% Restricci車n de cantidad de control
umin = [-0.2; -0.54];
umax = [0.2; 0.332];
delta_umin = [-0.05; -0.64];
delta_umax = [0.05; 0.64];

%%Matriz de correlaci車n de la ecuaci車n del espacio de estados de error de la cinem芍tica original
% Calcular la cantidad de control de referencia
idx = calc_target_index(x,y,refPos_x,refPos_y); 
v_r = target_v;
Delta_r = refDelta(idx);
heading_r = refPos_yaw(idx);

% Cantidad de estado real y cantidad de estado de referencia
X_real = [x,y,yaw];
Xr = [refPos_x(idx), refPos_y(idx), refPos_yaw(idx)];

% Encuentra el error de posici車n y 芍ngulo de rumbo
x_error  = x - refPos_x(idx);
y_error = y - refPos_y(idx);

% Seg迆n Baidu Apolo, calcula el error lateral
latError = y_error*cos(heading_r) - x_error*sin(heading_r);

% a,b dos matrices
a = [1    0   -v_r*sin(heading_r)*dt;
     0    1   v_r*cos(heading_r)*dt;
     0    0   1];
b = [cos(heading_r)*dt     0;
     sin(heading_r)*dt     0;
     tan(heading_r)*dt/L   v_r*dt/(L * (cos(Delta_r)^2))];

%% Matriz de correlaci車n de la nueva ecuaci車n del espacio de estados
% Volumen de estado nuevo
kesi = zeros(Nx+Nu,1);              % (Nx+Nu) ℅ 1
kesi(1:Nx) = X_real - Xr;
kesi(Nx+1:end) = U;

% Nueva matriz A
A_cell = cell(2,2);
A_cell{1,1} = a;
A_cell{1,2} = b;
A_cell{2,1} = zeros(Nu,Nx);
A_cell{2,2} = eye(Nu);
A = cell2mat(A_cell);           % (Nx+Nu) ℅ (Nx+Nu)

% Nueva matriz B
B_cell = cell(2,1);
B_cell{1,1} = b;
B_cell{2,1} = eye(Nu);
B = cell2mat(B_cell);           % (Nx+Nu) ℅ Nu

% Nueva matriz C
C = [eye(Nx), zeros(Nx, Nu)];   % Nx ℅ (Nx+Nu)

% Matriz PHI
PHI_cell = cell(Np,1);
for i = 1:Np
    PHI_cell{i,1}=C*A^i;  % Nx ℅ (Nx+Nu)
end
PHI = cell2mat(PHI_cell);   % (Nx * Np) ℅ (Nx + Nu)


% Matriz THETA
THETA_cell = cell(Np,Nc);
for i = 1:Np
    for j = 1:Nc
        if j <= i
            THETA_cell{i,j} = C*A^(i-j)*B;    % Nx ℅ Nu
        else
            THETA_cell{i,j} = zeros(Nx,Nu);
        end
    end
end
THETA = cell2mat(THETA_cell);                 % (Nx * Np) ℅ (Nu * Nc)


%% Matriz de correlaci車n de la funci車n objetivo cuadr芍tica

% Matriz H
H_cell = cell(2,2);
H_cell{1,1} = THETA'*Q*THETA + R;  % (Nu * Nc) ℅ (Nu * Nc)
H_cell{1,2} = zeros(Nu*Nc,1);
H_cell{2,1} = zeros(1,Nu*Nc);
H_cell{2,2} = row;
H = cell2mat(H_cell);            % (Nu * Nc + 1) ℅ (Nu * Nc + 1)

% Matriz E
E = PHI*kesi;                    % (Nx * Np) ℅ 1

% Matriz g
g_cell = cell(1,1);
g_cell{1,1} = E'*Q*THETA;          % (Nu * Nc ) ℅ 1ㄛPara hacer coincidir el n迆mero de filas con el n迆mero de columnas en H, agregue una nueva columna 0
g_cell{1,2} = 0;
g = cell2mat(g_cell);              % (Nu * Nc + 1 ) ℅ 1

%% Matriz de correlaci車n de restricciones

% Matriz A_I
A_t = zeros(Nc,Nc);     % Cuadrado triangular inferior
for i = 1:Nc
    A_t(i,1:i) = 1;
end
A_I = kron(A_t,eye(Nu));       % (Nu * Nc) ℅ (Nu * Nc)

% Matriz Ut
Ut = kron(ones(Nc,1),U);       % (Nu * Nc) ℅ 1

% Restricciones sobre la cantidad de control y el cambio de la cantidad de control
Umin = kron(ones(Nc,1),umin);
Umax = kron(ones(Nc,1),umax);
delta_Umin = kron(ones(Nc,1),delta_umin);
delta_Umax = kron(ones(Nc,1),delta_umax);

% Matriz A para la restricci車n de desigualdad de la funci車n quadprog Ax <= b
A_cons_cell = {A_I, zeros(Nu*Nc,1);       % Para hacer coincidir el n迆mero de columnas en H, agregue una nueva columna 0ㄛ(Nu * Nc) ℅ (Nu * Nc), (Nu * Nc) ℅1
    -A_I, zeros(Nu*Nc,1)}; 
A_cons = cell2mat(A_cons_cell);           % (Nu * Nc * 2) ℅ (Nu * Nc +1)

% Vector b utilizado para la restricci車n de desigualdad de la funci車n quadprog Ax <= b
b_cons_cell = {Umax-Ut;
    -Umin+Ut};
b_cons = cell2mat(b_cons_cell);

% L赤mites superior e inferior →U
lb = delta_Umin;
ub = delta_Umax;

%% Iniciar el proceso de soluci車n

options = optimoptions('quadprog','Display','iter','MaxIterations',100,'TolFun',1e-16);
delta_U = quadprog(H,g,A_cons,b_cons,[],[],lb,ub,[],options);   %(Nu * Nc +1) ℅ 1

%% Salida de c芍lculo

% Seleccione solo el primer conjunto de variables de control de delta_U a resolver. Nota: aqu赤 est芍 la cantidad de cambio en v_tilde y la cantidad de cambio en Delta_tilde
delta_v_tilde = delta_U(1);
delta_Delta_tilde = delta_U(2);

%Actualice la cantidad de control en este momento. Tenga en cuenta que la "cantidad de control" aqu赤 es v_tilde y Delta_tilde, no la v real y Delta
U(1) = kesi(4) + delta_v_tilde; 
U(2) = kesi(5) + delta_Delta_tilde;  

% Resuelve las variables de control reales v_real y Delta_real
v_real = U(1) + v_r; 
Delta_real = U(2) + Delta_r;

end