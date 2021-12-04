function NodeVector = U_quasi_uniform(n, k)
% C¨¢lculo del vector de nodo de B-spline cuasi uniforme, v¨¦rtices de control n + 1 totales, k-¨¦simo B-spline, orden k + 1
NodeVector = zeros(1, n+k+2);
piecewise = n - k + 1;       % El n¨²mero de segmentos de la curva.
if piecewise == 1            % Cuando solo hay una curva, n = k
    for i = k+2 : n+k+2
        NodeVector(1, i) = 1;
    end
else
    flag = 1;           % Cuando hay m¨¢s de una curva
    while flag ~= piecewise
        NodeVector(1, k+flag+1) = NodeVector(1, k + flag) + 1/piecewise;
        flag = flag + 1;
    end
    NodeVector(1, n+2 : n+k+2) = 1;   % Hay (k + 1) valores repetidos (orden) antes y despu¨¦s del vector de nodo
end
