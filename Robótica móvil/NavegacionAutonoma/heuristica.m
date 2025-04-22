
for i = 1:49
    for j = 1:49
        if(j==i)
            H(i,j) = 0;
        else
            % Calculamos distancia euclídea entre el nodo i y el j y le
            % restamos un número aleatorio entre 1 y 3 para que la matriz
            % de heurística sea mejor
            H(i,j) = sqrt((nodos(i,2)-nodos(j,2))^2+(nodos(i,3)-nodos(j,3))^2)-randi(3);
        end
        % Comprobamos si los nodos están o no conectados y calculamos sus
        % distancias euclídeas
        if (costes(i,j) ~= 0)
            costes(i,j) = sqrt((nodos(i,2)-nodos(j,2))^2+(nodos(i,3)-nodos(j,3))^2);
        end
    end
end
