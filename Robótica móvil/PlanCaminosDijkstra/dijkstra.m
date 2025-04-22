function [coste,ruta] = dijkstra(g,s,x)

    
    [m,n] = size(g); % Obtenemos el tamaño del grafo de entrada

    caminos = Inf(1,m); % Definimos un vector de infinitos
    % Creamos una matriz con: columna de nodos, columna de costes y columna
    % de nodo anterior
    caminos = [1:m;caminos;zeros(1,m)]';
    % Cambiamos la primera fila por la del nodo inicio y cambiamos su coste
    % a cero
    caminos([1 s],:) = caminos([s 1],:);
    caminos(1,2) = 0;
    
    % Creamos la matriz donde vamos a guardar los nodos que ya hemos
    % revisado y la matriz donde pondremos la ruta solución
    revisado = zeros(m,3);
    solucion = zeros(m,3);
    
    % i es el nodo que estamos revisando y p es la posición en la matriz de
    % revisados donde vamos a poner el nodo que acabemos de ver
    i = s;
    p = 1;
    
    
    while i ~= x
        % Creamos un bucle para recorrer las columnas de la matriz del
        % grafo
        for j = 1:length(g(1,:))
            % Este bucle nos devolverá la fila donde está ubicado el nodo
            % que buscamos en la matriz "caminos"
            for l = 1:length(caminos(:,1))
               if (caminos(l,1) == j)
                    nodo = l;
               end
            end
            % Comprobamos si el elemento que estamos viendo es distinto de
            % cero y si el coste hasta el nodo es menor que el coste
            % anterior y guardamos la información en la matriz "caminos"
            if (g(i,j) ~= 0 )
                if (g(i,j)+caminos(1,2) < caminos(nodo,2))
                    caminos(nodo,2) = g(i,j) + caminos(1,2);
                    caminos(nodo,3) = i;
                end
            end
        end
        % Guardamos en "revisado" el nodo, volvemos a cero la columna del nodo en la matriz del grafo
        % eliminamos la primera fila para no volver a comprobarla y ordenamos la matriz
        g(:,caminos(1,1)) = 0;
        revisado(p,:) = caminos(1,:);
        p = p+1;
        caminos(1,:) = [];
        caminos = sortrows(caminos,2);
        i = caminos(1,1);
    end

    % Guardamos la fila del nodo destino
    revisado(p,:) = caminos(1,:);
    % Guardamos el nodo destino en la solución y creamos la variable con la
    % que vamos a ver el nodo del que hemos llegado
    solucion(1,:) = revisado(p,:);
    ant = p;
    % Este bucle se va a encargar de recorrer la matriz "revisado" y
    % encontrar cuál es la ruta a seguir
    for j = 2:length(revisado)
        for l = 1:length(revisado(:,1))
           if (revisado(l,1) == revisado(ant,3) && revisado(ant,3)~= 0)
                ant = l;
                solucion(j,:) = revisado(l,:);
           end
        end
    end
    
    % Elimino las filas de la matriz que siguen valiendo 0
    solucion(sum(abs(solucion),2)==0,:)=[ ];
    solucion(:,sum(abs(solucion),1)==0)=[ ];
    
    % Defino el coste y la ruta, siendo esta la columna de los nodos de la
    % solución invertida, para poder ver el recorrido desde la salida a la
    % llegada
    coste = solucion(1,2);
    ruta = fliplr(solucion(:,1)');
end