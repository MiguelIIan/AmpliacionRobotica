function [coste,ruta] = NavAutoAest(nodos,g,h)

    % Pedimos los nodos de salida y destino al usuario
    s = input("Introduzca el nodo de salida: ");
    x = input("Introduzca el nodo de destino: ");

    [m,n] = size(g); % Obtenemos el tamaño del grafo de entrada

    caminos = inf(2,m); % Definimos un vector de infinitos
    % Creamos una matriz con: columna de nodos, columna de costes, columna de costes con la heurística
    % y columna de nodo anterior
    caminos = [uint32(1):uint32(m);caminos;zeros(1,m)]';
    % Cambiamos la primera fila por la del nodo inicio y cambiamos su coste
    % a cero
    caminos([1 s],:) = caminos([s 1],:);
    caminos(1,2) = 0;
    caminos(1,3) = h(x,s);
    
    % Creamos la matriz donde vamos a guardar los nodos que ya hemos
    % revisado y la matriz donde pondremos la ruta solución
    revisado = zeros(m,4);
    solucion = zeros(m,4);
    
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
                    % Guardamos el coste con la heurística
                    caminos(nodo,3) = g(i,j) + caminos(1,2) + h(x,j);
                    caminos(nodo,4) = i;
                end
            end
        end
        % Ordenamos la matriz, guardamos en "revisado" el nodo y eliminamos
        % la primera fila para no volver a comprobarlo.
        g(:,caminos(1,1)) = 0;
        revisado(p,:) = caminos(1,:);
        p = p+1;
        caminos(1,:) = [];
        % Reordenamos la matriz en función del coste con heurística menor
        caminos = sortrows(caminos,3);
        i = caminos(1,1);
    end

    revisado(m,:) = caminos(1,:);
    
    solucion(1,:) = revisado(m,:);
    ant = m;
    % Este bucle se va a encargar de recorrer la matriz "revisado" y
    % encontrar cuál es la ruta a seguir
    for j = 2:length(revisado)
        for l = 1:length(revisado(:,1))
           if (revisado(l,1) == revisado(ant,4) && revisado(ant,4)~= 0)
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
    
    % i es el nodo en el que está el robot
    i = 1;
    recorrido = [1]; % Aquí guardaremos si el robot ha llegado a cada nodo del camino
    % Carga del mapa de ocupacion
    map_img=imread('mapa2.pgm');
    map_neg=imcomplement(map_img);
    map_bin=imbinarize(map_neg);
    mapa=binaryOccupancyMap(map_bin);
    show(mapa);
    
    % Bucle que va a pintar en pantalla a la vez que simular el recorrido
    % reactivo del robot
    while (i <= (length(ruta)-1) && recorrido(i)~=0)
        % Llamamos a la función de la navegación reactiva y obtenemos si
        % llega o no al nodo
        recorrido = [recorrido EvitarObsmejor(ruta(i),ruta(i+1),mapa,nodos)];
        i = i + 1;
    end
    % Una vez terminado comprobamos si el robot ha llegado al final o si se
    % ha quedado atascado por el camino y no ha podido llegar.
    if recorrido(i) ~= 1
        fprintf('No se ha podido llegar al destino.\n')
    else
        fprintf('Destino alcanzado.\n')
    end
end