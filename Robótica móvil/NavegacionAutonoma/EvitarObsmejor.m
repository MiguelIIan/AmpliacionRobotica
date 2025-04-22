function alcanzable = EvitarObsmejor(ni,nf,mapa,nodos)
    
    % Configuracion del sensor (laser de barrido)
    max_rango=10;
    angulos=-pi/2:(pi/180):pi/2; % resolucion angular barrido laser
    
    % Caracteristicas del vehiculo y parametros del metodo
    v=0.4;            % Velocidad del robot
    D=1.5;           % Rango del efecto del campo de repulsión de los obstáculos
    alfa=1;           % Coeficiente de la componente de atracción
    beta=100;      % Coeficiente de la componente de repulsión

    % Marcar los puntos de inicio y destino
    hold on;
    title('Señala los puntos inicial y final de la trayectoria del robot');
    origen=nodos(ni,2:3);
    plot(origen(1), origen(2), 'go','MarkerFaceColor','green');  % Dibujamos el origen
    destino=nodos(nf,2:3);
    plot(destino(1), destino(2), 'ro','MarkerFaceColor','red');  % Dibujamos el destino

    % Inicialización
    robot=[origen 0];     % El robot empieza en la posición de origen (orientacion cero)
    path = [];
    path = [path; robot]; % Se añade al camino la posicion actual del robot
    iteracion=0;              % Se controla el nº de iteraciones por si se entra en un minimo local
   
    ant = [inf inf];
    cambio = 0;
    sector = atan2(destino(2)-origen(2),destino(1)-origen(1));

    % Calculo de la trayectoria
    while norm(destino-robot(1:2)) > v && iteracion<1000    % Hasta menos de una iteración de la meta (10 cm)
        % Definimos a 0 el vector con la fuerza de repulsión
        Frep = [0 0];
        % Calculamos con la ecuación el valor de la fuerza de atracción
        Fatr = alfa.*(destino - robot(1:2)); 
        Ftg = [0 0];
        %cerca = 10;
        % Usamos la función que nos devolverá la distancia a los obstáculos cercanos
        obs = Lidar(robot,mapa,angulos,max_rango);
        % Comprobamos si tenemos un objeto cercano recorriendo la matriz "obs"
        for i = 1:length(obs)
            % Comprobamos que el valor que estamos viendo no es nan, que indica
            % fuera de rango
           if (not(isnan(obs(i,1))))
              
               % calculamos la distancia entre el robot y el punto del
               % obstáculo
               rho = (robot(1:2)-obs(i,:));
               d = sqrt(rho(1)^2+rho(2)^2);
               % Comprobamos si la distancia es menor a la umbral definida
               % anteriormente
               if (d<=D)
                   % Sumamos a la fuerza de repulsión acumulada, la fuerza de repulsión que
                   % genera el punto que estamos comprobando
                   Frep = Frep + beta*((1/d)-(1/D))*((robot(1:2)-obs(i,:))/d^3);
               end
           end
        end
        
        % Creo el vector de la fuerza tangente a la de repulsión como su
        % vector normal. El vector normal de una recta Ax+By+C=0 es [A B]
        % mientras que el vector director es [-B A], por lo tanto, el
        % vector normal positivo a Frep será [Frep(2) -Frep(1)] y el
        % negativo será el opuesto.
        % Calculando el sector por el que nos vamos a mover desde el origen
        % al destino, decidimos una dirección de la fuerza tangente
        if (sector<=pi/2 && sector>=0)
            Ftg=-[Frep(2) -Frep(1)]; % Vector normal negativo a Frep
        elseif(sector>=-pi/2 && sector<=0)
            Ftg=[1.5*Frep(2) -Frep(1)]; % Vector normal positivo a Frep
        elseif(sector<=-pi/2)
            Ftg=-[Frep(2) -Frep(1)]; % Vector normal negativo a Frep
        else
            Ftg=[Frep(2) -Frep(1)]; % Vector normal positivo a Frep
        end
        % Calculamos la Fuerza resultante
        Fsol = Fatr + Frep;
        % Comprobamos en que sentido queremos que nos afecte la fuerza
        % tangente
        if (cambio == 1)
            Ftg = -Ftg;
            % Comprobamos si nos estamos quedando en un mínimo local, ya
            % que la fuerza de atracción no variará para cambiar el sentido
            % de la fuerza tangente
            if (iteracion-ite >= 100)
                if (norm(Fatr) >= norm(ant(iteracion-50,:))-0.1 && norm(Fatr) <= norm(ant(iteracion-50,:))+0.1)
                    cambio = 0;
                    Ftg = -Ftg;
                end
            end
        else
            % Comprobamos si nos estamos quedando en un mínimo local, ya
            % que la fuerza de atracción no variará para cambiar el sentido
            % de la fuerza tangente
            if (iteracion>50)
                ite = iteracion;
                if (norm(Fatr) >= norm(ant(iteracion-50,:))-0.1 && norm(Fatr) <= norm(ant(iteracion-50,:))+0.1)
                    Ftg = -Ftg;
                    cambio = 1;
                end
            end
        end
        % Guardamos la fuerza tangente anterior para compararla con la
        % siguiente
        ant = [ant; Fatr];
        % Calculamos la fuerza resultante con la fuerza tangente
        Fsol = Fsol + Ftg;

        % Hacemos unitario el vector fuerza para que no influya en la velocidad
        unitario = Fsol/sqrt(Fsol(1)^2+Fsol(2)^2);
        % Definimos la nueva posición del robot tras moverse
        robot = [robot(1)+unitario(1)*v robot(2)+unitario(2)*v atan2(unitario(2),unitario(1))];
        path = [path;robot];	% Se añade la nueva posición al camino seguido
        plot(path(:,1),path(:,2),'r'); % Pintamos el movimiento
        drawnow
        iteracion=iteracion+1;
    end
    
    if iteracion==1000   % Se ha caído en un mínimo local
        alcanzable = 0;
    else
        alcanzable = 1;
    end
end