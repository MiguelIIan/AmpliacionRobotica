% AMPLIACION DE ROBOTICA
% PRACTICA 4: Navegacion local con campos potenciales
% Evitar obstaculos

clc
clearvars
close all
%% Carga del mapa de ocupacion

map_img=imread('mapa1_150.png');
map_neg=imcomplement(map_img);
map_bin=imbinarize(map_neg);
mapa=binaryOccupancyMap(map_bin);
show(mapa);

% Marcar los puntos de inicio y destino
hold on;
title('Señala los puntos inicial y final de la trayectoria del robot');
origen=ginput(1);
plot(origen(1), origen(2), 'go','MarkerFaceColor','green');  % Dibujamos el origen
destino=ginput(1);
plot(destino(1), destino(2), 'ro','MarkerFaceColor','red');  % Dibujamos el destino

% Configuracion del sensor (laser de barrido)
max_rango=10;
angulos=-pi/2:(pi/180):pi/2; % resolucion angular barrido laser

% Caracteristicas del vehiculo y parametros del metodo
v=0.4;            % Velocidad del robot
D=1.5;           % Rango del efecto del campo de repulsión de los obstáculos
alfa=1;           % Coeficiente de la componente de atracción
beta=100;      % Coeficiente de la componente de repulsión

%% Inicialización

robot=[origen 0];     % El robot empieza en la posición de origen (orientacion cero)
path = [];                 % Se almacena el camino recorrido
path = [path; robot]; % Se añade al camino la posicion actual del robot
iteracion=0;              % Se controla el nº de iteraciones por si se entra en un minimo local

%% Calculo de la trayectoria

while norm(destino-robot(1:2)) > v && iteracion<1000    % Hasta menos de una iteración de la meta (10 cm)
    % TU CODIGO AQUI %%%%%%%%%%%%%

    % Definimos a 0 el vector con la fuerza de repulsión
    Frep = [0 0];
    % Calculamos con la ecuación el valor de la fuerza de atracción
    Fatr = alfa.*(destino - robot(1:2)); 
    % Usamos la función que nos devolverá la distancia a los obstáculos cercanos
    obs = SimulaLidar(robot,mapa,angulos,max_rango); 
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
    % Calculamos la Fuerza resultante
    Fsol = Fatr + Frep;

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
    fprintf('No se ha podido llegar al destino.\n')
else
    fprintf('Destino alcanzado.\n')
end

%% funcion para simular el sensor
function [obs]=SimulaLidar(robot, mapa, angulos, max_rango)
    obs=rayIntersection(mapa,robot,angulos, max_rango);
    % plot(obs(:,1),obs(:,2),'*r') % Puntos de interseccion lidar
    % plot(robot(1),robot(2),'ob') % Posicion del robot
    % for i = 1:length(angulos)
    %     plot([robot(1),obs(i,1)],...
    %         [robot(2),obs(i,2)],'-b') % Rayos de interseccion
    % end
    % % plot([robot(1),robot(1)-6*sin(angulos(4))],...
    % %     [robot(2),robot(2)+6*cos(angulos(4))],'-b') % Rayos fuera de
    % %     rango
end