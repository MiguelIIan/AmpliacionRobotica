%% funcion para simular el sensor
function [obs]=Lidar(robot, mapa, angulos, max_rango)
    obs=rayIntersection(mapa,robot,angulos, max_rango);
end