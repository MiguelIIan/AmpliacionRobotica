clear all
close all
clc

% Entorno cerrado definido por una lista de puntos
y=[0 30 30 0 0]';
x=[0 0 10 10 0]';

% Posición y orientación del vehiculo
x0= 5;
y0= 5;
phi0= 1; % Entre -pi y pi

rangos= laser2D(x, y, x0, y0, phi0);

dibujaBarrido(x, y, x0, y0, phi0, rangos);

figure

dibujaBarrido(x, y, 0, 0, 0, rangos);


