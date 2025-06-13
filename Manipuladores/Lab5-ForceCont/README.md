# Lab Session 5: Force Control

## Introducción

<div align="justify">
En esta práctica, se va a simular el comportamiento dinámico de un manipulador RR bajo un control de fuerzas con un bucle de posición interna. Al igual que en las otras sesiones, el comportamiento del manipulador se ha linealizado siguiendo la ley de control de la ecuación 1. Es por eso que el manipulador sigue un comportamiento puramente cinemático expresado por la ecuación 2.

$$
\begin{align}
\tau = M(q)·q_d'' + C(q,q')·q' + F_b·q' + g + J^T(q)·f_e \qquad\qquad (1)\\
\\
q'' = J(q)'·[x'' - J(q,q')'·q'] \qquad\qquad (2)
\end{align}
$$

  La idea de esta práctica es implementar la ley de control de fuerzas que se puede contemplar en la Imagen 2, siendo la Imagen 1 el control de posición del manipulador que se vió en la sesión anterior.

<p align="center">
  <img src="https://github.com/user-attachments/assets/a59b9bcb-51fa-4215-ab63-b2b47fc65119">
  <br>
  <i>Imagen 1.- Esquema del control de posición del manipulador.</i>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/f8bffa71-abb4-4905-8f6f-3b780f8bf1ac">
  <br>
  <i>Imagen 2.- Esquema del controlador de fuerzas.</i>
</p>

  La expresión matemática de este esquema sería la ecuación 3.

$$
\begin{align}
M_d·x_e'' + K_D·x_e' + K_P·(I_3 + C_F·K)·x_e = K_P·C_F·(K·x_r + f_d) \qquad\qquad (3)\\
\end{align}
$$

  En esta ecuación, vemos las matrices **$M_d$**, **$K_P$** y **$K_D$** que representan la matriz de masas, la de amortiguamiento y la de rigidez, respectivamente, que pertenecen al control de posición del manipulador. Este control, va a tener como entrada una posición en función de las fuerzas. En la ecuación 4 se puede ver el cálculo de esta posición.

$$
\begin{align}
x_F = C_F·(f_d - f_e) \qquad\qquad (4)\\
\end{align}
$$
  
  Donde $C_F$ es una constante proporcional, por ahora, $f_d$ es la fuerza de referencia y $f_e$ es la fuerza que el manipulador ejerce sobre una superficie elástica. Esta última viene dada por la expresión de la ecuación 5.

$$
\begin{align}
f_e = K·(x_e - x_r) \qquad\qquad (5)\\
\end{align}
$$


  Teniendo todo esto en cuenta, ahora se va a comenzar a dar valores a las matrices para poder realizar la simulación.
</div>

## Simular control P

<div align="justify">
Para llevar a cabo la simulación, se van a tomar los siguientes valores:  


$$
\begin{align}
x_r = [1.2,0.7] m \\
f_d = [10,0] N \\
x_{e,initial} = [1.3,0.7] m \\
\\
K = 
\begin{bmatrix}  
1000 & 0 \\
0 & 0
\end{bmatrix},
C_F = 
\begin{bmatrix}  
0.05 & 0 \\
0 & 0
\end{bmatrix} \\
\\
M_d = 
\begin{bmatrix}  
1000 & 0\\
0 & 1000
\end{bmatrix},
K_D = 
\begin{bmatrix}  
5000 & 0\\
0 & 5000
\end{bmatrix}, 
K_P = 
\begin{bmatrix}  
5000 & 0\\
0 & 5000
\end{bmatrix}
\end{align}
$$

Introduciendo todos estos valores en el esquema de Simulink, en la Imagen 3, vemos un comportamiento, en la Imagen 4, extraño, ya que la fuerza a la salida se estabiliza en una fuerza muy distinta a la de referencia. Esto se debe a que el controlador $C_F$ es proporcional y no corrige el error en régimen permanente.

<p align="center">
  <img src="https://github.com/user-attachments/assets/69eab521-f783-453f-bd18-27e75e872882">
  <i>Imagen 3.- Esquema del control de fuerzas implementado en Simulink.</i>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/8fbda213-7b22-4623-b348-bc71308b686f">
  <i>Imagen 4.- Fuerza del manipulador a la salida del control de fuerza.</i>
</p>

En la Imagen 4 se puede observar como, a causa del controlador proporcional, la fuerza a la salida vale $-13.8 N$ cuando la fuerza de referencia valía $10 N$.

Si se mira con detenimiento la salida del controlador de la posición, en la Imagen 5, se observa que, el manipulador, no se mantiene en su posición Y, sino que cae hasta el "suelo". 

<p align="center">
  <img src="https://github.com/user-attachments/assets/0dc6644c-942f-44f0-8ac0-091689899c6f">
  <i>Imagen 5.- Comportamiento en X e Y del manipulador.</i>
</p>

Esto se debe a que la fuerza que se le está enviando solo se ejerce en el eje X, lo que produce que el controlador proporcional $C_F$ dé como salida una posición donde la componente Y vale 0. Esto produce que el controlador de posición reciba como punto de destino [X,0] y mueva el manipulador hacia esa posición.
</div>

## Simular control PI

<div align="justify">

Como se ha visto anteriormente, el control P no es capaz de reducir el error en régimen permanente. Es por eso que se va a cambiar el control proporcional $C_F$ a un control proporcional-integral utilizando la ecuación 6.

$$
\begin{align}
C_F = K_F + K_I\int(·)dt \qquad\qquad (6)\\
\\
donde \qquad 
K_F = 
\begin{bmatrix}  
0.03 & 0 \\
0 & 0
\end{bmatrix},
K_I = 
\begin{bmatrix}  
0.03 & 0 \\
0 & 0
\end{bmatrix}
\end{align}
$$

El nuevo esquema en Simulink se puede apreciar en la Imagen 6.

<p align="center">
  <img src="https://github.com/user-attachments/assets/ec75d288-7c5a-4672-95a6-0f924293b63b">
  <i>Imagen 6.- Esquema de Simulink usando un control de fuerzas PI.</i>
</p>

La pregunta viene ahora, ¿ha resuelto este cambio nuestros problemas?. Observese la Imagen 7 con la fuerza a la salida del sistema.

<p align="center">
  <img src="https://github.com/user-attachments/assets/828b454d-bb4f-49ef-9e51-23ad29329f7c">
  <i>Imagen 7.- Salida de fuerza del sistema.</i>
</p>

En la Imagen 7 se puede ver como el control, esta vez, si llega en régimen permanente a $10 N$, la fuerza de referencia del sistema. Veamos la posición en X e Y en la Imagen 8 para ver si también se ha solucionado ese problema.

<p align="center">
  <img src="https://github.com/user-attachments/assets/42528ae5-4fdf-458f-80a1-eac73acd7f5c">
  <i>Imagen 8.- Posición X e Y del manipulador.</i>
</p>

En la Imagen 8, al igual que en la 5, se puede ver como la coordenada Y sigue bajando hasta 0, esto se debe a que el problema no radica en el controlador, si no que al considerar fuerzas solo en el eje X, la posición en Y acaba valiendo 0, por lo que el controlador de posición simplemente sigue esa consigna y baja el manipulador hasta el suelo.

Este sistema, hablando de la salida en fuerza del manipulador, se podría mejorar ajustando los valores de las ganancias del controlador PI para que no oscilase tanto, o, se podría utilizar un controlador PID, ya que, con la ganancia derivativa, se reducirían en gran medida las sobreoscilaciones del sistema.

En conclusión, viendo las Imagenes 7 y 8, se puede destacar que el control PI de fuerzas nos permite corregir el error en regimen permanente y dar, a la salida, la fuerza de referencia, aunque, por otro lado, este cambio no ha conseguido corregir el comportamiento de caída del manipulador al ser un control de posición y no estar ejerciendo fuerzas en el eje Y. 

</div>
