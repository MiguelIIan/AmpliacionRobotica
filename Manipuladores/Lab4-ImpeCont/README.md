# Lab Session 4: Impedance Control

<div align="justify">
  En esta sesión de prácticas se ha implementado un controlador cartesiano de impedancia siguiendo el esquema de la Imagen 1.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/fca70db7-6008-46b9-9d43-8320462a0ad2">
  <br>
  <i>Imagen 1.- Esquema del controlador cartesiano de impedancia.</i>
</p>

## Implementación del controlador

<div align="justify">
  El controlador tiene dos niveles: El primero, y mas cercano al manipulador, es la compensación de la dinámica a nivel de articulaciones y, a un nivel superior a este, está el controlador cartesiano de impedancia. Este último, necesita transformar de variables articulares a cartesianas usando el modelo cinemático y las diferenciales cinemáticas de primer y segundo orden. Para ello, se va a crear el nodo "impedance_controller.cpp".
<br><br>
  
  El nodo del controlador se subscribirá a los tópicos que dan la pose de equilibrio, el estado de las articulaciones y los esfuerzos externos al manipulador. Además, este nodo obtendrá los parámetros de las masas (**M**), los amortiguamientos (**B**) y las  rigideces (**K**) del manipulador del archivo "impedance_params.yaml". Conociendo ya las conexiones del nodo, se puede seguir el proceso que lleva a cabo el controlador para calcular las aceleraciones articulares que va a seguir el robot.
<br><br>
  
  1. Se hace un chequeo de que se reciben correctamente los mensajes de las subscripciones del nodo
<br><br>

  2. El controlador calcula la posición del manipulador en variables cartesianas a partir de sus posiciones articulares. Para ello, se usa la ecuación 1.
</div>

$$
\begin{align}
x = 
\begin{bmatrix}  
l_1·cos(q_1) + l_2·cos(q_1 + q_2) \\
l_1·sin(q_1) + l_2·sin(q_1 + q_2)
\end{bmatrix} \qquad\qquad (1)
\end{align}
$$

<div align="justify">
  Este cálculo, dentro del código, se puede apreciar en la Imagen 2. 
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/b2d173c6-b4da-439c-bfcb-f1669c053d68">
  <br>
  <i>Imagen 2.- Código que calcula la posición cartesiana a partir de la articular.</i>
</p>

<div align="justify">
  3. El controlador calcula ahora las nuevas matrices jacobianas en función de los nuevos datos. Las ecuaciones 2 y 3 muestran este cálculo.
</div>

$$
\begin{align}
J(q) = 
\begin{bmatrix}  
-l_1·sin(q_1) - l_2·sin(q_1 + q_2) & -l_2·sin(q_1 + q_2)\\
l_1·cos(q_1) + l_2·cos(q_1 + q_2) & l_2·cos(q_1 + q_2)
\end{bmatrix} \qquad\qquad (2)\\
\end{align}
$$

$$
\begin{align}
J'(q,q') = 
\begin{bmatrix}  
-l_1·cos(q_1)q_1' - l_2·cos(q_1 + q_2)q_1' & -l_2·cos(q_1 + q_2)q_2'\\
-l_1·sin(q_1)q_1' - l_2·sin(q_1 + q_2)q_1' & -l_2·sin(q_1 + q_2)q_2'
\end{bmatrix} \qquad\qquad (3)
\end{align}
$$

<div align="justify">
  En el código, el uso de estas matrices quedaría como en la Imagen 3.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/3cb164c8-d0c5-42a3-8fae-c43aebee6e38">
  <br>
  <i>Imagen 3.- Código que calcula las matrices Jacobiana y su derivada.</i>
</p>

<div align="justify">
  4. Una vez se ha calculado la matriz Jacobiana, se pueden obtener las velocidades cartesianas utilizando la cinemática diferencial de primer orden, es decir, la matriz Jacobiana. Para ello se usará la ecuación 4.
</div>

$$
\begin{align}
x' = J(q)·q' \qquad\qquad (4)
\end{align}
$$

<div align="justify">
  En la Imagen 4 se realiza este cálculo en el código del nodo.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/d755d0d2-adec-44f3-94ad-0a3b3cf1e16b">
  <br>
  <i>Imagen 4.- Código que calcula las velocidades cartesianas del manipulador.</i>
</p>

<div align="justify">
  5. Ahora que se ha calculado todo lo necesario, se calculan las aceleraciones cartesianas necesarias para que nuestro manipulador realice el comportamiento de impedancia que se desea. Para ello, se van a utilizar las matrices de masas, amortiguamientos y rigideces. Suponiendo que la única contribucióna al movimiento del manipulador viene dado solo por modelo de impedancia mecánica, la ecuación que se usará será la 5.
</div>

$$
\begin{align}
x_d'' = M^{-1}·(-B·x_{er}' - K·x_{er} + f_{ext}) \qquad\qquad (5)\\
**donde** \qquad x_{er}' = x' - x_d' \qquad **y** \qquad x_{er} = x - x_d
\end{align}
$$

<div align="justify">
  En el Imagen 5 se puede ver la implementación de este cálculo en el código.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/7df39dfd-3c86-4b86-a609-c71d31458ad3">
  <br>
  <i>Imagen 5.- Código que calcula las aceleraciones cartesianas deseadas.</i>
</p>

<div align="justify">
  6. LLegados a este punto ya se puede calcular las aceleraciones cartesianas que se le enviarán al manipulador para que empiece a moverse usando la cinemática diferencial de segundo orden. Para ello, se usará la ecuación 6. 
</div>

$$
\begin{align}
q'' = J(q)^{-1}·[x_d'' - J'(q,q')·q'] \qquad\qquad (6)
\end{align}
$$

<div align="justify">
  En la Imagen 6 se ve el cálculo dentro del código.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/002415d1-03ab-4920-948f-85833cfac5ab">
  <br>
  <i>Imagen 6.- Código que calcula las aceleraciones articulares deseadas.</i>
</p>

<div align="justify">
  7. Por último, el nodo comprueba que se ha llevado a cabo correctamente los cálculos antes de enviar las aceleraciones articulares calculadas.
  <br><br>

  De esta manera, se concluye la implementación del controlador de impedancia en nuestro código, ahora, se van a realizar las simulaciones y los experimentos para ver como funciona ante distintos estímulos. 
</div>

## Experimento 1: Aplicando fuerzas virtuales al manipulador


![Captura desde 2025-06-12 13-58-09](https://github.com/user-attachments/assets/5e790ba1-b1b3-40e5-89ab-449793b32f44)

![Captura desde 2025-06-12 13-58-01](https://github.com/user-attachments/assets/ca28135c-897d-4809-bc56-69bc363c4159)

*Notar que el video y las gráficas son pruebas distintas.
















