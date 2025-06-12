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

<div align="justify">
  En este primer experimento se va a observar el comportamiento del manipulador al actuar sobre él distintas fuerzas externas. Este comportamiento se puede apreciar en el Video 1 así como en las Imagenes 7 y 8.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/adc9ed0d-585f-4be8-a1a1-d3279c2105c0">
  <br>
  <i>Video 1.- Respuesta del manipulador a fuerzas externas.</i>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/5e790ba1-b1b3-40e5-89ab-449793b32f44">
  <br>
  <i>Imagen 7.- Comparación entre la fuerza externa y el movimiento del robot en el eje x.</i>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/ca28135c-897d-4809-bc56-69bc363c4159">
  <br>
  <i>Imagen 8.- Comparación entre la fuerza externa y el movimiento del robot en el eje y.</i>
</p>
<div align="right">
  
  ***Nota importante: El video y las gráficas no se corresponden entre si.***
  <br><br>
</div>

### Efectos de cambiar los parametros de la impedancia

<div align="justify">
  
  Ahora se va a probar a cambiar los parámetros **M**, **B** y **K** de la impedancia. Al cambiar la matriz **B** de su valor normal de [100,0,0,100] a [10,0,0,10] el comportamiento que obtenemos es mucho menos suave, ya que, al reducir el amortiguamiento, el manipulador opone menor resistencia a las fuerzas, lo que lo puede llevar muy facilmente a configuraciones degeneradas. Esto se puede observar en el Video 2 y la Imagen 9. Al cambiar la matriz **M** a un valor muy grande, cambiando de [1,0,0,1] inicialmente a [100,0,0,100], obtenemos un comportamiento sobreoscilante, muy inercial, debido a que el controlador cree q las masas son mucho mayores de lo que son en realidad. Esto se ve en el Video 3 y la Imagen 10. Por otro lado, modificar la matriz **K** no genera grandes diferencias que se puedan apreciar en la visualización.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/d22a6201-3a63-4880-aca9-c80a1442ef1d">
  <br>
  <i>Video 2.- Respuesta del manipulador a fuerzas externas con la nueva matriz de amortiguamiento.</i>
</p>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/1be93869-7473-425c-bc58-1ddc90c62586">
  <br>
  <i>Imagen 9.- Respuesta del manipulador a fuerzas externas con la nueva matriz de amortiguamiento.</i>
</p>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/964965c5-d16a-4664-b9eb-6c28745559f9">
  <br>
  <i>Video 3.- Respuesta del manipulador a fuerzas externas con la nueva matriz de masas.</i>
</p>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/d620dc73-de1b-4409-9dfb-46c70d018824">
  <br>
  <i>Imagen 10.- Respuesta del manipulador a fuerzas externas con la nueva matriz de masas.</i>
</p>
<div align="right">
  
  ***Nota importante: El video y las gráficas no se corresponden entre si.***
  <br><br>
</div>

<div align="justify">
  Se puede ver en la Imagen 10 que, al moverse con la inercia de esa supuesta gran masa, el manipulador oscila hasta el punto de acabar llegando a una singularidad. 
</div>

### Efectos de tener alta impedancia en X y baja impedancia en Y

<div align="justify">
  
  A continuación, en el Video 4, se comprobará el comportamiento del manipulador al tener distintas impedancias. Se puede observar como en el eje x, el manipulador se resiste más al movimiento, mientras que en el eje y se mueve demasiado rápido y oscila, lo que le acaba llevando a una singularidad.
</div>

<p align="center">
  <img src="https://github.com/user-attachments/assets/cb28c122-592c-49df-81d1-efa8e56b9e57">
  <br>
  <i>Video 4.- Respuesta del manipulador a fuerzas externas con ejes con distintas impedancias.</i>
</p>

### Preguntas

<div align="justify">
  
  **¿Las fuerzas ejercidas en el eje X generan movimiento en el eje Y?**

  Así es, la fuerza ejercida tanto en el eje X como en el eje Y generan movimiento en el eje perpendicular. Esto se debe a la forma del manipulador y que no estamos controlando la posición cartesiana, es decir, cuando aplicamos una fuerza, al estar el manipulador formado por articulaciones de revolución, el movimiento es circular al estar pivotando sobre la articulación. Esto va a ser lo que genere un movimiento en el eje perpendicular al de la fuerza.

  **¿Cómo podría resolverse este problema?**

  Este comportamiento se podría minimizar controlando la posición cartesiana del efector final en función de las fuerzas externas en vez de la articular. En esta práctica, hemos hecho un control de impedancias cogiendo como información de partida la posición articular del manipulador, y en función de la impedancia generada, volver a la posición de equilibrio más o menos rápido. Si quisiesemos resolver los movimientos en los ejes perpendiculares a las aplicaciones de las fuerzas, tendremos que controlar la impedancia con la posición cartesiana de partida.
</div>

## Experimento 2: Cambiar la posición de equilibrio

<div align="justify">
  En este experimento se va a ir cambiando la posición de equilibrio del manipulador para ver como responde. En el Video 5 se va a ver como el punto de equilibrio va cambiando y el manipulador lo va siguiendo, pero llega un punto en el que para ir al nuevo punto de equilibrio, el manipulador debe pasa por un sitio donde no llega, es decir, una singularidad de la posición, lo que genera ese comportamiento erratico de la simulación.
</div>

<p align="center">
  <img src="https://github.com/user-attachments/assets/55d6f945-839e-4c46-a3e5-26433c0164ac">
  <br>
  <i>Video 5.- Respuesta del manipulador a cambios de la posición de equilibrio.</i>
</p>

<div align="justify">
  Aunque no sea el mismo experimento, en la Imagen 11 se puede ver como el manipulador sigue las posiciones de equilibrio indicadas hasta que, para llegar a una de ellas, el manipulador debería pasar por una trayectoria a la que no llega, esto es una sigularidad en la posición y provoca que la simulación colapse. 
</div>

<p align="center">
  <img src="https://github.com/user-attachments/assets/370fb208-a826-4db2-b8e5-42a451d738bf">
  <br>
  <i>Imagen 11.- Respuesta del manipulador a cambios de la posición de equilibrio.</i>
</p>









