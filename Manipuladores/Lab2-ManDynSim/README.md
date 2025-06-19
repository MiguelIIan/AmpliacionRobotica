# Lab Session 2: Manipulator Dynamics Simulation

<div align="justify">
En esta práctica, se ha llevado a cabo la simulación de un manipulador de dos grados de libertad siendo afectado por la fuerza de la gravedad gracias al cálculo de su dinámica.
</div>

### Probando el paquete del manipulador de la UMA

<div align="justify">
    
Antes de empezar, se va a realizar una prueba del funcionamiento del paquete, para ello, se han ejecutado en cada terminal uno de los siguientes comandos.

</div>

    ros2 launch uma_arm_description uma_arm_visualization.launch.py

    ros2 run joint_state_publisher_gui joint_state_publisher_gui
    

<div align="justify">
    
Estos comandos abren la visualización de los nodos de ROS2 en RViz y crean una ventana con la que poder definir los ángulos de las articulaciones del manipulador. Esto se ve en la Imagen 1.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/879487f0-bdf3-4d97-bb5a-4206a627ebd6">
  <br>
  <i>Imagen 1.- Visualización en RViz del manipulador.</i>
</p>

### Simulando las dinamicas del manipulador

<div align="justify">
    
Primero de todo, se necesita clonar en el "workspace" el paquete "uma_arm_control". Este paquete se va a encargar de realizar los cálculos de la dinámica del manipulador, teniendo en cuenta las fuerzas que actuen sobre él, que en este caso, es solo la fuerza de la gravedad. Teniendo todo preparado, solo falta realizar los cálculos de la dinámica del robot y lanzar los nodos para comprobar el funcionamiento. Primero es necesario definir como es la dinámica del manipulador en la ecuación 1.
</div>

$$
\begin{align}
M(q)q'' + C(q,q')q' + F_bq' + g(q) = \tau + \tau_{ext}  \qquad\qquad (1)
\end{align}
$$

<div align="justify">
    
Ahora, hay que despejar las aceleraciones articulares, ya que son los datos que se le van a enviar al manipulador para que sepa como debe comportarse. Esto se ve en la ecuación 2.
</div>

$$
\begin{align}
q'' = M^{-1}(q) · [\tau + \tau_{ext} - C(q,q')q' - F_bq' - g(q)]  \qquad\qquad (2)
\end{align}
$$

<div align="justify">
    
Lo siguiente por hacer consiste en calcular cada una de las partes de la ecuación 2. En las ecuaciones 3, 4, 5 y 6 están los cálculos de cada una de estas matrices.
</div>

$$
\begin{align}
M(q) = 
\begin{bmatrix}  
m_1·l_1^2+m_2⋅(l_1^2+2⋅l_1⋅l_2⋅cos(q_2)+l_2^2) & m_2⋅(l_1⋅l_2⋅cos(q_2)+l_2^2) \\
m_2⋅(l_1⋅l_2⋅cos(q_2)+l_2^2) & m_2·l_2^2
\end{bmatrix} \qquad\qquad (3)
\end{align}
$$

$$
\begin{align}
C(q,q')q' = 
\begin{bmatrix}  
−m_2⋅l_1⋅l_2⋅sin(q_2)⋅(2⋅q_1'⋅q_2'+q_2'^2) \\
m_2⋅l_1⋅l_2⋅q_1^2⋅sin(q_2)​
\end{bmatrix} \qquad\qquad (4)
\end{align}
$$

$$
\begin{align}
F_b = 
\begin{bmatrix}  
b_1 & 0 \\
0 & b_2
\end{bmatrix} \qquad\qquad (5)
\end{align}
$$

$$
\begin{align}
g(q) = 
\begin{bmatrix}  
(m_1+m_2)⋅l_1⋅g⋅cos(q_1)+m_2⋅g⋅l_2⋅cos(q_1+q_2) \\
m_2⋅g⋅l_2⋅cos(q_1+q_2)​
\end{bmatrix} \qquad\qquad (6)
\end{align}
$$

<div align="justify">
    
Ahora, es necesario calcular el jacobiano, en la ecuación 7, para incluir las fuerzas externas que actúen sobre nuestro manipulador
</div>

$$
\begin{align}
J(q) = 
\begin{bmatrix}  
−l_1⋅sin(q_1)−l_2⋅sin(q_1+q_2) & ​−l_2⋅sin(q_1+q_2) \\
l_1⋅cos(q_1)+l_2⋅cos(q_1+q_2) & l_2⋅cos(q_1+q_2)​
\end{bmatrix} \qquad\qquad (7)
\end{align}
$$

<div align="justify">
    
De esta manera se puede calcular el torque externo ejercido sobre nuestro manipulador en la ecuación 8.
</div>

$$
\begin{align}
\tau_{ext} = J(q)^T · F_{ext} \qquad\qquad (8)
\end{align}
$$

<div align="justify">
    
Teniendo todos los cálculos claros, ya se puede rellenar la parte del código vacío como se ve en la Imagen 2.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/0a299a09-3e5e-420c-b9cf-3f00a929c8f7">
  <br>
  <i>Imagen 2.- Código que usa la dinámica del manipulador para calcular las aceleraciones articulares.</i>
</p>

<div align="justify">
    
Como se está implementando un sistema discreto, las velocidades y posiciones articulares se calcularán en las ecuaciones 9 y 10.
</div>

$$
\begin{align}
q' = \int q'' dt \Rightarrow q_{k+1}' = q_k' + q_{k+1}'' · \Delta t \qquad\qquad (9)
\end{align}
$$

$$
\begin{align}
q = \int q' dt \Rightarrow q_{k+1} = q_k + q_{k+1}' · \Delta t \qquad\qquad (10)
\end{align}
$$

<div align="justify">
    
Al introducirlo en el código, este quedaría como se puede ver en la Imagen 3.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/2dba289d-b9d4-466a-8e6e-a4d2f19f89c6">
  <br>
  <i>Imagen 3.- Código con el cálculo de las velocidades y posiciones articulares.</i>
</p>

### Ejecutando la simulación de la dinámica

<div align="justify">
    
Ejecutando cada uno de los siguientes comandos en una terminal, se inician los nodos con los que se van a visualizar y simular la dinámica del manipulador
</div>

    ros2 launch uma_arm_description uma_arm_visualization.launch.py

    ros2 launch uma_arm_control uma_arm_dynamics_launch.py

<div align="justify">
Al ejecutar los nodos, se obtiene el comportamiento del Video 1.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/8be9846c-8cb4-4bd4-a903-5b1ee743b7b1">
  <br>
  <i>Video 1.- Comportamiento dinámico del manipulador de dos grados de libertad.</i>
</p>

<div align="justify">
También se puede usar el comando "rqt_graph" para visualizar los nodos que se están ejecutando y los topics con los que se comunican. Esto se puede ver en la Imagen 4
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/a51f8d97-4184-470b-bdf7-55538432fdfa">
  <br>
  <i>Imagen 4.- Visualización de los nodos y los topics que se están ejecutando.</i>
</p>

### Representación gráfica

<div align="justify">
Se pueden visualizar las variables de los mensajes entre los nodos utilizando las "rosbags" y "Plotjuggler". Las "rosbags" consisten en un método para guardar todos los mensajes ROS2 que se hayan producido en un tiempo determinado, y "Plotjuggler" permite graficar las variables que se encuentran dentro de estos mensajes. El resultado de realizar este proceso se ve en la Imagen 5.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/a89a1fa4-cf79-44bd-9935-f5cb2787e843">
  <br>
  <i>Imagen 5.- Visualización de las variables de posición, velocidad y aceleración articulares de cada una de las articulaciones, haciendo uso de "Plotjuggler".</i>
</p>


### Experimentos

<div align="justify">
Se van a realizar varios experimentos, cambiando los parametros de la dinámica del manipulador. Los resultados de estos experimentos se pueden apreciar en las Imagenes 6, 7 y 8.
</div>


<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/dd8e7e24-5c49-4315-b05e-2789fc8679d8">
  <br>
  <i>Imagen 6.- Experimento aumentando las masas de los eslabones.</i>
</p>

<div align="justify">
    
En la Imagen 6 se ha cambiado $m_1$ a 5 kilogramos en vez de 3 y $m_2$ a 3 kilogramos en vez de 2. Se puede comprobar con la Imagen 5 que el manipulador oscila más y más rápido, esto se debe al aumento de masa que aumenta la fuerza gravitatoria que actua sobre el manipulador.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/394b3ad7-9c53-4dfd-b2a9-afeec33875eb">
  <br>
  <i>Imagen 7.- Experimento aumentando la amortiguación de las articulaciones.</i>
</p>

<div align="justify">
    
En la Imagen 7 se ha cambiado $b_1$ a 10 en vez de 5 y $b_2$ a 10 kilogramos en vez de 5. Se puede comprobar con la Imagen 5 que el manipulador oscila mucho menos y más lento, esto se debe al aumento de la fuerza de amortiguación, frenando el movimiento producido por la gravedad.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/3428620f-c615-4bd0-90ca-8b6649fbd3ce">
  <br>
  <i>Imagen 8.- Experimento reduciendo el valor de la gravedad.</i>
</p>

<div align="justify">
    
En la Imagen 8 se ha cambiado la gravedad de la normal de la tierra a 5 m/s. Comparando con la Imagen 5 se puede observar que el manipulador se mueve mucho más lento, esto se debe a la disminución de la fuerza gravitatoria que tira del manipulador hacia abajo.
</div>



